/*
 * io-edge-hub 上位机 Modbus 主机 (Task 5)
 *
 * 手撸 Modbus TCP + RTU 主机, 不依赖第三方 libmodbus. 支持 FC01/02/03/04/05/06/16.
 *   - TCP: 502 端口, MBAP 帧 [tid 2B BE][pid 2B=0][len 2B BE][uid 1B][PDU...]
 *   - RTU: PC 串口 8N1, ADU [addr 1B][PDU][CRC16-Modbus 2B LE]
 *
 * 同步 req/resp 模型: 公共 mb_transact() 负责打包 ADU + 发送 + 接收 + 拆包, 各 FC
 * 包装只组装/解析 PDU. WSAStartup 由 main.c 启动时调用一次, 本模块不再重复.
 *
 * 协议权威源:
 *   - applications/io-edge-hub/src/modbus/tcp.c (Modbus TCP 服务端)
 *   - applications/io-edge-hub/src/modbus/rtu.c (Modbus RTU 服务端)
 *   - applications/io-edge-hub/src/modbus/function.c (FC 回调)
 *
 * task-5-brief 的 mb_transact RTU 分支留了 return 0 占位, 此处按附录逻辑完整实现:
 * 按 fc 分支读取响应 + CRC 校验 + exception 解析, 不留占位.
 */
#include "modbus_client.h"
#include <stdio.h>
#include <string.h>

#pragma comment(lib, "ws2_32.lib")

struct MbClient {
	MbTransport transport;
	bool connected;
	uint8_t unit_id;
	bool last_no_resp;        /* 上次事务失败是否为 "完全无响应" (见头文件) */
	/* TCP */
	SOCKET sock;
	uint16_t tcp_tid;        /* MBAP transaction id, 自增 */
	/* RTU */
	HANDLE hCom;
	uint32_t baud;
	char last_error[128];
};

/* ================================================================
 * CRC16-Modbus (poly 0xA001, init 0xFFFF, 反向)
 *
 * Modbus RTU 标准校验: 多项式 0xA001 是 0x8005 的位反转 (reflected), 初值 0xFFFF,
 * 逐字节 LSB 先处理. CRC 附加到 ADU 末尾时低字节在前 (little-endian).
 * ================================================================ */

static uint16_t crc16_modbus(const uint8_t *data, int len)
{
	uint16_t crc = 0xFFFF;
	for (int i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
		}
	}
	return crc;
}

/* ================================================================
 * RTU 串口读取 helper
 *
 * rtu_read_n: 循环 ReadFile 直到读满 n 字节或超时 (由 COMMTIMEOUTS 控制).
 * 返回实际读到的字节数. 串口 ReadFile 在重叠模式 NULL 时受 COMMTIMEOUTS 约束:
 * ReadTotalTimeoutConstant=1000ms 总超时, ReadIntervalTimeout=50ms 帧间隙.
 * ================================================================ */

static int rtu_read_n(MbClient *m, uint8_t *buf, int n)
{
	int got = 0;
	while (got < n) {
		DWORD rd = 0;
		BOOL ok = ReadFile(m->hCom, buf + got, (DWORD)(n - got), &rd, NULL);
		if (!ok || rd == 0) {
			break;   /* 超时或错误, 停止 */
		}
		got += (int)rd;
	}
	return got;
}

/* RTU 3.5 字符帧间隔 (ms): max(1, 35000/baud). 9600bps≈4ms, 115200bps≈1ms.
 * 用于发送请求后等待从机开始回复前的间隔 (避免请求尾与回复头粘连). */
static int rtu_char_time_ms(uint32_t baud)
{
	int ms = (int)(35000u / baud);
	return ms < 1 ? 1 : ms;
}

/* TCP 错误码是否表示连接已死亡 (对端关闭/RST/本地不可用). 超时(WSAETIMEDOUT)等
 * 只代表对方未及时回复, 不算断开. 返回 true 时调用方应置 m->connected=false. */
static bool tcp_is_dead_error(int err)
{
	switch (err) {
	case WSAECONNRESET:
	case WSAECONNABORTED:
	case WSAENOTCONN:
	case WSAENETRESET:
	case WSAESHUTDOWN:
		return true;
	default:
		return false;
	}
}

/* ================================================================
 * PDU 传输层 mb_transact
 *
 * 入: pdu[0..pdulen-1] 含 fc. 出: out_pdu[0..] 含 fc, 长度为返回值 (含 fc); 0=失败.
 * 异常响应 (fc|0x80) 填 last_error 后返回 0 (调用者见 0 即失败).
 *
 * TCP: MBAP 帧 [tid 2B BE][pid 2B=0][len 2B BE][uid 1B][PDU...]; 响应先读 6B 头
 *      拿 len, 再读 len 字节 (uid+PDU), 去掉 uid 得 PDU.
 * RTU: ADU [addr 1B][PDU][crc 2B LE]; 响应按 fc 分支读取 (见下方详注).
 * ================================================================ */

static int mb_transact(MbClient *m, const uint8_t *pdu, int pdulen,
                       uint8_t *out_pdu, int out_cap)
{
	m->last_no_resp = false;
	if (m->transport == MB_TCP) {
		/* ===== TCP 分支 ===== */
		uint8_t adu[260];
		uint16_t tid = m->tcp_tid++;
		uint16_t len = (uint16_t)(pdulen + 1);   /* uid + pdu */
		adu[0] = (uint8_t)(tid >> 8); adu[1] = (uint8_t)tid;
		adu[2] = 0; adu[3] = 0;                   /* protocol id = 0 (Modbus) */
		adu[4] = (uint8_t)(len >> 8); adu[5] = (uint8_t)len;
		adu[6] = m->unit_id;
		memcpy(adu + 7, pdu, pdulen);
		int adulen = 7 + pdulen;

		if (send(m->sock, (const char *)adu, adulen, 0) != adulen) {
			if (tcp_is_dead_error(WSAGetLastError())) {
				sprintf(m->last_error, "连接已断开 (发送失败)");
				m->connected = false;
			} else {
				sprintf(m->last_error, "TCP 发送失败");
			}
			return 0;
		}

		/* 收 MBAP 头 6B, 处理 partial recv */
		int n = 0;
		while (n < 6) {
			int r = recv(m->sock, (char *)adu + n, 6 - n, 0);
			if (r == 0) {
				sprintf(m->last_error, "对端已断开连接");
				m->connected = false;
				return 0;
			}
			if (r < 0) {
				if (tcp_is_dead_error(WSAGetLastError())) {
					sprintf(m->last_error, "连接已断开 (接收失败)");
					m->connected = false;
				} else {
					sprintf(m->last_error, "TCP 响应超时");
					m->last_no_resp = true;
				}
				return 0;
			}
			n += r;
		}
		uint16_t rlen = ((uint16_t)adu[4] << 8) | adu[5];   /* uid + pdu 字节数 */

		/* 直接读入 out_pdu (临时含 uid), 再左移去掉 uid */
		if (rlen > out_cap) {
			sprintf(m->last_error, "TCP 响应过长");
			return 0;
		}
		int got = 0;
		while (got < rlen) {
			int r = recv(m->sock, (char *)out_pdu + got, rlen - got, 0);
			if (r == 0) {
				sprintf(m->last_error, "对端已断开连接");
				m->connected = false;
				return 0;
			}
			if (r < 0) {
				if (tcp_is_dead_error(WSAGetLastError())) {
					sprintf(m->last_error, "连接已断开 (接收失败)");
					m->connected = false;
				} else {
					sprintf(m->last_error, "TCP 响应中断");
				}
				return 0;
			}
			got += r;
		}
		/* out_pdu[0] = uid, 实际 PDU 从 out_pdu[1] 开始. 左移去掉 uid. */
		memmove(out_pdu, out_pdu + 1, got - 1);
		return got - 1;
	} else {
		/* ===== RTU 分支 (完整实现, 无占位) =====
		 *
		 * 1. 组 ADU = [unit_id][pdu...][crc16 LE], WriteFile 发出.
		 * 2. Sleep(3.5 字符间隔, max(1,35000/baud) ms).
		 * 3. 按 fc 分支读响应:
		 *    - 异常 (fc&0x80): 已读 addr+fc(2B), 再读 exception_code(1)+crc(2) = 3B,
		 *      校验整帧 CRC, 填 last_error, 返回 0.
		 *    - FC01/02/03/04 正常: 已读 addr+fc(2B), 再读 1B byte_count(bc),
		 *      再读 bc+2B (data + crc), 校验 CRC, 写 [fc][bc][data...] 到 out_pdu,
		 *      返回 2+bc.
 *    - FC05/06/16 正常: 响应 ADU 固定 8B (addr+fc+4B echo+CRC), 已读 2B, 再读 6B,
 *      校验 CRC, 写 5B PDU (fc+4B) 到 out_pdu, 返回 5.
		 * 4. 任何 ReadFile 不足 / CRC 错误 → 填 last_error 返回 0. */

		uint8_t adu[260];
		adu[0] = m->unit_id;
		memcpy(adu + 1, pdu, pdulen);
		uint16_t crc = crc16_modbus(adu, 1 + pdulen);
		adu[1 + pdulen] = (uint8_t)(crc & 0xFF);       /* CRC 低字节在前 (LE) */
		adu[2 + pdulen] = (uint8_t)(crc >> 8);
		DWORD wr = 0;
		int adulen = 3 + pdulen;
		if (!WriteFile(m->hCom, adu, (DWORD)adulen, &wr, NULL) || (int)wr != adulen) {
			sprintf(m->last_error, "RTU 发送失败");
			return 0;
		}

		/* 3.5 字符间隔 (近似: 35000/baud ms, 最小 1) */
		Sleep(rtu_char_time_ms(m->baud));

		/* 读前 2B: addr + fc. 不足即超时. */
		uint8_t hdr[2];
		if (rtu_read_n(m, hdr, 2) < 2) {
			sprintf(m->last_error, "RTU 响应超时");
			m->last_no_resp = true;
			return 0;
		}
		uint8_t fc = hdr[1];

		if (fc & 0x80) {
			/* ===== 异常响应 =====
			 * 帧 = [addr][fc|0x80][exception_code][crc_lo][crc_hi] = 5B.
			 * 已读 addr+fc(2B), 再读 exception_code(1) + crc(2) = 3B. */
			uint8_t rest[3];
			if (rtu_read_n(m, rest, 3) < 3) {
				sprintf(m->last_error, "RTU 异常响应不完整");
				return 0;
			}
			/* 校验 CRC: 整帧 = hdr(2) + rest(3) = 5B, crc 在 rest[1..2] */
			uint8_t frame[5];
			frame[0] = hdr[0]; frame[1] = hdr[1];
			frame[2] = rest[0]; frame[3] = rest[1]; frame[4] = rest[2];
			uint16_t calc = crc16_modbus(frame, 3);
			uint16_t recv_crc = (uint16_t)rest[1] | ((uint16_t)rest[2] << 8);
			if (calc != recv_crc) {
				sprintf(m->last_error, "RTU CRC 错误");
				return 0;
			}
			uint8_t ec = rest[0];
			switch (ec) {
			case 1: sprintf(m->last_error, "Modbus 异常: 非法功能码"); break;
			case 2: sprintf(m->last_error, "Modbus 异常: 非法地址"); break;
			case 3: sprintf(m->last_error, "Modbus 异常: 非法值"); break;
			case 4: sprintf(m->last_error, "Modbus 异常: 从机故障"); break;
			case 5: sprintf(m->last_error, "Modbus 异常: 确认"); break;
			case 6: sprintf(m->last_error, "Modbus 异常: 从机忙"); break;
			default: sprintf(m->last_error, "Modbus 异常 code=%d", ec); break;
			}
			return 0;
		} else if (fc == 0x01 || fc == 0x02 || fc == 0x03 || fc == 0x04) {
			/* ===== FC01/02/03/04 读响应 =====
			 * 帧 = [addr][fc][byte_count][data...][crc_lo][crc_hi].
			 * 已读 addr+fc(2B), 再读 1B byte_count(bc), 再读 bc+2B (data+crc). */
			uint8_t bc_buf[1];
			if (rtu_read_n(m, bc_buf, 1) < 1) {
				sprintf(m->last_error, "RTU 响应超时 (byte count)");
				return 0;
			}
			uint8_t bc = bc_buf[0];
			/* out_pdu 容量检查: fc + bc + bc(data) = 2 + bc 字节 */
			if (2 + bc > out_cap) {
				sprintf(m->last_error, "RTU 响应过长");
				return 0;
			}
			uint8_t tail[256];
			int need = bc + 2;   /* data(bc) + crc(2) */
			if (rtu_read_n(m, tail, need) < need) {
				sprintf(m->last_error, "RTU 响应不完整");
				return 0;
			}
			/* 校验 CRC: 整帧 = hdr(2) + bc(1) + data(bc) + crc(2) */
			uint8_t frame[260];
			frame[0] = hdr[0]; frame[1] = hdr[1]; frame[2] = bc;
			memcpy(frame + 3, tail, bc);
			uint16_t calc = crc16_modbus(frame, 3 + bc);
			uint16_t recv_crc = (uint16_t)tail[bc] | ((uint16_t)tail[bc + 1] << 8);
			if (calc != recv_crc) {
				sprintf(m->last_error, "RTU CRC 错误");
				return 0;
			}
			/* 写 out_pdu = [fc][bc][data...] */
			out_pdu[0] = fc;
			out_pdu[1] = bc;
			memcpy(out_pdu + 2, tail, bc);
			return 2 + bc;
		} else if (fc == 0x05 || fc == 0x06 || fc == 0x10) {
			/* ===== FC05/06/16 写响应 =====
			 * 帧 = [addr][fc][addr_hi][addr_lo][qty/value_hi][qty/value_lo][crc_lo][crc_hi]
			 *      = 8B 固定. 已读 addr+fc(2B), 再读 6B (4B echo + crc). */
			uint8_t rest[6];
			if (rtu_read_n(m, rest, 6) < 6) {
				sprintf(m->last_error, "RTU 写响应不完整");
				return 0;
			}
			uint8_t frame[8];
			frame[0] = hdr[0]; frame[1] = hdr[1];
			memcpy(frame + 2, rest, 6);
			uint16_t calc = crc16_modbus(frame, 6);   /* crc 在 frame[6..7] */
			uint16_t recv_crc = (uint16_t)rest[4] | ((uint16_t)rest[5] << 8);
			if (calc != recv_crc) {
				sprintf(m->last_error, "RTU CRC 错误");
				return 0;
			}
			/* 写 out_pdu = [fc][addr_hi][addr_lo][value/qty_hi][value/qty_lo] = 5 字节 PDU.
			 * (Modbus 规范: FC05/06/16 响应 PDU 均为 fc + 4B = 5B. brief 附录写 "7B PDU"
			 * 是把含 addr 的 ADU 误记为 PDU, 此处按规范返回 5B. 调用者只检查 resp[0]
			 * 不为异常, 长度不影响功能.) */
			if (5 > out_cap) {
				sprintf(m->last_error, "RTU 响应缓冲过小");
				return 0;
			}
			out_pdu[0] = fc;
			memcpy(out_pdu + 1, rest, 4);   /* addr_hi addr_lo value/qty_hi value/qty_lo */
			return 5;
		} else {
			sprintf(m->last_error, "RTU 未知功能码 0x%02X", fc);
			return 0;
		}
	}
}

/* ================================================================
 * 异常响应统一检查 (FC 包装用)
 *
 * mb_transact 返回的 PDU 已含 fc. 若 fc|0x80 为异常 — 但 TCP 路径异常响应的 fc
 * 在 out_pdu[0], RTU 路径异常已被 mb_transact 内部处理并返回 0. 因此 TCP 路径
 * 的异常在此统一捕获. 返回 true=正常可继续; false=已填 last_error.
 * ================================================================ */

static bool check_normal(MbClient *m, const uint8_t *resp, int n)
{
	if (n <= 0) return false;
	if (resp[0] & 0x80) {
		uint8_t ec = (n >= 2) ? resp[1] : 0;
		switch (ec) {
		case 1: sprintf(m->last_error, "Modbus 异常: 非法功能码"); break;
		case 2: sprintf(m->last_error, "Modbus 异常: 非法地址"); break;
		case 3: sprintf(m->last_error, "Modbus 异常: 非法值"); break;
		case 4: sprintf(m->last_error, "Modbus 异常: 从机故障"); break;
		case 5: sprintf(m->last_error, "Modbus 异常: 确认"); break;
		case 6: sprintf(m->last_error, "Modbus 异常: 从机忙"); break;
		default: sprintf(m->last_error, "Modbus 异常 code=%d", ec); break;
		}
		return false;
	}
	return true;
}

/* ================================================================
 * 生命周期
 * ================================================================ */

MbClient *MbClient_Create(void)
{
	return (MbClient *)calloc(1, sizeof(MbClient));
}

void MbClient_Destroy(MbClient *m)
{
	if (!m) return;
	MbClient_Disconnect(m);
	free(m);
}

const char *MbClient_GetLastError(MbClient *m)
{
	return m ? m->last_error : "NULL manager";
}

bool MbClient_LastNoResponse(const MbClient *m)
{
	return m ? m->last_no_resp : false;
}

bool MbClient_IsConnected(const MbClient *m)
{
	return m && m->connected;
}

MbTransport MbClient_GetTransport(const MbClient *m)
{
	return m ? m->transport : MB_TCP;
}

/* ================================================================
 * 连接管理
 * ================================================================ */

bool MbClient_ConnectTcp(MbClient *m, const char *ip, uint16_t port, uint8_t unit_id)
{
	if (!m) return false;
	if (m->connected) {
		MbClient_Disconnect(m);
	}
	m->sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m->sock == INVALID_SOCKET) {
		sprintf(m->last_error, "TCP socket 失败: %d", WSAGetLastError());
		return false;
	}

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);
	sa.sin_addr.s_addr = inet_addr(ip);

	/* 非阻塞 connect + select 限时 3s, 避免对不可达 IP 阻塞 ~21s 卡死 UI. */
	unsigned long nb = 1;
	ioctlsocket(m->sock, FIONBIO, &nb);
	int r = connect(m->sock, (struct sockaddr *)&sa, sizeof(sa));
	if (r == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK) {
		sprintf(m->last_error, "TCP 连接失败: %d", WSAGetLastError());
		closesocket(m->sock);
		m->sock = INVALID_SOCKET;
		return false;
	}
	if (r != 0) {
		fd_set wset;
		FD_ZERO(&wset);
		FD_SET(m->sock, &wset);
		struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
		int sr = select(0, NULL, &wset, NULL, &tv);
		if (sr <= 0) {
			sprintf(m->last_error, sr == 0 ? "TCP 连接超时 (3s)" : "TCP select 失败");
			closesocket(m->sock);
			m->sock = INVALID_SOCKET;
			return false;
		}
		int soerr = 0;
		int sl = sizeof(soerr);
		getsockopt(m->sock, SOL_SOCKET, SO_ERROR, (char *)&soerr, &sl);
		if (soerr != 0) {
			sprintf(m->last_error, "TCP 连接被拒: %d", soerr);
			closesocket(m->sock);
			m->sock = INVALID_SOCKET;
			return false;
		}
	}
	/* 恢复阻塞模式 + 设收发超时 */
	nb = 0;
	ioctlsocket(m->sock, FIONBIO, &nb);
	DWORD tmo = 1000;   /* 1s 收发超时, 与 RTU COMMTIMEOUTS 对齐 */
	setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof(tmo));
	setsockopt(m->sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tmo, sizeof(tmo));

	m->transport = MB_TCP;
	m->connected = true;
	m->unit_id = unit_id;
	m->tcp_tid = 1;
	return true;
}

bool MbClient_ConnectRtu(MbClient *m, const wchar_t *com_port, uint32_t baud, uint8_t unit_id)
{
	if (!m) return false;
	if (m->connected) {
		MbClient_Disconnect(m);
	}
	m->hCom = CreateFileW(com_port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
	                      OPEN_EXISTING, 0, NULL);
	if (m->hCom == INVALID_HANDLE_VALUE) {
		sprintf(m->last_error, "打开串口失败: %lu", GetLastError());
		return false;
	}
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(m->hCom, &dcb)) {
		sprintf(m->last_error, "GetCommState 失败: %lu", GetLastError());
		CloseHandle(m->hCom);
		m->hCom = INVALID_HANDLE_VALUE;
		return false;
	}
	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	if (!SetCommState(m->hCom, &dcb)) {
		sprintf(m->last_error, "SetCommState 失败: %lu", GetLastError());
		CloseHandle(m->hCom);
		m->hCom = INVALID_HANDLE_VALUE;
		return false;
	}
	COMMTIMEOUTS to;
	to.ReadIntervalTimeout = 50;
	to.ReadTotalTimeoutConstant = 1000;
	to.ReadTotalTimeoutMultiplier = 0;
	to.WriteTotalTimeoutConstant = 1000;
	to.WriteTotalTimeoutMultiplier = 0;
	SetCommTimeouts(m->hCom, &to);

	/* 清空串口残留缓冲, 避免上次会话的脏数据干扰 */
	PurgeComm(m->hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);

	m->transport = MB_RTU;
	m->connected = true;
	m->unit_id = unit_id;
	m->baud = baud;
	return true;
}

void MbClient_Disconnect(MbClient *m)
{
	if (!m) return;
	if (m->transport == MB_TCP && m->sock != INVALID_SOCKET) {
		closesocket(m->sock);
		m->sock = INVALID_SOCKET;
	} else if (m->transport == MB_RTU && m->hCom != INVALID_HANDLE_VALUE) {
		CloseHandle(m->hCom);
		m->hCom = INVALID_HANDLE_VALUE;
	}
	m->connected = false;
}

/* ================================================================
 * FC 包装
 *
 * PDU 组装/解析规则:
 *   FC01/02 读: req [fc][addr_hi][addr_lo][qty_hi][qty_lo]; resp [fc][bc][bits LSB-first]
 *   FC03/04 读: req 同上; resp [fc][bc][reg_hi reg_lo]... 大端 16 位
 *   FC05 写单线圈: req [0x05][addr_hi][addr_lo][0xFF00 if on else 0x0000]; resp 回显
 *   FC06 写单寄存器: req [0x06][addr_hi][addr_lo][val_hi][val_lo]; resp 回显
 *   FC16 写多寄存器: req [0x10][addr_hi][addr_lo][qty_hi][qty_lo][bc=qty*2][reg...]; resp [0x10][addr_hi][addr_lo][qty_hi][qty_lo]
 * ================================================================ */

bool MbClient_ReadCoils(MbClient *m, uint16_t addr, uint16_t qty, uint8_t *out_bits)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	if (qty == 0) {
		sprintf(m->last_error, "qty 不能为 0");
		return false;
	}
	uint8_t pdu[5] = { 0x01, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(qty >> 8), (uint8_t)qty };
	uint8_t resp[256];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	if (!check_normal(m, resp, n)) return false;
	int bc = resp[1];
	if (n < 2 + bc) {
		sprintf(m->last_error, "响应过短");
		return false;
	}
	/* 位按 LSB-first 打包进字节: byte0 bit0 = coil addr, byte0 bit1 = addr+1, ... */
	for (uint16_t i = 0; i < qty; i++) {
		uint8_t byte = resp[2 + (i >> 3)];
		out_bits[i] = (uint8_t)((byte >> (i & 7)) & 1);
	}
	return true;
}

bool MbClient_ReadDiscreteInputs(MbClient *m, uint16_t addr, uint16_t qty, uint8_t *out_bits)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	if (qty == 0) {
		sprintf(m->last_error, "qty 不能为 0");
		return false;
	}
	uint8_t pdu[5] = { 0x02, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(qty >> 8), (uint8_t)qty };
	uint8_t resp[256];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	if (!check_normal(m, resp, n)) return false;
	int bc = resp[1];
	if (n < 2 + bc) {
		sprintf(m->last_error, "响应过短");
		return false;
	}
	for (uint16_t i = 0; i < qty; i++) {
		uint8_t byte = resp[2 + (i >> 3)];
		out_bits[i] = (uint8_t)((byte >> (i & 7)) & 1);
	}
	return true;
}

bool MbClient_ReadHolding(MbClient *m, uint16_t addr, uint16_t qty, uint16_t *out_regs)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	if (qty == 0) {
		sprintf(m->last_error, "qty 不能为 0");
		return false;
	}
	uint8_t pdu[5] = { 0x03, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(qty >> 8), (uint8_t)qty };
	uint8_t resp[256];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	if (!check_normal(m, resp, n)) return false;
	int bc = resp[1];
	if (n < 2 + bc) {
		sprintf(m->last_error, "响应过短");
		return false;
	}
	for (uint16_t i = 0; i < qty; i++) {
		out_regs[i] = (uint16_t)(((uint16_t)resp[2 + i * 2] << 8) | resp[3 + i * 2]);
	}
	return true;
}

bool MbClient_ReadInput(MbClient *m, uint16_t addr, uint16_t qty, uint16_t *out_regs)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	if (qty == 0) {
		sprintf(m->last_error, "qty 不能为 0");
		return false;
	}
	uint8_t pdu[5] = { 0x04, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(qty >> 8), (uint8_t)qty };
	uint8_t resp[256];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	if (!check_normal(m, resp, n)) return false;
	int bc = resp[1];
	if (n < 2 + bc) {
		sprintf(m->last_error, "响应过短");
		return false;
	}
	for (uint16_t i = 0; i < qty; i++) {
		out_regs[i] = (uint16_t)(((uint16_t)resp[2 + i * 2] << 8) | resp[3 + i * 2]);
	}
	return true;
}

bool MbClient_WriteSingleCoil(MbClient *m, uint16_t addr, bool on)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	/* FC05 PDU = [0x05][addr_hi][addr_lo][FF00/0000] = 5B (fc + 4B data) */
	uint8_t pdu[5] = { 0x05, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(on ? 0xFF : 0x00), 0x00 };
	uint8_t resp[16];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	return check_normal(m, resp, n);
}

bool MbClient_WriteSingleReg(MbClient *m, uint16_t addr, uint16_t value)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	/* FC06 PDU = [0x06][addr_hi][addr_lo][val_hi][val_lo] = 5B (fc + 4B data) */
	uint8_t pdu[5] = { 0x06, (uint8_t)(addr >> 8), (uint8_t)addr,
	                   (uint8_t)(value >> 8), (uint8_t)value };
	uint8_t resp[16];
	int n = mb_transact(m, pdu, 5, resp, sizeof(resp));
	return check_normal(m, resp, n);
}

bool MbClient_WriteMultiReg(MbClient *m, uint16_t addr, uint16_t qty, const uint16_t *values)
{
	if (!m || !m->connected) {
		if (m) sprintf(m->last_error, "Modbus 未连接");
		return false;
	}
	if (qty == 0 || !values) {
		sprintf(m->last_error, "qty/values 非法");
		return false;
	}
	/* PDU = [0x10][addr_hi][addr_lo][qty_hi][qty_lo][bc=qty*2][reg values 大端] */
	uint8_t pdu[256];
	int pdulen = 6 + qty * 2;
	if (pdulen > (int)sizeof(pdu)) {
		sprintf(m->last_error, "qty 过大");
		return false;
	}
	pdu[0] = 0x10;
	pdu[1] = (uint8_t)(addr >> 8);
	pdu[2] = (uint8_t)addr;
	pdu[3] = (uint8_t)(qty >> 8);
	pdu[4] = (uint8_t)qty;
	pdu[5] = (uint8_t)(qty * 2);
	for (uint16_t i = 0; i < qty; i++) {
		pdu[6 + i * 2] = (uint8_t)(values[i] >> 8);
		pdu[7 + i * 2] = (uint8_t)values[i];
	}
	uint8_t resp[16];
	int n = mb_transact(m, pdu, pdulen, resp, sizeof(resp));
	return check_normal(m, resp, n);
}
