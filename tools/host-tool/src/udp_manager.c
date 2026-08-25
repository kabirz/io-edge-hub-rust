/*
 * io-edge-hub 上位机 UDP 管理器 - 同步 req/resp 客户端 (Task 3)
 *
 * 单 socket bind 本地任意端口 (源端口由 OS 分配), 对目标 ip:8600 sendto +
 * recvfrom (SO_RCVTIMEO 超时). 覆盖固件 UDP 全部命令:
 *   - 配置 0x10-0x19 (大端多字节字段): SET/GET IP, SET/GET MODBUS, SET_TIME, FACTORY_RESET
 *   - 升级 0x01-0x03 (小端 size/offset/crc) + 0x04 版本 / 0x05 重启
 *   - 设备发现 (GET_IP 0x11, 广播允许): 子网定向广播 + 8601 跨网段回复监听
 *
 * 升级流式部分由 tab2 worker 线程顺序调用 FwStart/FwData/FwEnd; tab1 配置命令
 * 为单发单收, UI 阻塞即可. 不复用 handler-receiver 的 RX 线程模型.
 *
 * 协议权威来源:
 *   - applications/io-edge-hub/src/udp.c (0x10+ 应用命令)
 *   - libs/udp_fw_upgrade/udp_fw_upgrade.c (0x01-0x05 升级/版本/重启)
 *
 * collect_broadcast_addrs() 与 UdpManager_CRC16_CCITT() 自 handler-receiver
 * src/udp_manager.c 复制 (CRC 与 Zephyr crc16_ccitt 完全一致).
 */
#include "udp_manager.h"
#include <iphlpapi.h>
#include <stdio.h>
#include <string.h>

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")

struct UdpManager {
	SOCKET sock;                /* UDP socket, bind 0.0.0.0:0 */
	char last_error[128];
};

/* ================================================================
 * 子网定向广播地址枚举 (复制自 handler-receiver src/udp_manager.c)
 *
 * Windows 发 255.255.255.255 (有限广播) 时多网卡主机路由表无法决定从哪个
 * 接口发出 → 包被丢弃. 改用各网卡子网定向广播 (如 192.168.1.255), 遍历所有
 * 非回环网卡逐个发出, 确保板子无论连哪个网卡都能收到.
 * ================================================================ */

/* 收集本机所有非回环网卡的子网定向广播地址. 返回填充数量.
 * addrs[i] 为网络序 s_addr. (复制自 handler-receiver, 删去冗余注释) */
static int collect_broadcast_addrs(unsigned long *addrs, int max_cnt)
{
	ULONG bufLen = 15000;
	PIP_ADAPTER_ADDRESSES pAddrs = (PIP_ADAPTER_ADDRESSES)malloc(bufLen);
	int cnt = 0;

	if (pAddrs == NULL) {
		return 0;
	}

	ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
		      GAA_FLAG_SKIP_DNS_SERVER | GAA_FLAG_INCLUDE_PREFIX;

	if (GetAdaptersAddresses(AF_INET, flags, NULL, pAddrs, &bufLen) != NO_ERROR) {
		free(pAddrs);
		return 0;
	}

	PIP_ADAPTER_ADDRESSES p = pAddrs;
	while (p && cnt < max_cnt) {
		if (p->OperStatus != IfOperStatusUp) {
			p = p->Next;
			continue;
		}
		if (p->IfType == IF_TYPE_SOFTWARE_LOOPBACK ||
		    p->IfType == IF_TYPE_TUNNEL) {
			p = p->Next;
			continue;
		}

		PIP_ADAPTER_UNICAST_ADDRESS ua = p->FirstUnicastAddress;
		while (ua && cnt < max_cnt) {
			struct sockaddr_in *sa = (struct sockaddr_in *)ua->Address.lpSockaddr;
			unsigned long ip = sa->sin_addr.s_addr;
			ULONG plen;
			unsigned long mask, bcast;

			/* 跳过回环 (127.x), 未配置 (0.x), link-local (169.254.x) */
			if ((ip & htonl(0xFF000000)) == htonl(0x7F000000) ||
			    (ip & htonl(0xFFFF0000)) == htonl(0xA9FE0000) ||
			    ip == 0) {
				ua = ua->Next;
				continue;
			}

			/* OnLinkPrefixLength = IPv4 前缀长度 (Win Vista+).
			 * 定向广播 = (ip & mask) | ~mask */
			plen = ua->OnLinkPrefixLength;
			mask = (plen == 0) ? 0 : htonl(0xFFFFFFFF << (32 - plen));
			bcast = (ip & mask) | ~mask;

			addrs[cnt++] = bcast;
			ua = ua->Next;
		}
		p = p->Next;
	}

	free(pAddrs);
	return cnt;
}

/* ================================================================
 * 内部: 收发原语
 * ================================================================ */

/* 发 cmd 到 ip:8600, 阻塞等回复 (timeout_ms), 校验回复首字节 == cmd.
 * req: 含 cmd 字节的完整请求. resp/out_resp_len: 回复 (含 echo cmd).
 * timeout_ms: 接收超时 (IOEDGE_UDP_TIMEOUT_MS=常规 1s; FW_START=5s 擦 flash;
 *             FW_END=10s flush+读回重算 CRC). 返回 true=收到合法回复. */
static bool send_recv(UdpManager *m, const char *ip, uint8_t cmd,
                      const uint8_t *req, int req_len,
                      uint8_t *resp, int *out_resp_len, int timeout_ms)
{
	struct sockaddr_in dst = {0};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(IOEDGE_UDP_PORT);
	dst.sin_addr.s_addr = inet_addr(ip);
	if (dst.sin_addr.s_addr == INADDR_NONE) {
		sprintf(m->last_error, "非法 IP: %s", ip);
		return false;
	}

	if (sendto(m->sock, (const char *)req, req_len, 0,
	           (struct sockaddr *)&dst, sizeof(dst)) == SOCKET_ERROR) {
		sprintf(m->last_error, "sendto 失败: %d", WSAGetLastError());
		return false;
	}

	/* 设置接收超时 (按命令类型不同, 见注释) */
	DWORD tmo = timeout_ms;
	setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof(tmo));

	struct sockaddr_in from = {0};
	int fromlen = sizeof(from);
	int n = recvfrom(m->sock, (char *)resp, 128, 0,
	                 (struct sockaddr *)&from, &fromlen);
	if (n <= 0) {
		sprintf(m->last_error, "设备无响应 (timeout)");
		return false;
	}
	if (n < 1 || resp[0] != cmd) {
		sprintf(m->last_error, "回复格式错误");
		return false;
	}
	*out_resp_len = n;
	return true;
}

/* ================================================================
 * 生命周期
 * ================================================================ */

UdpManager *UdpManager_Create(void)
{
	UdpManager *m = (UdpManager *)calloc(1, sizeof(*m));
	if (!m) return NULL;
	m->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m->sock == INVALID_SOCKET) {
		free(m);
		return NULL;
	}
	/* bind 本地任意端口 (源端口由 OS 分配, 固件回复到源端口) */
	struct sockaddr_in local = {0};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = INADDR_ANY;
	bind(m->sock, (struct sockaddr *)&local, sizeof(local));
	/* 允许广播 (Discover 用) */
	BOOL bc = TRUE;
	setsockopt(m->sock, SOL_SOCKET, SO_BROADCAST, (const char *)&bc, sizeof(bc));
	return m;
}

void UdpManager_Destroy(UdpManager *m)
{
	if (!m) return;
	if (m->sock != INVALID_SOCKET) closesocket(m->sock);
	free(m);
}

const char *UdpManager_GetLastError(UdpManager *m)
{
	return m ? m->last_error : "NULL manager";
}

/* ================================================================
 * 升级命令组 (0x01-0x03, 小端 size/offset/crc)
 * ================================================================ */

bool UdpManager_FwStart(UdpManager *m, const char *ip, uint32_t img_size,
                        const uint8_t keyhash[32], uint8_t *out_status,
                        uint16_t *out_v2_chunk)
{
	uint8_t req[1 + 4 + 32];
	int reqlen = 1 + 4;
	req[0] = 0x01;
	req[1] = (uint8_t)(img_size);          /* LE32 */
	req[2] = (uint8_t)(img_size >> 8);
	req[3] = (uint8_t)(img_size >> 16);
	req[4] = (uint8_t)(img_size >> 24);
	if (keyhash) {
		memcpy(req + 5, keyhash, 32);
		reqlen += 32;
	}
	uint8_t resp[64];
	int rn = 0;
	/* FW_START: 固件按镜像大小擦 slot1 (4KB 向上取整, 常超 1s),
	 * 给 5s 超时 (与 handler-receiver 一致). */
	if (!send_recv(m, ip, 0x01, req, reqlen, resp, &rn, 5000)) return false;
	if (rn < 2) { sprintf(m->last_error, "FW_START 回复过短"); return false; }
	if (out_status) *out_status = resp[1];
	/* 新固件回复带 [v2_chunk 2B] (DATA_V2 协商); 老固件无 → 0 (停等模式) */
	if (out_v2_chunk) {
		*out_v2_chunk = (rn >= 4) ? (uint16_t)(resp[2] | (resp[3] << 8)) : 0;
	}
	return true;
}

bool UdpManager_FwData(UdpManager *m, const char *ip, const uint8_t *data, int len,
                       uint32_t *out_offset)
{
	if (len > 511) { sprintf(m->last_error, "FW_DATA 单块超 511B"); return false; }
	uint8_t req[1 + 511];
	req[0] = 0x02;
	memcpy(req + 1, data, len);
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x02, req, 1 + len, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (rn < 5) { sprintf(m->last_error, "FW_DATA 回复过短"); return false; }
	if (out_offset) *out_offset = (uint32_t)resp[1] | ((uint32_t)resp[2] << 8) |
	                              ((uint32_t)resp[3] << 16) | ((uint32_t)resp[4] << 24);
	return true;
}

bool UdpManager_FwEnd(UdpManager *m, const char *ip, uint8_t test, uint16_t crc16,
                      uint8_t *out_result)
{
	uint8_t req[4];
	req[0] = 0x03;
	req[1] = test;
	req[2] = (uint8_t)(crc16);       /* LE16 */
	req[3] = (uint8_t)(crc16 >> 8);
	uint8_t resp[64];
	int rn = 0;
	/* FW_END: 固件 flush 写入后按 64B 块读回整个已写区域重算 CRC (满 slot 易
	 * 1-3s), 给 10s 超时 (与 handler-receiver 一致). */
	if (!send_recv(m, ip, 0x03, req, 4, resp, &rn, 10000)) return false;
	if (rn < 2) { sprintf(m->last_error, "FW_END 回复过短"); return false; }
	if (out_result) *out_result = resp[1];
	return true;
}

/* ==================== FW_DATA_V2 (0x06) 窗口流水线 ==================== */

#define FW_V2_WINDOW      8     /* go-back-N 窗口帧数 */
#define FW_V2_ACK_TMO     1000  /* 窗口级 ACK 超时 ms (覆盖渐进擦除的扇区擦停顿 ~400ms) */
#define FW_V2_MAX_RETRY   8     /* 单窗口停滞重试上限 */
#define FW_V2_CHUNK_MAX   1400  /* 单帧数据上限 (以太网 MTU 内) */

bool UdpManager_FwDataV2Stream(UdpManager *m, const char *ip,
                               const uint8_t *data, uint32_t total, int chunk,
                               UdpProgressFn progress, void *user_data,
                               UdpCancelFn cancel)
{
	struct sockaddr_in dst = {0};
	uint8_t *frame;
	uint8_t resp[64];
	uint32_t off;
	int retries = 0;

	dst.sin_family = AF_INET;
	dst.sin_port = htons(IOEDGE_UDP_PORT);
	dst.sin_addr.s_addr = inet_addr(ip);
	if (dst.sin_addr.s_addr == INADDR_NONE) {
		sprintf(m->last_error, "非法 IP: %s", ip);
		return false;
	}
	if (chunk <= 0 || chunk > FW_V2_CHUNK_MAX) {
		sprintf(m->last_error, "FW_DATA_V2 chunk 非法: %d", chunk);
		return false;
	}
	frame = (uint8_t *)malloc(5 + chunk);
	if (!frame) {
		sprintf(m->last_error, "内存不足");
		return false;
	}

	off = 0;
	while (off < total) {
		uint32_t win_end, w, confirmed;
		DWORD deadline;

		if (cancel && cancel(user_data)) {
			sprintf(m->last_error, "用户取消升级");
			free(frame);
			return false;
		}
		win_end = off + FW_V2_WINDOW * (uint32_t)chunk;
		if (win_end > total || win_end < off) {
			win_end = total;
		}

		/* 发送一个窗口 [off, win_end): 连发不等回复 */
		for (w = off; w < win_end; w += (uint32_t)chunk) {
			uint32_t n = (win_end - w > (uint32_t)chunk)
			             ? (uint32_t)chunk : win_end - w;

			frame[0] = 0x06;
			frame[1] = (uint8_t)w;          /* LE32 */
			frame[2] = (uint8_t)(w >> 8);
			frame[3] = (uint8_t)(w >> 16);
			frame[4] = (uint8_t)(w >> 24);
			memcpy(frame + 5, data + w, n);
			if (sendto(m->sock, (const char *)frame, 5 + n, 0,
			           (struct sockaddr *)&dst, sizeof(dst)) == SOCKET_ERROR) {
				sprintf(m->last_error, "sendto 失败: %d", WSAGetLastError());
				free(frame);
				return false;
			}
		}

		/* 收窗口内 ACK (回复始终为设备期望 offset), 追踪最大确认 */
		deadline = GetTickCount() + FW_V2_ACK_TMO;
		confirmed = off;
		while (confirmed < win_end) {
			DWORD now = GetTickCount();
			DWORD tmo;
			struct sockaddr_in from = {0};
			int fromlen = sizeof(from);
			int n;

			if (now >= deadline) {
				break;
			}
			tmo = deadline - now;
			setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO,
			           (const char *)&tmo, sizeof(tmo));
			n = recvfrom(m->sock, (char *)resp, sizeof(resp), 0,
			             (struct sockaddr *)&from, &fromlen);
			if (n <= 0) {
				break;  /* 超时: 走重传 */
			}
			if (n >= 5 && resp[0] == 0x06) {
				uint32_t roff = (uint32_t)resp[1] | ((uint32_t)resp[2] << 8) |
				                ((uint32_t)resp[3] << 16) |
				                ((uint32_t)resp[4] << 24);

				if (roff > confirmed) {
					confirmed = (roff > total) ? total : roff;
					retries = 0;  /* 有推进即重置停滞计数 */
				}
			}
		}
		if (progress) {
			progress(confirmed, user_data);
		}

		if (confirmed >= win_end) {
			off = confirmed;
			continue;
		}
		/* 窗口未完全确认: 从确认处 go-back-N 重传 (重复帧设备自动丢弃) */
		retries++;
		if (retries > FW_V2_MAX_RETRY) {
			sprintf(m->last_error,
			        "窗口重试超限 (offset=%u, 设备停滞或链路中断)",
			        confirmed);
			free(frame);
			return false;
		}
		off = confirmed;
	}
	free(frame);
	return true;
}

/* ================================================================
 * 配置命令组 (0x10+, 大端多字节字段)
 * ================================================================ */

bool UdpManager_SetIp(UdpManager *m, const char *ip, uint8_t ip4[4], uint8_t *out_ok)
{
	uint8_t req[5];
	req[0] = 0x10;
	memcpy(req + 1, ip4, 4);
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x10, req, 5, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (out_ok) *out_ok = resp[1];
	return true;
}

bool UdpManager_GetIp(UdpManager *m, const char *ip, uint8_t ip4[4])
{
	uint8_t req[1] = { 0x11 };
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x11, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	/* 回 [0x11][ip 4B] */
	if (rn < 5) { sprintf(m->last_error, "GET_IP 回复过短"); return false; }
	if (ip4) memcpy(ip4, resp + 1, 4);
	return true;
}

bool UdpManager_SetModbus(UdpManager *m, const char *ip, uint8_t slave_id,
                          uint16_t baud, uint8_t *out_ok)
{
	uint8_t req[4];
	req[0] = 0x12;
	req[1] = slave_id;
	req[2] = (uint8_t)(baud >> 8);    /* BE16 */
	req[3] = (uint8_t)(baud);
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x12, req, 4, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (out_ok) *out_ok = resp[1];
	return true;
}

bool UdpManager_GetModbus(UdpManager *m, const char *ip, uint8_t *out_slave,
                          uint16_t *out_baud)
{
	uint8_t req[1] = { 0x13 };
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x13, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (rn < 4) { sprintf(m->last_error, "GET_MODBUS 回复过短"); return false; }
	if (out_slave) *out_slave = resp[1];
	if (out_baud)  *out_baud  = ((uint16_t)resp[2] << 8) | resp[3]; /* BE16 */
	return true;
}

/* SET_TIME (0x14): 设 [0x14][unix_ts BE32], 回 [0x14][ok 1B]. */
bool UdpManager_SetTime(UdpManager *m, const char *ip, uint32_t unix_ts, uint8_t *out_ok)
{
	uint8_t req[5];
	req[0] = 0x14;
	req[1] = (uint8_t)(unix_ts >> 24);   /* BE32 */
	req[2] = (uint8_t)(unix_ts >> 16);
	req[3] = (uint8_t)(unix_ts >> 8);
	req[4] = (uint8_t)(unix_ts);
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x14, req, 5, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (out_ok) *out_ok = resp[1];
	return true;
}

/* GET_KEYHASH (0x15): 发 [0x15], 回 [0x15][keyhash 32B]。
 * 设备自报它 START 校验用的公钥指纹 —— 与升级同一通道, 换钥匙零同步。 */
bool UdpManager_GetKeyhash(UdpManager *m, const char *ip, uint8_t out_keyhash[32])
{
	uint8_t req[1] = { 0x15 };
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x15, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (rn < 33) { sprintf(m->last_error, "GET_KEYHASH 回复过短"); return false; }
	if (out_keyhash) memcpy(out_keyhash, resp + 1, 32);
	return true;
}

bool UdpManager_FactoryReset(UdpManager *m, const char *ip, uint8_t *out_ok)
{
	uint8_t req[1] = { 0x19 };
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x19, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	if (out_ok) *out_ok = resp[1];
	return true;
}

/* GET_VERSION (0x04): 回 [0x04][ASCII 版本串, 无 NUL] */
bool UdpManager_GetVersion(UdpManager *m, const char *ip, char *out_ver, int out_cap)
{
	if (!out_ver || out_cap <= 0) {
		sprintf(m->last_error, "invalid args");
		return false;
	}
	uint8_t req[1] = { 0x04 };
	uint8_t resp[64];
	int rn = 0;
	if (!send_recv(m, ip, 0x04, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS)) return false;
	int vlen = rn - 1;
	if (vlen <= 0) { sprintf(m->last_error, "GET_VERSION 空回复"); return false; }
	if (vlen >= out_cap) vlen = out_cap - 1;
	memcpy(out_ver, resp + 1, vlen);
	out_ver[vlen] = 0;
	return true;
}

/* REBOOT (0x05): 设备收到即重启, 回复不可靠 → 不强求回复. */
bool UdpManager_Reboot(UdpManager *m, const char *ip)
{
	uint8_t req[1] = { 0x05 };
	uint8_t resp[64];
	int rn = 0;
	send_recv(m, ip, 0x05, req, 1, resp, &rn, IOEDGE_UDP_TIMEOUT_MS);
	return true;  /* reboot 不强求回复 */
}

/* ================================================================
 * 设备发现 (GET_IP 0x11, broadcast-allowed)
 *
 * 向所有本机非回环网卡的子网定向广播地址发 0x11 到 8600. 固件回复路由:
 *   - 同子网: 单播回到发送方源端口 (主 socket 收得到)
 *   - 跨子网: 发往有限广播 (INADDR_BROADCAST) 的 8601 → 需单独 bind 8601 监听
 * 回复 payload 为 4B IP (大端). 上位机格式化为点分十进制 "a.b.c.d".
 *
 * 监听窗口内轮询主 socket 与 8601 socket, 去重 (按 IP 字符串), 每条一行填入 out.
 * ================================================================ */

/* 把一条设备发现回复追加到 out (按行). 返回是否新增 (用于计数). */
static int discover_append_line(char *out, int out_cap, int *out_len,
                                const char *line)
{
	/* 去重: out 中已存在相同行则不重复追加 */
	if (strstr(out, line) != NULL) {
		return 0;
	}
	int room = out_cap - *out_len - 1;   /* 留 1 字节给 NUL */
	int need = (int)strlen(line) + 1;    /* 行内容 + '\n' */
	if (need > room) {
		need = room;
	}
	if (need <= 0) {
		return 0;
	}
	/* 行内容 (截断到 room) + '\n' */
	int line_len = (int)strlen(line);
	if (line_len > need - 1) line_len = need - 1;
	memcpy(out + *out_len, line, line_len);
	*out_len += line_len;
	out[*out_len] = '\n';
	(*out_len)++;
	out[*out_len] = 0;
	return 1;
}

/* 处理一个发现回复包: 校验首字节 == 0x11, 提取 4B IP, 格式化为 "a.b.c.d" 追加到 out. */
static int discover_handle_reply(const uint8_t *buf, int n,
                                 char *out, int out_cap, int *out_len)
{
	if (n < 5 || buf[0] != 0x11) {
		return 0;
	}
	char line[24];
	snprintf(line, sizeof(line), "%u.%u.%u.%u", buf[1], buf[2], buf[3], buf[4]);
	return discover_append_line(out, out_cap, out_len, line);
}

bool UdpManager_Discover(UdpManager *m, char *out, int out_cap, int *out_count)
{
	if (out_cap <= 0) {
		if (out_count) *out_count = 0;
		sprintf(m->last_error, "out 缓冲过小");
		return false;
	}
	out[0] = 0;

	unsigned long bcasts[16];
	int nb = collect_broadcast_addrs(bcasts, 16);
	uint8_t req[1] = { 0x11 };   /* GET_IP, 固件注册为广播允许命令 */

	/* 发定向广播到所有网卡:8600 */
	struct sockaddr_in dst = {0};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(IOEDGE_UDP_PORT);
	for (int i = 0; i < nb; i++) {
		dst.sin_addr.s_addr = bcasts[i];
		sendto(m->sock, (const char *)req, 1, 0,
		       (struct sockaddr *)&dst, sizeof(dst));
	}

	/* 8601 监听 socket: 收跨网段广播回复 (固件 INADDR_BROADCAST:8601) */
	DWORD tmo = 800;
	setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof(tmo));
	SOCKET s86 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in a86 = {0};
	a86.sin_family = AF_INET;
	a86.sin_port = htons(IOEDGE_UDP_REPLY_PORT);
	a86.sin_addr.s_addr = INADDR_ANY;
	/* 8601 可能被同机其他实例占用, 允许复用以避免 bind 失败 */
	BOOL reuse = TRUE;
	setsockopt(s86, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));
	bind(s86, (struct sockaddr *)&a86, sizeof(a86));
	setsockopt(s86, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof(tmo));

	int cnt = 0;
	int out_len = 0;
	uint8_t buf[128];

	/* 监听窗口 ~1s: 主 socket (同子网单播) + 8601 socket (跨子网广播) 轮询.
	 * 用短超时 recvfrom 多轮, 总时长用 GetTickCount 控制 (比 time() 精度高). */
	DWORD end = GetTickCount() + 1000;
	for (;;) {
		DWORD now = GetTickCount();
		if (now >= end) break;
		DWORD left = end - now;
		if (left < 1) left = 1;

		/* 主 socket: 缩短本轮超时到剩余时间的一部分, 提高响应灵敏度 */
		DWORD t1 = (left > 100) ? 100 : left;
		setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&t1, sizeof(t1));
		struct sockaddr_in from;
		int fl = sizeof(from);
		int n = recvfrom(m->sock, (char *)buf, sizeof(buf), 0,
		                 (struct sockaddr *)&from, &fl);
		if (n > 0) {
			cnt += discover_handle_reply(buf, n, out, out_cap, &out_len);
		}

		/* 8601 socket */
		DWORD t2 = (left > 100) ? 100 : left;
		setsockopt(s86, SOL_SOCKET, SO_RCVTIMEO, (const char *)&t2, sizeof(t2));
		fl = sizeof(from);
		n = recvfrom(s86, (char *)buf, sizeof(buf), 0,
		             (struct sockaddr *)&from, &fl);
		if (n > 0) {
			cnt += discover_handle_reply(buf, n, out, out_cap, &out_len);
		}
	}

	closesocket(s86);

	/* 恢复主 socket 默认超时, 避免影响后续命令 */
	DWORD def_tmo = IOEDGE_UDP_TIMEOUT_MS;
	setsockopt(m->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&def_tmo, sizeof(def_tmo));

	if (out_count) *out_count = cnt;
	if (cnt == 0) {
		sprintf(m->last_error, "未发现设备");
		return false;
	}
	return true;
}

/* ================================================================
 * CRC16-CCITT (复制自 handler-receiver src/udp_manager.c)
 *
 * 与 Zephyr crc16_ccitt (subsys/crc/crc16_sw.c) 完全一致: poly 0x1021,
 * init 0x0000, bit-reflected 变体 (非标准 MSB-first CCITT). UDP 升级 FW_END
 * 用此 CRC 校验 slot1 全量.
 * ================================================================ */
uint16_t UdpManager_CRC16_CCITT(const uint8_t *data, size_t len)
{
	uint16_t seed = 0x0000;

	for (; len > 0; len--) {
		uint8_t e, f;

		e = (uint8_t)seed ^ *data;
		++data;
		f = (uint8_t)(e ^ (e << 4));
		seed = (uint16_t)((seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^
				  ((uint16_t)f >> 4));
	}
	return seed;
}
