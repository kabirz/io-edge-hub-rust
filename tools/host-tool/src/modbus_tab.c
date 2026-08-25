/* io-edge-hub 上位机 - Tab3 "Modbus 调试"
 *
 * 程序化创建全部控件, 单一 g_mb 静态结构持有所有控件 HWND + MbClient 实例 + 寄存器
 * 元数据缓存. WM_CREATE 创建控件 + MbClient_Create; WM_DESTROY 断开 + 销毁 + KillTimer.
 *
 * 通道: 单选 TCP / RTU. 切换时 ShowWindow 显示对应子区域 (IP/端口 vs COM/波特率).
 * 连接: TCP 调 MbClient_ConnectTcp; RTU 调 MbClient_ConnectRtu. 单一连接.
 *
 * 四块面板 (groupbox 分隔):
 *   - DI 面板 (16 LED, 只读): ReadDiscreteInputs(0,16) → 16 个 STATIC 控件,
 *     WM_CTLCOLORSTATIC 按 bit 着色 (绿=ON, 灰=OFF).
 *   - DO 面板 (8 按钮, 可写): 8 个 BUTTON. 点击 → WriteSingleCoil(addr,!state) →
 *     立即 ReadCoils(0,8) 回读并更新按钮文字 (DOx ON/OFF).
 *   - AI 面板 (4 文本, 只读): ReadInput(1,4) → AI1/AI2 显示 mA, AI3/AI4 显示 V.
 *   - 寄存器表 ListView: 17 holding (时间戳两字合并) + 6 input, 双击写, "查询选中" 读单行.
 *
 * 自动刷新: 勾选 → SetTimer(hSelf, IDC_MB_TIMER, interval, NULL);
 * WM_TIMER 刷新 DI/DO/AI 面板 + 全部保持/输入寄存器表 (同一 UI 线程顺序执行).
 */
#include "modbus_client.h"   /* 须先于 windows.h 拉 winsock2.h (避免 winsock1 冲突) */
#include "modbus_tab.h"
#include "resource.h"
#include <commctrl.h>
#include <setupapi.h>
#include <devguid.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <wchar.h>

#pragma comment(lib, "setupapi.lib")

/* ===== 寄存器元数据表 (spec §7) ===== */

enum { RW_RW, RW_RO, RW_WO, RW_WO_TRIG };

typedef struct {
	uint16_t addr;       /* 0-based offset */
	const wchar_t *name;
	bool is_input;       /* true=input (FC04), false=holding (FC03/06) */
	int  rw;             /* RW_RW / RW_RO / RW_WO / RW_WO_TRIG */
} RegMeta;

static const RegMeta g_regs[] = {
	/* holding (FC03 读 / FC06 写, offset 0x00-0x11) */
	{0x00, L"DO输出控制",  false, RW_RW},
	{0x01, L"DI使能位图",  false, RW_RW},
	{0x02, L"AI使能位图",  false, RW_RW},
	{0x03, L"DI采样间隔ms", false, RW_RW},
	{0x04, L"AI采样间隔ms", false, RW_RW},
	{0x05, L"历史保存开关", false, RW_RW},
	{0x06, L"CAN业务帧ID", false, RW_RW},
	{0x07, L"CAN波特率(kbps)", false, RW_RW},
	{0x08, L"RS485波特率(bps)", false, RW_RW},
	{0x09, L"Modbus从机ID", false, RW_RW},
	{0x0A, L"IP第1字节",  false, RW_RW},
	{0x0B, L"IP第2字节",  false, RW_RW},
	{0x0C, L"IP第3字节",  false, RW_RW},
	{0x0D, L"IP第4字节",  false, RW_RW},
	{0x0E, L"时间戳",     false, RW_RW},
	{0x10, L"参数保存触发", false, RW_WO},
	{0x11, L"重启触发",   false, RW_WO},
	/* input (FC04 读, offset 0x00-0x05) */
	{0x00, L"固件版本",   true,  RW_RO},
	{0x01, L"AI1电流",   true,  RW_RO},
	{0x02, L"AI2电流",   true,  RW_RO},
	{0x03, L"AI3电压",   true,  RW_RO},
	{0x04, L"AI4电压",   true,  RW_RO},
	{0x05, L"DI位图",    true,  RW_RO},
};
#define REG_COUNT (int)(sizeof(g_regs) / sizeof(g_regs[0]))
#define HOLDING_COUNT 17      /* g_regs 前 17 项为 holding (时间戳两字已合并) */
#define HOLDING_PHYS_COUNT 18 /* 物理 holding 寄存器数 (0x00-0x11), 批量读用 */
#define DI_COUNT 16
#define DO_COUNT 8
#define AI_COUNT 4

/* ===== 静态状态: 所有控件 HWND + MbClient + 自动刷新 ===== */
typedef struct {
	HWND hSelf;
	/* 通道单选 */
	HWND hChanTcp, hChanRtu;
	/* TCP 行 */
	HWND hTcpLbl1, hIp, hTcpLbl2, hPort;
	/* RTU 行 */
	HWND hRtuLbl1, hCom, hRtuLbl2, hBaud, hRtuLbl3, hUidRtu;
	/* 连接 + 状态 */
	HWND hConn, hStatus;
	/* 刷新 + 自动刷新 */
	HWND hRefreshAll, hAutoRef, hAutoRefInt, hAutoRefLbl;
	/* 面板 groupbox */
	HWND hGbDi, hGbDo, hGbAi, hGbReg;
	/* DI 面板: 16 LED + 16 标签 */
	HWND hDi[DI_COUNT];
	/* DO 面板: 8 按钮 */
	HWND hDo[DO_COUNT];
	/* AI 面板: 4 文本 */
	HWND hAi[AI_COUNT];
	/* 寄存器表 */
	HWND hRegList, hRegQuery, hRegHint;
	/* 日志 */
	HWND hLog;
	/* manager */
	MbClient *mb;
	bool connected;
	bool auto_timer;     /* SetTimer 是否激活 */
	/* DI 当前位图 (用于 LED 着色, WM_CTLCOLORSTATIC 读) */
	uint8_t di_bits[DI_COUNT];
	bool di_valid;       /* di_bits 是否有效 */
} ModbusTab;

static ModbusTab g_mb;
static HFONT g_hFont = NULL;
static const wchar_t *MODBUS_TAB_CLASS = L"ioEdgeHubModbusTabCls";
static BOOL g_classRegistered = FALSE;

/* 串口与波特率选项 */
static const int g_bauds[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
#define BAUD_COUNT (int)(sizeof(g_bauds) / sizeof(g_bauds[0]))

/* 枚举系统中实际存在的 COM 口填入下拉框 (SetupAPI, 而非硬编码 COM1..COM32).
 * 按 COM 后的数字升序排列 (COM1 COM2 ... COM10, 字符串序会错把 COM10 排在 COM2 前).
 * 无串口时下拉为空 (连接时会提示先选择串口). */

/* qsort 比较器: 解析 "COM<n>" 的 n; 解析失败的串排在最后 (按字符串序). */
static int com_port_cmp(const void *a, const void *b)
{
	const wchar_t *pa = (const wchar_t *)a;
	const wchar_t *pb = (const wchar_t *)b;
	int na = -1, nb = -1;
	swscanf(pa, L"COM%d", &na);
	swscanf(pb, L"COM%d", &nb);
	if (na >= 0 && nb >= 0) {
		if (na != nb) return na < nb ? -1 : 1;
		return wcscmp(pa, pb);
	}
	if (na >= 0) return -1;
	if (nb >= 0) return 1;
	return wcscmp(pa, pb);
}

static void enumerate_com_ports(HWND hCombo)
{
	/* 上限 64 个: 物理串口数远达不到, 超出忽略 */
	wchar_t names[64][32];
	int count = 0;

	HDEVINFO hdevinfo = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, NULL, NULL,
	                                         DIGCF_PRESENT);
	if (hdevinfo == INVALID_HANDLE_VALUE) return;

	SP_DEVINFO_DATA devdata;
	memset(&devdata, 0, sizeof(devdata));
	devdata.cbSize = sizeof(devdata);

	for (DWORD idx = 0; SetupDiEnumDeviceInfo(hdevinfo, idx, &devdata); idx++) {
		if (count >= 64) break;
		HKEY hkey = SetupDiOpenDevRegKey(hdevinfo, &devdata, DICS_FLAG_GLOBAL, 0,
		                                 DIREG_DEV, KEY_READ);
		if (hkey == INVALID_HANDLE_VALUE) continue;
		wchar_t portname[64] = {0};
		DWORD type = 0, size = sizeof(portname);
		if (RegQueryValueExW(hkey, L"PortName", NULL, &type,
		                     (LPBYTE)portname, &size) == ERROR_SUCCESS &&
		    type == REG_SZ && portname[0] != L'\0') {
			size_t plen = wcslen(portname);
			if (plen > 31) plen = 31;
			memcpy(names[count], portname, plen * sizeof(wchar_t));
			names[count][plen] = L'\0';
			count++;
		}
		RegCloseKey(hkey);
	}
	SetupDiDestroyDeviceInfoList(hdevinfo);

	qsort(names, (size_t)count, sizeof(names[0]), com_port_cmp);

	SendMessageW(hCombo, CB_RESETCONTENT, 0, 0);
	for (int i = 0; i < count; i++)
		SendMessageW(hCombo, CB_ADDSTRING, 0, (LPARAM)names[i]);

	/* 默认选中第一个实际存在的 COM 口 */
	if (SendMessageW(hCombo, CB_GETCOUNT, 0, 0) > 0) {
		SendMessageW(hCombo, CB_SETCURSEL, 0, 0);
	}
}

/* ===== 控件创建辅助 (与 config_tab.c / upgrade_tab.c 一致) ===== */

static HWND create_label(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"STATIC", text,
		WS_CHILD | WS_VISIBLE, x, y, w, h,
		g_mb.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_edit(int x, int y, int w, int h, int id, DWORD extra)
{
	HWND hw = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | extra,
		x, y, w, h, g_mb.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_button(const wchar_t *text, int x, int y, int w, int h, int id)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		x, y, w, h, g_mb.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_groupbox(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
		x, y, w, h, g_mb.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

/* 创建 4 段 IP 编辑框 (ES_NUMBER + 限 3 字符). 返回末尾 x 坐标. */
static int create_ip_row(int x, int y, int ids[4], HWND out_hwnd[4])
{
	int seg_w = 34, dot_w = 6, gap = 2;
	for (int i = 0; i < 4; i++) {
		out_hwnd[i] = create_edit(x, y, seg_w, 22, ids[i], ES_NUMBER);
		SendMessageW(out_hwnd[i], EM_SETLIMITTEXT, 3, 0);
		x += seg_w;
		if (i < 3) {
			create_label(L".", x, y + 3, dot_w, 16);
			x += dot_w + gap;
		}
	}
	return x;
}

/* ===== 业务辅助 ===== */

/* 日志框追加一行 (带 [HH:MM:SS] 时间戳). */
static void log_append(const wchar_t *msg)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	wchar_t line[600];
	swprintf(line, 600, L"[%02d:%02d:%02d] %ls\r\n",
	         st.wHour, st.wMinute, st.wSecond, msg);
	int len = GetWindowTextLengthW(g_mb.hLog);
	SendMessageW(g_mb.hLog, EM_SETSEL, len, len);
	SendMessageW(g_mb.hLog, EM_REPLACESEL, 0, (LPARAM)line);
}

/* 传输层检测到连接断开时更新 UI 状态 (定义在下方, 先声明). */
static void on_link_lost(void);

/* 显示 Modbus 传输错误 (弹框 + 日志).
 * MbClient_GetLastError 返回 UTF-8 char* (MSVC /utf-8 编译), 必须用
 * MultiByteToWideChar(CP_UTF8) 转宽串, 不能用 swprintf 的 %hs (按 CP_ACP 解).
 * 若传输层已检测到连接断开: 只更新连接状态并记日志, 不弹框 (避免连环弹窗). */
static void show_mb_error(const wchar_t *op)
{
	const char *e = MbClient_GetLastError(g_mb.mb);
	wchar_t werr[192];
	MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
	wchar_t m[256];
	swprintf(m, 256, L"%ls 失败: %ls", op, werr);
	log_append(m);
	if (g_mb.connected && !MbClient_IsConnected(g_mb.mb)) {
		on_link_lost();
		return;
	}
	MessageBoxW(g_mb.hSelf, m, L"错误", MB_ICONERROR);
}

/* 当前选中通道: 0=TCP, 1=RTU */
static int current_channel(void)
{
	return (SendMessageW(g_mb.hChanRtu, BM_GETCHECK, 0, 0) == BST_CHECKED) ? 1 : 0;
}

/* 从单个 IP 输入框读 "a.b.c.d", 写入 ip4[4]. 返回是否合法 (4 段齐全, 每段 0-255). */
static bool read_ip_edit(HWND hEdit, uint8_t ip4[4])
{
	wchar_t buf[32];
	GetWindowTextW(hEdit, buf, 32);
	int v[4];
	if (swscanf(buf, L"%d.%d.%d.%d", &v[0], &v[1], &v[2], &v[3]) != 4) return false;
	for (int i = 0; i < 4; i++) {
		if (v[i] < 0 || v[i] > 255) return false;
		ip4[i] = (uint8_t)v[i];
	}
	return true;
}

/* 切换通道: 显示/隐藏 TCP 与 RTU 子区域. */
static void apply_channel_visibility(void)
{
	int rtu = current_channel();
	/* 每次切到 RTU 重新枚举 COM 口 (软件启动后才插入的串口也能识别).
	 * 原选中串口仍存在则保持选中, 否则回落到默认 (第一个). */
	if (rtu) {
		wchar_t prev[32] = {0};
		int sel = (int)SendMessageW(g_mb.hCom, CB_GETCURSEL, 0, 0);
		if (sel >= 0)
			SendMessageW(g_mb.hCom, CB_GETLBTEXT, sel, (LPARAM)prev);
		enumerate_com_ports(g_mb.hCom);
		if (prev[0]) {
			int match = (int)SendMessageW(g_mb.hCom, CB_FINDSTRINGEXACT,
			                               (WPARAM)-1, (LPARAM)prev);
			if (match >= 0)
				SendMessageW(g_mb.hCom, CB_SETCURSEL, match, 0);
		}
	}
	/* TCP 行 */
	ShowWindow(g_mb.hTcpLbl1, rtu ? SW_HIDE : SW_SHOW);
	ShowWindow(g_mb.hIp, rtu ? SW_HIDE : SW_SHOW);
	ShowWindow(g_mb.hTcpLbl2, rtu ? SW_HIDE : SW_SHOW);
	ShowWindow(g_mb.hPort,    rtu ? SW_HIDE : SW_SHOW);
	/* RTU 行 */
	ShowWindow(g_mb.hRtuLbl1, rtu ? SW_SHOW : SW_HIDE);
	ShowWindow(g_mb.hCom,     rtu ? SW_SHOW : SW_HIDE);
	ShowWindow(g_mb.hRtuLbl2, rtu ? SW_SHOW : SW_HIDE);
	ShowWindow(g_mb.hBaud,    rtu ? SW_SHOW : SW_HIDE);
	ShowWindow(g_mb.hRtuLbl3, rtu ? SW_SHOW : SW_HIDE);
	ShowWindow(g_mb.hUidRtu,  rtu ? SW_SHOW : SW_HIDE);
}

/* 设置连接状态: 状态灯文字 + 按钮启用. is_conn=true 已连接. */
static void set_conn_state(bool is_conn)
{
	g_mb.connected = is_conn;
	if (is_conn) {
		SetWindowTextW(g_mb.hStatus, L"● 已连接");
		SetWindowTextW(g_mb.hConn, L"断开");
		EnableWindow(g_mb.hConn, TRUE);
		EnableWindow(g_mb.hRefreshAll, TRUE);
	} else {
		SetWindowTextW(g_mb.hStatus, L"○ 未连接");
		SetWindowTextW(g_mb.hConn, L"连接");
		EnableWindow(g_mb.hConn, TRUE);
		EnableWindow(g_mb.hRefreshAll, FALSE);
		/* 断开即停止自动刷新 (手动断开或对端断开都一样), 并复位勾选 */
		if (g_mb.auto_timer) {
			KillTimer(g_mb.hSelf, IDC_MB_TIMER);
			g_mb.auto_timer = false;
		}
		SendMessageW(g_mb.hAutoRef, BM_SETCHECK, BST_UNCHECKED, 0);
		/* 连接断开后 LED/DO/AI 文本失效 */
		g_mb.di_valid = false;
		InvalidateRect(g_mb.hSelf, NULL, FALSE);
	}
}

/* 传输层检测到连接断开 (TCP 对端关闭等): 更新 UI 连接状态, 不再弹窗. */
static void on_link_lost(void)
{
	set_conn_state(false);
	log_append(L"连接已断开, 自动刷新已停止");
}

/* R/W 列显示文字 */
static const wchar_t *rw_label(int rw)
{
	switch (rw) {
	case RW_RW:       return L"RW";
	case RW_RO:       return L"RO";
	case RW_WO:       return L"WO";
	case RW_WO_TRIG:  return L"WO(触发)";
	default:          return L"";
	}
}

/* ===== 寄存器表 ListView 辅助 ===== */

/* 把 holding/input 寄存器值格式化为显示串. 每格先显示原始寄存器值, 换算/
 * 解释结果放括号里 (所有行都能看到实际寄存器值):
 * - 原始值: holding/input 16 位值; 时间戳 0x0E 为合并的 32 位 Unix 秒
 * - 位图类 (DO控制/DI使能/AI使能/DI位图): 十六进制 + (逐位二进制)
 * - AI 电流/电压 (input 0x01-0x04, 0.01 精度): 原始值 + (X.XX mA/V)
 * - 固件版本 (input 0x00): 十六进制 + (vX.Y.Z)
 * - 时间戳 (holding 0x0E): 十六进制 + (YYYY-MM-DD HH:MM:SS)
 * - CAN 帧 ID: 十六进制
 * - 采样间隔/波特率: 原始值 + (ms/kbps/bps)
 * - 其余 (从机ID/IP字节/开关/触发): 十进制原始值 */
static void format_reg_value(int row_idx, uint32_t value, wchar_t *out, int cap)
{
	const RegMeta *r = &g_regs[row_idx];

	/* 时间戳 (holding 0x0E): 32 位 Unix 秒 → 原始值 + 本机时区日期时间 */
	if (!r->is_input && r->addr == 0x0E) {
		time_t t = (time_t)value;
		struct tm tmv;
		if (localtime_s(&tmv, &t) == 0) {
			wchar_t ts[24];
			wcsftime(ts, 24, L"%Y-%m-%d %H:%M:%S", &tmv);
			swprintf(out, cap, L"0x%08X (%ls)", (unsigned)value, ts);
		} else {
			swprintf(out, cap, L"0x%08X", (unsigned)value);
		}
		return;
	}

	uint16_t v = (uint16_t)value;

	if (r->is_input) {
		switch (r->addr) {
		case 0x01: swprintf(out, cap, L"%u (%.2f mA)", v, v / 100.0); return;
		case 0x02: swprintf(out, cap, L"%u (%.2f mA)", v, v / 100.0); return;
		case 0x03: swprintf(out, cap, L"%u (%.2f V)",  v, v / 100.0); return;
		case 0x04: swprintf(out, cap, L"%u (%.2f V)",  v, v / 100.0); return;
		case 0x05: { /* DI1-DI16 位图 */
			wchar_t bin[17];
			for (int i = 0; i < 16; i++) {
				bin[i] = (v & (1u << (15 - i))) ? L'1' : L'0';
			}
			bin[16] = 0;
			swprintf(out, cap, L"0x%04X (%ls)", v, bin);
			return;
		}
		case 0x00: { /* 固件版本: MAJOR<<12 | MINOR<<8 | PATCH */
			int major = (v >> 12) & 0xF;
			int minor = (v >> 8) & 0xF;
			int patch = v & 0xFF;
			swprintf(out, cap, L"0x%04X (v%d.%d.%d)", v, major, minor, patch);
			return;
		}
		default:   swprintf(out, cap, L"%u", v); return;
		}
	}

	/* holding */
	switch (r->addr) {
	case 0x00: /* DO1-DO8 位图 */
	case 0x01: /* DI 使能位图 */
	case 0x02: { /* AI 使能位图 */
		wchar_t bin[17];
		for (int i = 0; i < 16; i++) {
			bin[i] = (v & (1u << (15 - i))) ? L'1' : L'0';
		}
		bin[16] = 0;
		swprintf(out, cap, L"0x%04X (%ls)", v, bin);
		return;
	}
	case 0x03: swprintf(out, cap, L"%u (ms)", v); return;  /* DI 采样间隔 */
	case 0x04: swprintf(out, cap, L"%u (ms)", v); return;  /* AI 采样间隔 */
	case 0x06: swprintf(out, cap, L"0x%04X", v); return; /* CAN 业务帧 ID */
	case 0x07: swprintf(out, cap, L"%u (kbps)", v); return; /* CAN 波特率 */
	case 0x08: swprintf(out, cap, L"%u (bps)", v); return; /* RS485 波特率 */
	default:   swprintf(out, cap, L"%u", v); return;     /* 开关/从机ID/IP字节/触发 */
	}
}

/* 更新 ListView 某行的"当前值"列. */
static void update_listview_row(int row_idx, uint32_t value)
{
	wchar_t vs[64];
	format_reg_value(row_idx, value, vs, 64);
	ListView_SetItemText(g_mb.hRegList, row_idx, 2, vs);
}

/* ===== 面板刷新 ===== */

/* 刷新 DI 面板: ReadDiscreteInputs(0,16) → 缓存位图 + 触发重绘. 失败仅记日志不弹框. */
static bool refresh_di(void)
{
	uint8_t bits[DI_COUNT];
	if (!MbClient_ReadDiscreteInputs(g_mb.mb, 0, DI_COUNT, bits)) {
		log_append(L"读 DI (FC02) 失败");
		g_mb.di_valid = false;
		return false;
	}
	memcpy(g_mb.di_bits, bits, DI_COUNT);
	g_mb.di_valid = true;
	/* 触发 16 个 LED STATIC 重绘 (InvalidateRect 触发 WM_CTLCOLORSTATIC) */
	for (int i = 0; i < DI_COUNT; i++) {
		if (g_mb.hDi[i]) InvalidateRect(g_mb.hDi[i], NULL, TRUE);
	}
	return true;
}

/* 刷新 DO 面板: ReadCoils(0,8) → 更新按钮文字. 失败仅记日志不弹框. */
static bool refresh_do(void)
{
	uint8_t bits[DO_COUNT];
	if (!MbClient_ReadCoils(g_mb.mb, 0, DO_COUNT, bits)) {
		log_append(L"读 DO (FC01) 失败");
		return false;
	}
	for (int i = 0; i < DO_COUNT; i++) {
		wchar_t buf[32];
		swprintf(buf, 32, L"DO%d %ls", i + 1, bits[i] ? L"ON" : L"OFF");
		SetWindowTextW(g_mb.hDo[i], buf);
	}
	return true;
}

/* 刷新 AI 面板: ReadInput(1,4) → 更新 4 个文本.
 * 注: input offset 1-4 即 AI1-AI4 (offset 0 是固件版本). 失败仅记日志不弹框. */
static bool refresh_ai(void)
{
	uint16_t regs[AI_COUNT];
	if (!MbClient_ReadInput(g_mb.mb, 1, AI_COUNT, regs)) {
		log_append(L"读 AI (FC04) 失败");
		return false;
	}
	for (int i = 0; i < AI_COUNT; i++) {
		wchar_t buf[64];
		if (i < 2) {
			swprintf(buf, 64, L"AI%d: %.2f mA", i + 1, regs[i] / 100.0);
		} else {
			swprintf(buf, 64, L"AI%d: %.2f V",  i + 1, regs[i] / 100.0);
		}
		SetWindowTextW(g_mb.hAi[i], buf);
	}
	return true;
}

/* 读单个 holding 行 (按 addr). 时间戳 0x0E 需读 0x0E+0x0F 合并为 32 位.
 * 返回是否成功. */
static bool read_holding_row(uint16_t addr, uint32_t *out)
{
	if (addr == 0x0E) {
		uint16_t hi = 0, lo = 0;
		if (!MbClient_ReadHolding(g_mb.mb, 0x0E, 1, &hi)) return false;
		if (!MbClient_ReadHolding(g_mb.mb, 0x0F, 1, &lo)) return false;
		*out = ((uint32_t)hi << 16) | lo;
		return true;
	}
	uint16_t v = 0;
	if (!MbClient_ReadHolding(g_mb.mb, addr, 1, &v)) return false;
	*out = v;
	return true;
}

/* 刷新寄存器表全部 23 行: 17 holding (时间戳两字合并) + 6 input. 读失败记日志但继续. */
static void refresh_reg_table(void)
{
	uint32_t vals[REG_COUNT];
	bool ok[REG_COUNT];

	/* holding 18 个连续读 (offset 0x00-0x11, 一次 FC03 读 18 个); 失败改逐个读.
	 * 逐个读只对 "设备有应答但拒绝整段" 有意义; 完全无响应 (超时) 时逐个读
	 * 只会每个再等 ~1s 超时, 直接整批标失败. */
	uint16_t hold[HOLDING_PHYS_COUNT];
	bool hold_ok = MbClient_ReadHolding(g_mb.mb, 0, HOLDING_PHYS_COUNT, hold);
	bool hold_each = false;
	if (!hold_ok) {
		hold_each = !MbClient_LastNoResponse(g_mb.mb);
		log_append(hold_each ? L"批量读 holding 失败, 改逐个读"
		                      : L"批量读 holding 失败 (设备无响应)");
	}
	for (int i = 0; i < HOLDING_COUNT; i++) {
		const RegMeta *r = &g_regs[i];
		if (hold_ok) {
			ok[i] = true;
			if (r->addr == 0x0E) {
				/* 时间戳: 高16位在 0x0E, 低16位在 0x0F */
				vals[i] = ((uint32_t)hold[0x0E] << 16) | hold[0x0F];
			} else {
				vals[i] = hold[r->addr];
			}
		} else {
			ok[i] = hold_each && read_holding_row(r->addr, &vals[i]);
		}
	}
	/* input 6 个连续读 (offset 0x00-0x05); 失败改逐个读 (无响应同理跳过) */
	uint16_t inp[6];
	bool inp_ok = MbClient_ReadInput(g_mb.mb, 0, 6, inp);
	bool inp_each = false;
	if (!inp_ok) {
		inp_each = !MbClient_LastNoResponse(g_mb.mb);
		log_append(inp_each ? L"批量读 input 失败, 改逐个读"
		                      : L"批量读 input 失败 (设备无响应)");
	}
	for (int i = 0; i < 6; i++) {
		int row = HOLDING_COUNT + i;
		if (inp_ok) {
			ok[row] = true;
			vals[row] = inp[i];
		} else {
			uint16_t v = 0;
			ok[row] = inp_each && MbClient_ReadInput(g_mb.mb, g_regs[row].addr, 1, &v);
			vals[row] = v;
		}
	}
	/* 更新 ListView 行 */
	for (int i = 0; i < REG_COUNT; i++) {
		if (ok[i]) {
			update_listview_row(i, vals[i]);
		} else {
			ListView_SetItemText(g_mb.hRegList, i, 2, L"(读取失败)");
		}
	}
}

/* ===== 连接 / 断开 ===== */

static void on_connect(void)
{
	/* 已连接 → 切换为断开 */
	if (g_mb.connected) {
		MbClient_Disconnect(g_mb.mb);
		set_conn_state(false);
		log_append(L"已断开");
		return;
	}

	int rtu = current_channel();
	uint8_t uid = 1;

	if (rtu) {
		/* RTU: COM + baud + uid */
		wchar_t wbuf[16];
		GetWindowTextW(g_mb.hUidRtu, wbuf, 16);
		int v = _wtoi(wbuf);
		if (v < 1 || v > 247) {
			MessageBoxW(g_mb.hSelf, L"从机 ID 应在 1-247", L"输入错误",
			            MB_ICONERROR);
			return;
		}
		uid = (uint8_t)v;
		int sel = (int)SendMessageW(g_mb.hCom, CB_GETCURSEL, 0, 0);
		if (sel < 0) {
			MessageBoxW(g_mb.hSelf, L"请选择串口", L"输入错误",
			            MB_ICONERROR);
			return;
		}
		wchar_t com_text[32] = {0};
		SendMessageW(g_mb.hCom, CB_GETLBTEXT, sel, (LPARAM)com_text);
		/* com_text 形如 "COM3"; 转为设备路径 \\.\COM3 */
		wchar_t com_path[40];
		swprintf(com_path, 40, L"\\\\.\\%ls", com_text);
		int bsel = (int)SendMessageW(g_mb.hBaud, CB_GETCURSEL, 0, 0);
		uint32_t baud = (bsel >= 0 && bsel < BAUD_COUNT) ? (uint32_t)g_bauds[bsel] : 9600;

		wchar_t m[128];
		swprintf(m, 128, L"正在连接 %ls @ %u (uid=%u)...", com_text, baud, uid);
		log_append(m);
		/* 真正进入阻塞连接前禁用按钮, 防止重入. 校验类失败在上面已 return. */
		EnableWindow(g_mb.hConn, FALSE);
		SetWindowTextW(g_mb.hConn, L"正在连接...");
		bool ok = MbClient_ConnectRtu(g_mb.mb, com_path, baud, uid);
		if (!ok) {
			show_mb_error(L"RTU 连接");
			SetWindowTextW(g_mb.hConn, L"连接");
			EnableWindow(g_mb.hConn, TRUE);
			return;
		}
	} else {
		/* TCP: IP + port (Modbus TCP 不需要 UID, unit id 固定 1) */
		uint8_t ip4[4];
		if (!read_ip_edit(g_mb.hIp, ip4)) {
			MessageBoxW(g_mb.hSelf,
			            L"目标 IP 格式错误, 请输入 a.b.c.d (每段 0-255)",
			            L"输入错误", MB_ICONERROR);
			return;
		}
		char ip[32];
		snprintf(ip, sizeof(ip), "%u.%u.%u.%u", ip4[0], ip4[1], ip4[2], ip4[3]);
		wchar_t wp[16];
		GetWindowTextW(g_mb.hPort, wp, 16);
		int port = _wtoi(wp);
		if (port <= 0 || port > 65535) port = 502;

		wchar_t m[128];
		swprintf(m, 128, L"正在连接 %hs:%d...", ip, port);
		log_append(m);
		/* 真正进入阻塞连接前禁用按钮, 防止重入. */
		EnableWindow(g_mb.hConn, FALSE);
		SetWindowTextW(g_mb.hConn, L"正在连接...");
		bool ok = MbClient_ConnectTcp(g_mb.mb, ip, (uint16_t)port, 1);
		if (!ok) {
			show_mb_error(L"TCP 连接");
			SetWindowTextW(g_mb.hConn, L"连接");
			EnableWindow(g_mb.hConn, TRUE);
			return;
		}
	}

	set_conn_state(true);
	/* 连接后立即加载所有面板 + 寄存器表的实际值.
	 * 首次读即完全无响应 (非 Modbus 设备 / 波特率或 UID 不对): 跳过剩余刷新,
	 * 否则每个读取各等 ~1s 超时会把界面卡住半分钟. */
	if (!refresh_do() && MbClient_LastNoResponse(g_mb.mb)) {
		log_append(L"设备无响应 (非 Modbus RTU 设备, 或波特率/UID 不匹配), 已跳过初始刷新");
	} else {
		refresh_di();
		refresh_ai();
		refresh_reg_table();
	}
	wchar_t m[64];
	swprintf(m, 64, L"已连接 (%ls)", rtu ? L"RTU" : L"TCP");
	log_append(m);
}

/* ===== DO 按钮 / 查询选中 / 双击写 ===== */

/* DO 按钮点击: 取当前按钮文字判断状态 → 翻转 → WriteSingleCoil → 回读 */
static void on_do_click(int idx)
{
	/* 从按钮文字提取当前 ON/OFF (文字形如 "DO3 ON") */
	wchar_t buf[32];
	GetWindowTextW(g_mb.hDo[idx], buf, 32);
	bool cur_on = wcsstr(buf, L"ON") != NULL;
	bool new_on = !cur_on;
	if (!MbClient_WriteSingleCoil(g_mb.mb, (uint16_t)idx, new_on)) {
		show_mb_error(L"写 DO (FC05)");
		return;
	}
	/* 回读全部 DO (设备可能多端控制, 以设备实际状态为准) */
	refresh_do();
}

/* 查询选中行: 读单寄存器 → 更新当前值列. */
static void on_query_selected(void)
{
	int sel = ListView_GetNextItem(g_mb.hRegList, -1, LVNI_SELECTED);
	if (sel < 0 || sel >= REG_COUNT) {
		MessageBoxW(g_mb.hSelf, L"请先在列表中选择一行", L"提示",
		            MB_ICONWARNING);
		return;
	}
	const RegMeta *r = &g_regs[sel];
	uint32_t val = 0;
	bool ok;
	if (r->is_input) {
		uint16_t v = 0;
		ok = MbClient_ReadInput(g_mb.mb, r->addr, 1, &v);
		val = v;
	} else if (r->addr == 0x0E) {
		ok = read_holding_row(0x0E, &val);   /* 时间戳: 合并 0x0E+0x0F */
	} else {
		uint16_t v = 0;
		ok = MbClient_ReadHolding(g_mb.mb, r->addr, 1, &v);
		val = v;
	}
	if (!ok) {
		show_mb_error(L"查询寄存器");
		ListView_SetItemText(g_mb.hRegList, sel, 2, L"(读取失败)");
		return;
	}
	update_listview_row(sel, val);
	wchar_t m[128];
	swprintf(m, 128, L"查询 %ls = %u", r->name, (unsigned)val);
	log_append(m);
}

/* Win32 标准无 InputBox, 这里注册临时窗口类, 创建弹出窗口 (EDIT + OK/Cancel),
 * 通过私有消息循环实现模态. 不依赖 dialog resource. */
static wchar_t g_input_buf[32];
static HWND g_hInputEdit;
static HWND g_hInputDlg;
static bool g_input_confirmed;

static LRESULT CALLBACK input_wnd_proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE: {
		g_hInputEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", g_input_buf,
			WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL,
			15, 15, 330, 24, hWnd, (HMENU)100, g_hInst, NULL);
		SendMessageW(g_hInputEdit, WM_SETFONT, (WPARAM)g_hFont, TRUE);
		/* 两按钮在 360 宽客户区居中: 总宽 80+10+80=170, 起始 x=(360-170)/2=95 */
		HWND hCancel = CreateWindowExW(0, L"BUTTON", L"取消",
			WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
			95, 50, 80, 26, hWnd, (HMENU)IDCANCEL, g_hInst, NULL);
		SendMessageW(hCancel, WM_SETFONT, (WPARAM)g_hFont, TRUE);
		HWND hOk = CreateWindowExW(0, L"BUTTON", L"确定",
			WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_DEFPUSHBUTTON,
			185, 50, 80, 26, hWnd, (HMENU)IDOK, g_hInst, NULL);
		SendMessageW(hOk, WM_SETFONT, (WPARAM)g_hFont, TRUE);
		return 0;
	}
	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK) {
			GetWindowTextW(g_hInputEdit, g_input_buf, 32);
			g_input_confirmed = true;
			PostQuitMessage(0);
			return 0;
		}
		if (LOWORD(wParam) == IDCANCEL) {
			g_input_confirmed = false;
			PostQuitMessage(0);
			return 0;
		}
		break;
	case WM_CLOSE:
		g_input_confirmed = false;
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

/* 通用字符串输入弹窗: 预填 buf, 确定后写回 buf. 取消返回 false.
 * 复用与 prompt_uint_modal 相同的临时窗口类 (编辑框 + OK/Cancel). */
static bool prompt_str_modal(const wchar_t *title, wchar_t *buf, int cap)
{
	static const wchar_t *CLS = L"ioEdgeHubInputBoxCls";
	static BOOL registered = FALSE;
	if (!registered) {
		WNDCLASSW wc = {0};
		wc.lpfnWndProc = input_wnd_proc;
		wc.hInstance = g_hInst;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
		wc.lpszClassName = CLS;
		RegisterClassW(&wc);
		registered = TRUE;
	}

	wcscpy_s(g_input_buf, 31, buf);   /* WM_CREATE 用它预填编辑框 */
	g_input_confirmed = false;
	g_hInputDlg = CreateWindowExW(WS_EX_DLGMODALFRAME, CLS, title,
		WS_POPUP | WS_CAPTION | WS_SYSMENU,
		0, 0, 360, 130,
		g_mb.hSelf, NULL, g_hInst, NULL);
	if (!g_hInputDlg) return false;

	/* 弹窗居中到主窗口 (主窗口不可见时退回桌面). 避免系统级联默认落左上角. */
	RECT rcDlg, rcParent;
	GetWindowRect(g_hInputDlg, &rcDlg);
	HWND hParent = IsWindow(g_hMain) ? g_hMain : GetDesktopWindow();
	GetWindowRect(hParent, &rcParent);
	int dlgW = rcDlg.right - rcDlg.left;
	int dlgH = rcDlg.bottom - rcDlg.top;
	int px = rcParent.left + ((rcParent.right - rcParent.left) - dlgW) / 2;
	int py = rcParent.top  + ((rcParent.bottom - rcParent.top) - dlgH) / 2;
	SetWindowPos(g_hInputDlg, NULL, px, py, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	EnableWindow(g_mb.hSelf, FALSE);
	ShowWindow(g_hInputDlg, SW_SHOW);
	UpdateWindow(g_hInputDlg);
	SetFocus(g_hInputEdit);

	/* 模态消息循环 */
	MSG msg;
	while (GetMessageW(&msg, NULL, 0, 0) > 0) {
		if (msg.message == WM_QUIT) break;
		if (!IsDialogMessageW(g_hInputDlg, &msg)) {
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	EnableWindow(g_mb.hSelf, TRUE);
	SetFocus(g_mb.hSelf);
	HWND hDlg = g_hInputDlg;
	g_hInputDlg = NULL;
	if (hDlg) DestroyWindow(hDlg);

	if (!g_input_confirmed) return false;
	wcscpy_s(buf, cap, g_input_buf);
	return true;
}

/* 数字输入弹窗 (支持 0x 十六进制 和 十进制, 值 ≤ 0xFFFF). 取消/非法返回 false. */
static bool prompt_uint_modal(const wchar_t *title, unsigned *out_val)
{
	wchar_t buf[32];
	buf[0] = L'\0';
	if (!prompt_str_modal(title, buf, 32)) return false;

	/* 解析输入: 支持 0x 十六进制 和 十进制 */
	wchar_t *end = NULL;
	unsigned long v;
	if (wcsncmp(buf, L"0x", 2) == 0 || wcsncmp(buf, L"0X", 2) == 0) {
		v = wcstoul(buf + 2, &end, 16);
	} else {
		v = wcstoul(buf, &end, 10);
	}
	if (end == buf) return false;
	if (v > 0xFFFF) return false;
	*out_val = (unsigned)v;
	return true;
}

/* 把 Unix 秒格式化为本机时区 "YYYY-MM-DD HH:MM:SS". */
static void format_time_local(time_t t, wchar_t *out, int cap)
{
	struct tm tmv;
	if (localtime_s(&tmv, &t) != 0) {
		out[0] = L'\0';
		return;
	}
	wcsftime(out, cap, L"%Y-%m-%d %H:%M:%S", &tmv);
}

/* 双击"时间戳"行: 以具体时间设置设备 RTC.
 * 设备协议: 写 0x0E(高16位) → 写 0x0F(低16位) 触发 set_timestamp. */
static void on_set_timestamp(void)
{
	if (!g_mb.connected) {
		MessageBoxW(g_mb.hSelf, L"未连接", L"提示", MB_ICONWARNING);
		return;
	}
	wchar_t buf[32];
	format_time_local(time(NULL), buf, 32);
	if (!prompt_str_modal(L"设置设备时间 (YYYY-MM-DD HH:MM:SS)", buf, 32)) return;

	int y, mo, d, h, mi, s;
	if (swscanf(buf, L"%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6) {
		MessageBoxW(g_mb.hSelf, L"时间格式错误, 应为 YYYY-MM-DD HH:MM:SS", L"输入错误",
		            MB_ICONERROR);
		return;
	}
	struct tm tmv;
	memset(&tmv, 0, sizeof(tmv));
	tmv.tm_year = y - 1900;
	tmv.tm_mon = mo - 1;
	tmv.tm_mday = d;
	tmv.tm_hour = h;
	tmv.tm_min = mi;
	tmv.tm_sec = s;
	tmv.tm_isdst = -1;
	time_t t = mktime(&tmv);
	if (t == (time_t)-1 || t < 0) {
		MessageBoxW(g_mb.hSelf, L"时间无效", L"输入错误", MB_ICONERROR);
		return;
	}
	uint32_t ts = (uint32_t)t;
	uint16_t hi = (uint16_t)(ts >> 16);
	uint16_t lo = (uint16_t)(ts & 0xFFFF);
	if (!MbClient_WriteSingleReg(g_mb.mb, 0x0E, hi)) {
		show_mb_error(L"写时间戳高字 (FC06)");
		return;
	}
	if (!MbClient_WriteSingleReg(g_mb.mb, 0x0F, lo)) {
		show_mb_error(L"写时间戳低字 (FC06)");
		return;
	}
	/* 读回刷新 (设备读 0x0E/0x0F 返回实时时间) */
	uint32_t rb = 0;
	if (read_holding_row(0x0E, &rb)) {
		for (int i = 0; i < REG_COUNT; i++) {
			if (!g_regs[i].is_input && g_regs[i].addr == 0x0E) {
				update_listview_row(i, rb);
				break;
			}
		}
	}
	wchar_t m[128];
	swprintf(m, 128, L"已设置设备时间 %ls", buf);
	log_append(m);
}

/* 双击 ListView 行: 按 rw 决定读/写. */
static void on_listview_dblclk(void)
{
	int sel = ListView_GetNextItem(g_mb.hRegList, -1, LVNI_SELECTED);
	if (sel < 0 || sel >= REG_COUNT) return;
	const RegMeta *r = &g_regs[sel];

	if (r->rw == RW_RO) {
		MessageBoxW(g_mb.hSelf, L"该寄存器只读", L"提示", MB_ICONINFORMATION);
		return;
	}
	if (r->is_input) {
		MessageBoxW(g_mb.hSelf, L"输入寄存器不可写", L"提示", MB_ICONINFORMATION);
		return;
	}
	/* 时间戳行 (holding 0x0E): 以具体时间设置设备 RTC */
	if (r->addr == 0x0E) {
		on_set_timestamp();
		return;
	}
	if (r->rw == RW_WO || r->rw == RW_WO_TRIG) {
		/* 只写触发 (保存/重启): 写 1 触发, 弹确认 */
		wchar_t m[128];
		swprintf(m, 128, L"确认向 0x%02X (%ls) 写入触发值 1?", r->addr, r->name);
		if (MessageBoxW(g_mb.hSelf, m, L"触发确认", MB_YESNO | MB_ICONQUESTION) != IDYES) {
			return;
		}
		if (!MbClient_WriteSingleReg(g_mb.mb, r->addr, 1)) {
			show_mb_error(L"写触发寄存器 (FC06)");
			return;
		}
		log_append(L"已写入触发值 1");
		return;
	}
	/* RW: 弹输入框 */
	unsigned v = 0;
	wchar_t title[64];
	swprintf(title, 64, L"输入 %ls (0x%02X) 的新值", r->name, r->addr);
	if (!prompt_uint_modal(title, &v)) return;
	if (!MbClient_WriteSingleReg(g_mb.mb, r->addr, (uint16_t)v)) {
		show_mb_error(L"写寄存器 (FC06)");
		return;
	}
	/* 写后立即回读刷新该行 */
	uint16_t rb = 0;
	if (MbClient_ReadHolding(g_mb.mb, r->addr, 1, &rb)) {
		update_listview_row(sel, rb);
	}
	wchar_t m[128];
	swprintf(m, 128, L"已写 %ls = %u (回读 %u)", r->name, v, rb);
	log_append(m);
}

/* ===== 刷新全部 ===== */

static void on_refresh_all(void)
{
	if (!g_mb.connected) {
		MessageBoxW(g_mb.hSelf, L"未连接", L"提示", MB_ICONWARNING);
		return;
	}
	log_append(L"开始刷新全部...");
	bool a = refresh_do();
	if (!a && MbClient_LastNoResponse(g_mb.mb)) {
		/* 设备完全无响应: 剩余读取必然同样超时, 跳过 (避免界面长时间卡住) */
		log_append(L"设备无响应, 跳过本轮剩余刷新");
		return;
	}
	bool b = refresh_di();
	bool c = refresh_ai();
	refresh_reg_table();
	/* 刷新中发现连接断开: 停止自动刷新并复位连接状态 (不弹框) */
	if (!MbClient_IsConnected(g_mb.mb)) {
		on_link_lost();
		return;
	}
	wchar_t m[96];
	swprintf(m, 96, L"刷新完成 (DO=%ls, DI=%ls, AI=%ls, 表已更新)",
	         a ? L"OK" : L"失败", b ? L"OK" : L"失败", c ? L"OK" : L"失败");
	log_append(m);
}

/* 自动刷新 WM_TIMER 回调: 刷 DI/DO/AI + 全部保持/输入寄存器表.
 * 全程不弹窗: 失败仅记日志, 若传输层检测到连接断开则停止自动刷新. */
static void on_timer(void)
{
	if (!g_mb.connected) return;
	bool ok = refresh_do();
	if (!ok && MbClient_LastNoResponse(g_mb.mb)) {
		return;   /* 设备无响应: 跳过本轮剩余, 避免逐项超时拖住 UI */
	}
	ok = refresh_di() && ok;
	ok = refresh_ai() && ok;
	/* 连接仍有效时同时刷新全部保持/输入寄存器 */
	if (MbClient_IsConnected(g_mb.mb)) {
		refresh_reg_table();
	}
	if (!ok && !MbClient_IsConnected(g_mb.mb)) {
		on_link_lost();
	}
}

/* 切换自动刷新开关. */
static void on_autoref_toggle(void)
{
	bool checked = (SendMessageW(g_mb.hAutoRef, BM_GETCHECK, 0, 0) == BST_CHECKED);
	if (checked) {
		wchar_t wb[16];
		GetWindowTextW(g_mb.hAutoRefInt, wb, 16);
		int ms = _wtoi(wb);
		if (ms < 100) ms = 1000;  /* 防止过小 */
		SetTimer(g_mb.hSelf, IDC_MB_TIMER, (UINT)ms, NULL);
		g_mb.auto_timer = true;
		log_append(L"自动刷新已开启");
	} else {
		if (g_mb.auto_timer) {
			KillTimer(g_mb.hSelf, IDC_MB_TIMER);
			g_mb.auto_timer = false;
		}
		log_append(L"自动刷新已关闭");
	}
}

/* ===== WM_COMMAND 分发 ===== */

static void on_command(WPARAM wParam)
{
	WORD id = LOWORD(wParam);
	WORD code = HIWORD(wParam);

	/* 通道单选切换 */
	if (id == IDC_MB_CHAN_TCP && code == BN_CLICKED) {
		apply_channel_visibility();
		return;
	}
	if (id == IDC_MB_CHAN_RTU && code == BN_CLICKED) {
		apply_channel_visibility();
		return;
	}
	if (code != BN_CLICKED) return;

	/* DO 按钮 */
	if (id >= IDC_MB_DO_BASE && id < IDC_MB_DO_BASE + DO_COUNT) {
		on_do_click(id - IDC_MB_DO_BASE);
		return;
	}

	switch (id) {
	case IDC_MB_CONNECT:    on_connect(); break;
	case IDC_MB_REFRESH_ALL: on_refresh_all(); break;
	case IDC_MB_AUTOREF:    on_autoref_toggle(); break;
	case IDC_MB_REG_QUERY:  on_query_selected(); break;
	}
}

/* ===== WM_CREATE: 创建所有控件 ===== */

static void create_controls(HWND hWnd)
{
	g_mb.hSelf = hWnd;
	g_hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

	/* 行坐标基准 */
	int gx = 12, gw = 776;

	/* ===== 连接 groupbox ===== */
	create_groupbox(L"连接", gx, 4, gw, 90);
	/* 行1: 通道单选 */
	create_label(L"通道:", gx + 12, 30, 40, 14);
	g_mb.hChanTcp = CreateWindowExW(0, L"BUTTON", L"TCP",
		WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP,
		gx + 56, 26, 60, 22, hWnd, (HMENU)(INT_PTR)IDC_MB_CHAN_TCP, g_hInst, NULL);
	g_mb.hChanRtu = CreateWindowExW(0, L"BUTTON", L"RTU",
		WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
		gx + 120, 26, 60, 22, hWnd, (HMENU)(INT_PTR)IDC_MB_CHAN_RTU, g_hInst, NULL);
	SendMessageW(g_mb.hChanTcp, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	SendMessageW(g_mb.hChanRtu, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	SendMessageW(g_mb.hChanTcp, BM_SETCHECK, BST_CHECKED, 0); /* 默认 TCP */
	/* 连接/断开 + 状态 */
	g_mb.hConn = create_button(L"连接", gx + 200, 26, 70, 22, IDC_MB_CONNECT);
	create_label(L"状态:", gx + 352, 30, 36, 14);
	g_mb.hStatus = create_label(L"○ 未连接", gx + 388, 30, 120, 14);

	/* 行2: TCP 行 (默认显示) — IP + 端口 */
	g_mb.hTcpLbl1 = create_label(L"目标 IP:", gx + 12, 60, 50, 14);
	g_mb.hIp = create_edit(gx + 64, 56, 140, 22, IDC_MB_IP1, 0);
	/* 默认填 192.168.12.101 */
	SetWindowTextW(g_mb.hIp, L"192.168.12.101");
	g_mb.hTcpLbl2 = create_label(L"端口:", gx + 230, 60, 32, 14);
	g_mb.hPort = create_edit(gx + 262, 56, 48, 22, IDC_MB_PORT, ES_NUMBER);
	SetWindowTextW(g_mb.hPort, L"502");

	/* 行2 (重叠位置, 默认隐藏): RTU 行 — COM + 波特率 + uid */
	g_mb.hRtuLbl1 = create_label(L"串口:", gx + 12, 60, 32, 14);
	g_mb.hCom = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
		gx + 44, 56, 90, 240, hWnd, (HMENU)(INT_PTR)IDC_MB_COM, g_hInst, NULL);
	SendMessageW(g_mb.hCom, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	/* 自动枚举实际存在的 COM 口 (非硬编码 COM1..32) */
	enumerate_com_ports(g_mb.hCom);
	g_mb.hRtuLbl2 = create_label(L"波特率:", gx + 144, 60, 44, 14);
	g_mb.hBaud = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
		gx + 188, 56, 90, 240, hWnd, (HMENU)(INT_PTR)IDC_MB_BAUD, g_hInst, NULL);
	SendMessageW(g_mb.hBaud, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	for (int i = 0; i < BAUD_COUNT; i++) {
		wchar_t buf[16];
		swprintf(buf, 16, L"%d", g_bauds[i]);
		SendMessageW(g_mb.hBaud, CB_ADDSTRING, 0, (LPARAM)buf);
	}
	SendMessageW(g_mb.hBaud, CB_SETCURSEL, 1, 0); /* 默认 9600 */
	g_mb.hRtuLbl3 = create_label(L"UID:", gx + 290, 60, 30, 14);
	g_mb.hUidRtu = create_edit(gx + 320, 56, 40, 22, IDC_MB_UID, ES_NUMBER);
	SetWindowTextW(g_mb.hUidRtu, L"1");

	/* 默认 TCP, 隐藏 RTU 行 */
	apply_channel_visibility();

	/* 初始按钮状态: 未连接时禁用刷新 */
	EnableWindow(g_mb.hRefreshAll, FALSE);

	/* ===== DI 面板 groupbox ===== */
	g_mb.hGbDi = create_groupbox(L"DI (16 路数字输入, 只读)", gx, 110, gw, 56);
	/* 16 个 LED (STATIC): 单行 16 个, 均匀铺开 */
	int di_x = gx + 12;
	int di_y = 132;
	for (int i = 0; i < DI_COUNT; i++) {
		wchar_t lbl[8];
		swprintf(lbl, 8, L"%d", i + 1);
		HWND hw = CreateWindowExW(0, L"STATIC", lbl,
			WS_CHILD | WS_VISIBLE | SS_CENTER,
			di_x + i * 44, di_y, 40, 22, hWnd,
			(HMENU)(INT_PTR)(IDC_MB_DI_BASE + i), g_hInst, NULL);
		SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
		g_mb.hDi[i] = hw;
	}

	/* ===== DO 面板 groupbox ===== */
	g_mb.hGbDo = create_groupbox(L"DO (8 路数字输出, 点击切换)", gx, 182, gw, 56);
	int do_x = gx + 12;
	int do_y = 204;
	for (int i = 0; i < DO_COUNT; i++) {
		wchar_t lbl[32];
		swprintf(lbl, 32, L"DO%d ?", i + 1);
		HWND hw = create_button(lbl, do_x + i * 88, do_y, 82, 24,
		                        IDC_MB_DO_BASE + i);
		g_mb.hDo[i] = hw;
	}

	/* ===== AI 面板 groupbox ===== */
	g_mb.hGbAi = create_groupbox(L"AI (4 路模拟输入)", gx, 254, gw, 50);
	int ai_x = gx + 12;
	int ai_y = 276;
	for (int i = 0; i < AI_COUNT; i++) {
		HWND hw = create_label(L"--", ai_x + i * 180, ai_y, 170, 18);
		g_mb.hAi[i] = hw;
	}

	/* ===== 寄存器表 groupbox ===== */
	g_mb.hGbReg = create_groupbox(L"寄存器表 (双击写 RW, 选中后点查询读)", gx, 312, gw, 260);
	/* 刷新全部 + 自动刷新 (放在寄存器表上方) */
	g_mb.hRefreshAll = create_button(L"刷新全部", gx + 12, 332, 80, 22, IDC_MB_REFRESH_ALL);
	g_mb.hAutoRef = CreateWindowExW(0, L"BUTTON", L"自动刷新",
		WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
		gx + 100, 332, 80, 22, hWnd, (HMENU)(INT_PTR)IDC_MB_AUTOREF, g_hInst, NULL);
	SendMessageW(g_mb.hAutoRef, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	g_mb.hAutoRefLbl = create_label(L"间隔(ms):", gx + 184, 336, 56, 14);
	g_mb.hAutoRefInt = create_edit(gx + 240, 332, 60, 22, IDC_MB_AUTOREF_INT, ES_NUMBER);
	SetWindowTextW(g_mb.hAutoRefInt, L"500");
	/* 查询选中 */
	g_mb.hRegQuery = create_button(L"查询选中", gx + 320, 332, 80, 22, IDC_MB_REG_QUERY);
	g_mb.hRegHint = create_label(L"(提示: 双击 RW 行可写入; WO 行写触发值; RO 行只读)",
		gx + 408, 336, 300, 14);

	/* ListView */
	g_mb.hRegList = CreateWindowExW(0, WC_LISTVIEWW, L"",
		WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL | LVS_SHOWSELALWAYS | WS_BORDER,
		gx + 12, 360, gw - 24, 200, hWnd,
		(HMENU)(INT_PTR)IDC_MB_REG_LIST, g_hInst, NULL);
	SendMessageW(g_mb.hRegList, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	ListView_SetExtendedListViewStyle(g_mb.hRegList,
		LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);

	/* 列头 */
	LVCOLUMNW col;
	memset(&col, 0, sizeof(col));
	col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;
	col.cx = 60;  col.pszText = (LPWSTR)L"地址";   col.iSubItem = 0;
	ListView_InsertColumn(g_mb.hRegList, 0, &col);
	col.cx = 150; col.pszText = (LPWSTR)L"名称";   col.iSubItem = 1;
	ListView_InsertColumn(g_mb.hRegList, 1, &col);
	col.cx = 220; col.pszText = (LPWSTR)L"当前值"; col.iSubItem = 2;
	ListView_InsertColumn(g_mb.hRegList, 2, &col);
	col.cx = 80;  col.pszText = (LPWSTR)L"R/W";    col.iSubItem = 3;
	ListView_InsertColumn(g_mb.hRegList, 3, &col);

	/* 23 行 (17 holding + 6 input) */
	for (int i = 0; i < REG_COUNT; i++) {
		const RegMeta *r = &g_regs[i];
		wchar_t addr_str[16];
		swprintf(addr_str, 16, L"%d", r->is_input ? (30001 + r->addr) : (40001 + r->addr));
		LVITEMW it;
		memset(&it, 0, sizeof(it));
		it.mask = LVIF_TEXT;
		it.iItem = i;
		it.iSubItem = 0;
		it.pszText = addr_str;
		ListView_InsertItem(g_mb.hRegList, &it);
		ListView_SetItemText(g_mb.hRegList, i, 1, (LPWSTR)r->name);
		ListView_SetItemText(g_mb.hRegList, i, 2, (LPWSTR)L"(未读取)");
		ListView_SetItemText(g_mb.hRegList, i, 3, (LPWSTR)rw_label(r->rw));
	}

	/* ===== 操作日志 groupbox ===== */
	create_groupbox(L"操作日志", gx, 584, gw, 184);
	g_mb.hLog = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY |
		ES_AUTOVSCROLL | WS_VSCROLL,
		gx + 12, 604, gw - 24, 156,
		hWnd, (HMENU)(INT_PTR)IDC_MB_LOG, g_hInst, NULL);
	SendMessageW(g_mb.hLog, WM_SETFONT, (WPARAM)g_hFont, TRUE);
}

/* ===== 窗口过程 ===== */

static LRESULT CALLBACK mb_wndproc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		g_hInst = ((LPCREATESTRUCT)lParam)->hInstance;
		create_controls(hWnd);
		g_mb.mb = MbClient_Create();
		if (!g_mb.mb) {
			log_append(L"错误: MbClient 创建失败");
		} else {
			log_append(L"就绪. 请选择通道 (TCP/RTU) 并连接设备");
		}
		return 0;
	case WM_COMMAND:
		on_command(wParam);
		return 0;
	case WM_NOTIFY: {
		LPNMHDR pnmh = (LPNMHDR)lParam;
		if (pnmh->hwndFrom == g_mb.hRegList && pnmh->code == NM_DBLCLK) {
			on_listview_dblclk();
		}
		return 0;
	}
	case WM_TIMER:
		if (wParam == IDC_MB_TIMER) {
			on_timer();
		}
		return 0;
	case WM_SIZE:
		/* 控件保持固定位置 (与 tab1/tab2 一致). */
		return 0;
	case WM_CTLCOLORDLG:
		/* 对话框底色 BTNFACE */
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	case WM_CTLCOLORSTATIC: {
		HDC hdc = (HDC)wParam;
		HWND hCtrl = (HWND)lParam;
		/* 只读多行日志 EDIT 不能用 TRANSPARENT (会残留旧文字), 用不透明背景.
		 * 注: 只读 EDIT 也走 WM_CTLCOLORSTATIC. 放在 DI-LED 着色之前: 日志 EDIT
		 * 与 DI LED 控件 ID 不重叠, 两者互斥. */
		if (GetWindowLongPtrW(hCtrl, GWL_STYLE) & ES_READONLY) {
			SetBkMode(hdc, OPAQUE);
			SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
			SetBkColor(hdc, GetSysColor(COLOR_WINDOW));
			return (LRESULT)GetSysColorBrush(COLOR_WINDOW);
		}
		/* DI LED 着色: 控件 ID 在 3100..3115 范围.
		 * 三态使用预创建画刷 (进程级单例, 不释放), 避免 WM_CTLCOLORSTATIC 高频
		 * 调用导致的 GDI 句柄泄漏. */
		INT_PTR cid = GetWindowLongPtrW(hCtrl, GWLP_ID);
		if (cid >= IDC_MB_DI_BASE && cid < IDC_MB_DI_BASE + DI_COUNT) {
			static HBRUSH brOn = NULL, brOff = NULL, brUnk = NULL;
			if (!brOn)  brOn  = CreateSolidBrush(RGB(0, 200, 0));
			if (!brOff) brOff = CreateSolidBrush(RGB(128, 128, 128));
			if (!brUnk) brUnk = CreateSolidBrush(RGB(200, 200, 200));
			int idx = (int)(cid - IDC_MB_DI_BASE);
			SetBkMode(hdc, OPAQUE);
			SetTextColor(hdc, RGB(255, 255, 255));
			if (!g_mb.di_valid) {
				SetBkColor(hdc, RGB(200, 200, 200));
				return (LRESULT)brUnk;
			} else if (g_mb.di_bits[idx]) {
				SetBkColor(hdc, RGB(0, 200, 0));
				return (LRESULT)brOn;
			} else {
				SetBkColor(hdc, RGB(128, 128, 128));
				return (LRESULT)brOff;
			}
		}
		/* 其他 STATIC: 透明 + BTNFACE 底色 */
		SetBkMode(hdc, TRANSPARENT);
		SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	}
	case WM_DESTROY:
		if (g_mb.auto_timer) {
			KillTimer(hWnd, IDC_MB_TIMER);
			g_mb.auto_timer = false;
		}
		if (g_mb.mb) {
			if (g_mb.connected) {
				MbClient_Disconnect(g_mb.mb);
				g_mb.connected = false;
			}
			MbClient_Destroy(g_mb.mb);
			g_mb.mb = NULL;
		}
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

/* ===== 公共 API ===== */

HWND ModbusTab_Create(HWND hParent, HINSTANCE hInst)
{
	g_hInst = hInst;

	if (!g_classRegistered) {
		WNDCLASSW wc = {0};
		wc.lpfnWndProc = mb_wndproc;
		wc.hInstance = hInst;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
		wc.lpszClassName = MODBUS_TAB_CLASS;
		RegisterClassW(&wc);
		g_classRegistered = TRUE;
	}

	HWND h = CreateWindowExW(0, MODBUS_TAB_CLASS, L"",
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
		0, 0, 700, 520, hParent, NULL, hInst, NULL);
	return h;
}
