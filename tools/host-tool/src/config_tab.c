/* io-edge-hub 上位机 - Tab1 "UDP 参数配置"
 *
 * 程序化创建全部控件 (不用 dialog resource), 用静态 g_cfg 持有所有控件 HWND +
 * UdpManager 实例 (tab 生命周期内复用). WM_CREATE 创建控件 + UdpManager_Create;
 * WM_DESTROY 调 UdpManager_Destroy.
 *
 * 布局 (主窗口最小 720x560, tab 显示区约 712x504):
 *   - 设备发现 groupbox: 发现按钮 + 设备下拉 + 目标 IP (4 段) + 版本行 + 查询/重启
 *   - 网络参数 groupbox: 新 IP (4 段) + 应用 (持久化, 需手动重启生效)
 *   - Modbus 参数 groupbox: 从机地址 + 波特率下拉 + 应用/读取
 *   - 时间设置 groupbox: 应用本机时间到设备 RTC (UDP 0x14)
 *   - 出厂重置按钮
 *   - 操作日志 groupbox: 多行只读 EDIT (带时间戳)
 */
#include "udp_manager.h"   /* 须先于 windows.h 拉 winsock2.h (避免 winsock1 冲突) */
#include "config_tab.h"
#include "resource.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <wchar.h>

/* ===== 静态状态: 所有控件 HWND + UdpManager 实例 ===== */
typedef struct {
	HWND hSelf;
	HWND hConn;
	HWND hDevList, hIp, hNip, hVersion, hMbSlave, hMbBaud;
	HWND hTime;
	HWND hLog;
	UdpManager *udp;
	bool udp_connected;   /* UDP 连接状态 (连接后启用查询/设置) */
} ConfigTab;

static ConfigTab g_cfg;
static HFONT g_hFont = NULL;
static const wchar_t *CONFIG_TAB_CLASS = L"ioEdgeHubConfigTabCls";
static BOOL g_classRegistered = FALSE;

/* 标准波特率 (Modbus 下拉用) */
static const int g_bauds[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
#define BAUD_COUNT (int)(sizeof(g_bauds) / sizeof(g_bauds[0]))

/* ===== 控件创建辅助 ===== */

static HWND create_label(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"STATIC", text,
		WS_CHILD | WS_VISIBLE, x, y, w, h,
		g_cfg.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_edit(int x, int y, int w, int h, int id, DWORD extra)
{
	HWND hw = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | extra,
		x, y, w, h, g_cfg.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_button(const wchar_t *text, int x, int y, int w, int h, int id)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		x, y, w, h, g_cfg.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_groupbox(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
		x, y, w, h, g_cfg.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

/* 创建一段 4 段 IP 编辑框 (ES_NUMBER + 限 3 字符). 返回末尾 x 坐标. */
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

/* 从单个 IP 输入框读 "a.b.c.d", 写入 ip4[4].
 * 返回是否合法: 必须严格匹配 4 段, 每段 0-255. */
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

/* 当前目标 IP 拼成点分十进制窄字符串 ("a.b.c.d"). 不校验合法性, 调用方负责. */
static void current_target_ip(char *out, int cap)
{
	wchar_t buf[32];
	GetWindowTextW(g_cfg.hIp, buf, 32);
	WideCharToMultiByte(CP_ACP, 0, buf, -1, out, cap, NULL, NULL);
}

/* 日志框追加一行 (带 [HH:MM:SS] 时间戳). */
static void log_append(const wchar_t *msg)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	wchar_t line[600];
	swprintf(line, 600, L"[%02d:%02d:%02d] %ls\r\n",
	         st.wHour, st.wMinute, st.wSecond, msg);
	int len = GetWindowTextLengthW(g_cfg.hLog);
	SendMessageW(g_cfg.hLog, EM_SETSEL, len, len);
	SendMessageW(g_cfg.hLog, EM_REPLACESEL, 0, (LPARAM)line);
}

/* 刷新本机时间显示 (WM_TIMER 每秒调用). */
static void update_time_display(void)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	wchar_t buf[40];
	swprintf(buf, 40, L"%04d-%02d-%02d %02d:%02d:%02d",
	         st.wYear, st.wMonth, st.wDay,
	         st.wHour, st.wMinute, st.wSecond);
	SetWindowTextW(g_cfg.hTime, buf);
}

/* 通用: 传输失败时弹错误框 + 记日志. */
static void show_transport_error(const wchar_t *op)
{
	/* UdpManager_GetLastError 返回 UTF-8 char* (MSVC /utf-8 编译), 用
	 * MultiByteToWideChar(CP_UTF8) 转, 不能用 swprintf 的 %hs (按 CP_ACP 解). */
	const char *e = UdpManager_GetLastError(g_cfg.udp);
	wchar_t werr[192];
	MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
	wchar_t m[256];
	swprintf(m, 256, L"%ls 失败: %ls", op, werr);
	MessageBoxW(g_cfg.hSelf, m, L"错误", MB_ICONERROR);
	log_append(m);
}

/* 把 baud 值选中到下拉框 (若无匹配项则追加并选中). */
static void select_baud(uint16_t baud)
{
	for (int i = 0; i < BAUD_COUNT; i++) {
		if (g_bauds[i] == baud) {
			SendMessageW(g_cfg.hMbBaud, CB_SETCURSEL, i, 0);
			return;
		}
	}
	/* 非标准值: 追加 */
	wchar_t buf[16];
	swprintf(buf, 16, L"%u", baud);
	int idx = (int)SendMessageW(g_cfg.hMbBaud, CB_ADDSTRING, 0, (LPARAM)buf);
	SendMessageW(g_cfg.hMbBaud, CB_SETCURSEL, idx, 0);
}

/* ===== WM_CREATE: 创建所有控件 ===== */

static void create_controls(HWND hWnd)
{
	g_cfg.hSelf = hWnd;
	g_hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

	/* 行坐标基准 */
	int gx = 12, gw = 776;

	/* ===== 设备发现 groupbox ===== */
	create_groupbox(L"设备发现", gx, 4, gw, 112);
	/* 行1: 发现按钮 + 连接按钮 + 设备下拉 */
	create_button(L"发现设备", gx + 12, 28, 90, 24, IDC_CFG_DISCOVER_BTN);
	g_cfg.hConn = create_button(L"连接", gx + 108, 28, 80, 24, IDC_CFG_CONNECT);
	create_label(L"设备列表:", gx + 200, 32, 64, 14);
	g_cfg.hDevList = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP,
		gx + 264, 28, 280, 220, hWnd, (HMENU)(INT_PTR)IDC_CFG_DEVLIST, g_hInst, NULL);
	SendMessageW(g_cfg.hDevList, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	/* 行2: 目标设备 IP (单框, 整体输入 "a.b.c.d") */
	create_label(L"目标设备 IP:", gx + 12, 62, 90, 14);
	g_cfg.hIp = create_edit(gx + 104, 58, 140, 22, IDC_CFG_IP1, 0);
	/* 行3: 版本 + 查询 + 重启 (按钮右对齐) */
	create_label(L"版本:", gx + 12, 92, 40, 14);
	g_cfg.hVersion = create_label(L"(未查询)", gx + 54, 92, 280, 14);
	create_button(L"查询版本", gx + gw - 180, 88, 80, 24, IDC_CFG_GETVER);
	create_button(L"重启", gx + gw - 96, 88, 80, 24, IDC_CFG_REBOOT);

	/* ===== 网络参数 groupbox ===== */
	create_groupbox(L"网络参数", gx, 130, gw, 50);
	create_label(L"新 IP:", gx + 12, 154, 44, 14);
	g_cfg.hNip = create_edit(gx + 60, 150, 140, 22, IDC_CFG_NIP1, 0);
	create_button(L"应用", gx + gw - 96, 150, 80, 24, IDC_CFG_NIP_APPLY);


	/* ===== Modbus 参数 groupbox ===== */
	create_groupbox(L"Modbus 参数", gx, 196, gw, 50);
	create_label(L"从机地址:", gx + 12, 220, 64, 14);
	g_cfg.hMbSlave = create_edit(gx + 76, 216, 50, 22, IDC_CFG_MB_SLAVE, ES_NUMBER);
	create_label(L"波特率:", gx + 140, 220, 48, 14);
	g_cfg.hMbBaud = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL | WS_TABSTOP,
		gx + 188, 216, 100, 220, hWnd, (HMENU)(INT_PTR)IDC_CFG_MB_BAUD, g_hInst, NULL);
	SendMessageW(g_cfg.hMbBaud, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	for (int i = 0; i < BAUD_COUNT; i++) {
		wchar_t buf[16];
		swprintf(buf, 16, L"%d", g_bauds[i]);
		SendMessageW(g_cfg.hMbBaud, CB_ADDSTRING, 0, (LPARAM)buf);
	}
	SendMessageW(g_cfg.hMbBaud, CB_SETCURSEL, 1, 0); /* 默认 9600 */
	create_button(L"应用", gx + gw - 180, 216, 80, 24, IDC_CFG_MB_APPLY);
	create_button(L"读取", gx + gw - 96, 216, 80, 24, IDC_CFG_MB_READ);

	/* ===== 时间设置 groupbox ===== */
	create_groupbox(L"时间设置", gx, 262, gw, 50);
	create_label(L"本机时间:", gx + 12, 286, 64, 14);
	g_cfg.hTime = create_label(L"--", gx + 80, 286, 220, 14);
	create_button(L"应用本机时间", gx + gw - 180, 282, 168, 24, IDC_CFG_TIME_APPLY);

	/* ===== 出厂重置 (右对齐) ===== */
	create_button(L"出厂重置", gx + gw - 96, 326, 80, 24, IDC_CFG_FACTORY);

	/* ===== 操作日志 groupbox + 多行只读 EDIT ===== */
	create_groupbox(L"操作日志", gx, 366, gw, 396);
	g_cfg.hLog = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY |
		ES_AUTOVSCROLL | WS_VSCROLL,
		gx + 12, 386, gw - 24, 368,
		hWnd, (HMENU)(INT_PTR)IDC_CFG_LOG, g_hInst, NULL);
	SendMessageW(g_cfg.hLog, WM_SETFONT, (WPARAM)g_hFont, TRUE);
}

/* ===== WM_COMMAND: 按钮处理 ===== */

/* 设备下拉选择变更: 发现回复格式为 "a.b.c.d", 直接解析 IP 自动填入. */
static void on_devlist_changed(void)
{
	int sel = (int)SendMessageW(g_cfg.hDevList, CB_GETCURSEL, 0, 0);
	if (sel < 0) return;
	wchar_t wentry[64] = {0};
	SendMessageW(g_cfg.hDevList, CB_GETLBTEXT, sel, (LPARAM)wentry);
	int ip[4];
	if (swscanf(wentry, L"%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]) == 4) {
		wchar_t buf[32];
		swprintf(buf, 32, L"%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
		SetWindowTextW(g_cfg.hIp, buf);
		log_append(L"已从设备列表回填目标 IP");
	}
}

/* 发现设备: 调 GET_IP 广播, 拆分结果填下拉 (每行一个 "a.b.c.d"). */
static void on_discover(void)
{
	char buf[2048];
	int cnt = 0;
	SendMessageW(g_cfg.hDevList, CB_RESETCONTENT, 0, 0);
	log_append(L"正在发现设备...");
	if (UdpManager_Discover(g_cfg.udp, buf, sizeof(buf), &cnt)) {
		char *p = strtok(buf, "\n");
		while (p) {
			wchar_t w[160];
			MultiByteToWideChar(CP_UTF8, 0, p, -1, w, 160);
			SendMessageW(g_cfg.hDevList, CB_ADDSTRING, 0, (LPARAM)w);
			p = strtok(NULL, "\n");
		}
		if (cnt > 0) SendMessageW(g_cfg.hDevList, CB_SETCURSEL, 0, 0);
		wchar_t m[64];
		swprintf(m, 64, L"发现 %d 台设备", cnt);
		log_append(m);
	} else {
		log_append(L"未发现设备");
	}
}

/* 应用 IP (SET_IP 0x10): 目标 IP + 新 IP, 按结果分支. */
static void on_apply_ip(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	uint8_t nip[4];
	if (!read_ip_edit(g_cfg.hNip, nip)) {
		MessageBoxW(g_cfg.hSelf,
		            L"新 IP 格式错误, 请输入 a.b.c.d (每段 0-255)",
		            L"输入错误", MB_ICONERROR);
		return;
	}
	/* 目标 IP 也要校验 (传输层会用到). 单框输入可能为空或格式错. */
	uint8_t tip[4];
	if (!read_ip_edit(g_cfg.hIp, tip)) {
		MessageBoxW(g_cfg.hSelf,
		            L"目标设备 IP 格式错误, 请输入 a.b.c.d",
		            L"输入错误", MB_ICONERROR);
		return;
	}
	/* read_ip_edit 已校验过, 重新拼合法串覆盖 current_target_ip 的原始文本,
	 * 避免传输层收到 "192.168.1." 这类尾部缺失但仍能 WideCharToMultiByte 的串. */
	snprintf(ip, sizeof(ip), "%u.%u.%u.%u", tip[0], tip[1], tip[2], tip[3]);
	uint8_t ok = 0;
	if (UdpManager_SetIp(g_cfg.udp, ip, nip, &ok)) {
		if (ok) {
			log_append(L"SET_IP 成功 (已持久化, 需手动重启生效)");
			MessageBoxW(g_cfg.hSelf,
			            L"IP 已设置并持久化\n请手动重启设备 (重启按钮或重新上电) 使新 IP 生效",
			            L"成功", MB_ICONINFORMATION);
		} else {
			log_append(L"SET_IP 被拒绝 (IP 末段 0/255 或首段 0/127/224-255)");
			MessageBoxW(g_cfg.hSelf, L"设备拒绝该 IP", L"警告",
			            MB_ICONWARNING);
		}
	} else {
		show_transport_error(L"SET_IP");
	}
}

/* 查询版本 (0x04). */
static void on_get_version(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	char ver[64] = {0};
	if (UdpManager_GetVersion(g_cfg.udp, ip, ver, sizeof(ver))) {
		wchar_t wver[64];
		MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 64);
		SetWindowTextW(g_cfg.hVersion, wver);
		wchar_t m[128];
		swprintf(m, 128, L"GET_VERSION 成功: %hs", ver);
		log_append(m);
	} else {
		SetWindowTextW(g_cfg.hVersion, L"(查询失败)");
		show_transport_error(L"GET_VERSION");
	}
}

/* 重启 (0x05). 设备收到即重启, 回复不可靠. */
static void on_reboot(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	if (MessageBoxW(g_cfg.hSelf, L"确认重启目标设备?", L"确认",
	                MB_YESNO | MB_ICONQUESTION) != IDYES) {
		return;
	}
	UdpManager_Reboot(g_cfg.udp, ip);
	log_append(L"REBOOT 已发送");
	MessageBoxW(g_cfg.hSelf, L"重启命令已发送", L"提示", MB_ICONINFORMATION);
}

/* 应用 Modbus 参数 (SET_MODBUS 0x12). */
static void on_apply_modbus(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	wchar_t ws[8];
	GetWindowTextW(g_cfg.hMbSlave, ws, 8);
	int slave = _wtoi(ws);
	if (slave < 1 || slave > 247) {
		MessageBoxW(g_cfg.hSelf, L"从机地址应在 1-247", L"输入错误",
		            MB_ICONERROR);
		return;
	}
	int bsel = (int)SendMessageW(g_cfg.hMbBaud, CB_GETCURSEL, 0, 0);
	uint16_t baud = (bsel >= 0 && bsel < BAUD_COUNT) ? (uint16_t)g_bauds[bsel] : 9600;
	uint8_t ok = 0;
	if (UdpManager_SetModbus(g_cfg.udp, ip, (uint8_t)slave, baud, &ok)) {
		if (ok) {
			wchar_t m[64];
			swprintf(m, 64, L"SET_MODBUS 成功 (slave=%d, baud=%u)", slave, baud);
			log_append(m);
			MessageBoxW(g_cfg.hSelf, L"Modbus 参数已应用", L"成功",
			            MB_ICONINFORMATION);
		} else {
			log_append(L"SET_MODBUS 被拒绝");
			MessageBoxW(g_cfg.hSelf, L"设备拒绝该参数", L"警告",
			            MB_ICONWARNING);
		}
	} else {
		show_transport_error(L"SET_MODBUS");
	}
}

/* 读取 Modbus 参数 (GET_MODBUS 0x13). */
static void on_read_modbus(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	uint8_t slave = 0;
	uint16_t baud = 0;
	if (UdpManager_GetModbus(g_cfg.udp, ip, &slave, &baud)) {
		wchar_t buf[16];
		swprintf(buf, 16, L"%u", slave);
		SetWindowTextW(g_cfg.hMbSlave, buf);
		select_baud(baud);
		log_append(L"GET_MODBUS 成功");
	} else {
		show_transport_error(L"GET_MODBUS");
	}
}

/* 应用本机时间到设备 (SET_TIME 0x14). 取上位机当前 Unix 时间戳发给设备. */
static void on_apply_time(void)
{
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	/* time(NULL) 返回 time_t (Unix 秒). Win32 time_t 是 64-bit, 截断到 32 位
	 * (设备端 uint32 接收, 2038 年前均有效). */
	uint32_t ts = (uint32_t)time(NULL);
	uint8_t ok = 0;
	if (UdpManager_SetTime(g_cfg.udp, ip, ts, &ok)) {
		if (ok) {
			wchar_t m[96];
			swprintf(m, 96, L"SET_TIME 成功 (unix=%u)", ts);
			log_append(m);
			MessageBoxW(g_cfg.hSelf, L"时间已同步到设备 RTC", L"成功",
			            MB_ICONINFORMATION);
		} else {
			log_append(L"SET_TIME 被拒绝 (时间戳越界)");
			MessageBoxW(g_cfg.hSelf, L"设备拒绝该时间戳", L"警告",
			            MB_ICONWARNING);
		}
	} else {
		show_transport_error(L"SET_TIME");
	}
}

/* 出厂重置 (0x19): 需 MB_YESNO 警告确认 (擦除存储分区). */
static void on_factory_reset(void)
{
	if (MessageBoxW(g_cfg.hSelf,
	                L"确认出厂重置?\n将擦除所有参数 (IP/Modbus) 并重启设备",
	                L"危险操作", MB_YESNO | MB_ICONWARNING) != IDYES) {
		return;
	}
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	uint8_t ok = 0;
	if (UdpManager_FactoryReset(g_cfg.udp, ip, &ok)) {
		if (ok) {
			log_append(L"出厂重置已执行, 设备将重启");
			MessageBoxW(g_cfg.hSelf, L"出厂重置已执行, 设备将重启", L"完成",
			            MB_ICONINFORMATION);
		} else {
			log_append(L"出厂重置被设备拒绝");
			MessageBoxW(g_cfg.hSelf, L"设备拒绝出厂重置", L"警告",
			            MB_ICONWARNING);
		}
	} else {
		show_transport_error(L"FACTORY_RESET");
	}
}

/* 按 UDP 连接状态刷新各查询/设置按钮可用性.
 * 未连接时: 查询版本/重启/网络应用/Modbus应用+读取/时间应用/出厂重置 全禁用.
 * 已连接时: 全部恢复可用. */
static void update_buttons(void)
{
	BOOL en = g_cfg.udp_connected ? TRUE : FALSE;
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_GETVER), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_REBOOT), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_NIP_APPLY), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_MB_APPLY), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_MB_READ), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_TIME_APPLY), en);
	EnableWindow(GetDlgItem(g_cfg.hSelf, IDC_CFG_FACTORY), en);
}

/* UDP 连接/断开 toggle. 连接 = 用目标 IP 调 GET_VERSION 握手, 成功即已连接. */
static void on_connect(void)
{
	if (g_cfg.udp_connected) {
		/* 断开 */
		g_cfg.udp_connected = false;
		SetWindowTextW(g_cfg.hConn, L"连接");
		SetWindowTextW(g_cfg.hVersion, L"(未查询)");
		log_append(L"已断开连接");
		update_buttons();
		return;
	}
	char ip[32];
	current_target_ip(ip, sizeof(ip));
	char ver[64] = {0};
	if (!UdpManager_GetVersion(g_cfg.udp, ip, ver, sizeof(ver))) {
		show_transport_error(L"连接设备 (GET_VERSION)");
		return;
	}
	g_cfg.udp_connected = true;
	SetWindowTextW(g_cfg.hConn, L"断开");
	wchar_t wver[64];
	MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 64);
	SetWindowTextW(g_cfg.hVersion, wver);
	wchar_t m[128];
	swprintf(m, 128, L"连接成功: %hs", ver);
	log_append(m);
	update_buttons();
}

/* WM_COMMAND 总分发. */
static void on_command(WPARAM wParam, LPARAM lParam)
{
	WORD id = LOWORD(wParam);
	WORD code = HIWORD(wParam);

	/* 设备下拉选择变更: 自动回填目标 IP */
	if (id == IDC_CFG_DEVLIST && code == CBN_SELCHANGE) {
		on_devlist_changed();
		return;
	}
	if (code != BN_CLICKED) return;
	(void)lParam;

	switch (id) {
	case IDC_CFG_DISCOVER_BTN: on_discover(); break;
	case IDC_CFG_CONNECT:      on_connect(); break;
	case IDC_CFG_GETVER:       on_get_version(); break;
	case IDC_CFG_REBOOT:       on_reboot(); break;
	case IDC_CFG_NIP_APPLY:    on_apply_ip(); break;
	case IDC_CFG_MB_APPLY:     on_apply_modbus(); break;
	case IDC_CFG_MB_READ:      on_read_modbus(); break;
	case IDC_CFG_TIME_APPLY:   on_apply_time(); break;
	case IDC_CFG_FACTORY:      on_factory_reset(); break;
	}
}

/* ===== 窗口过程 ===== */

static LRESULT CALLBACK cfg_wndproc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		g_hInst = ((LPCREATESTRUCT)lParam)->hInstance;
		create_controls(hWnd);
		g_cfg.udp = UdpManager_Create();
		g_cfg.udp_connected = false;
		if (!g_cfg.udp) {
			log_append(L"错误: UdpManager 创建失败");
		} else {
			log_append(L"就绪. 请先发现设备并点击\"连接\"");
		}
		/* 初始未连接: 禁用所有查询/设置按钮 */
		update_buttons();
		/* 启动 1s 定时器刷新本机时间显示 */
		update_time_display();
		SetTimer(hWnd, IDC_CFG_TIMER, 1000, NULL);
		return 0;
	case WM_TIMER:
		if (wParam == IDC_CFG_TIMER) {
			update_time_display();
		}
		return 0;
	case WM_COMMAND:
		on_command(wParam, lParam);
		return 0;
	case WM_SIZE:
		/* 控件保持固定位置 (与 handler-receiver 一致). */
		return 0;
	case WM_CTLCOLORDLG:
		/* 对话框底色 BTNFACE */
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	case WM_CTLCOLORSTATIC: {
		HDC hdc = (HDC)wParam;
		HWND hCtrl = (HWND)lParam;
		/* 只读多行日志 EDIT 不能用 TRANSPARENT (会残留旧文字), 用不透明背景.
		 * 注: 只读 EDIT 也走 WM_CTLCOLORSTATIC. */
		if (GetWindowLongPtrW(hCtrl, GWL_STYLE) & ES_READONLY) {
			SetBkMode(hdc, OPAQUE);
			SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
			SetBkColor(hdc, GetSysColor(COLOR_WINDOW));
			return (LRESULT)GetSysColorBrush(COLOR_WINDOW);
		}
		/* 其他 STATIC: 透明 + BTNFACE 底色 (视觉与父窗口一致). */
		SetBkMode(hdc, TRANSPARENT);
		SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	}
	case WM_DESTROY:
		if (g_cfg.udp) {
			UdpManager_Destroy(g_cfg.udp);
			g_cfg.udp = NULL;
		}
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

/* ===== 公共 API ===== */

HWND ConfigTab_Create(HWND hParent, HINSTANCE hInst)
{
	g_hInst = hInst;

	if (!g_classRegistered) {
		WNDCLASSW wc = {0};
		wc.lpfnWndProc = cfg_wndproc;
		wc.hInstance = hInst;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
		wc.lpszClassName = CONFIG_TAB_CLASS;
		RegisterClassW(&wc);
		g_classRegistered = TRUE;
	}

	HWND h = CreateWindowExW(0, CONFIG_TAB_CLASS, L"",
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
		0, 0, 700, 500, hParent, NULL, hInst, NULL);
	return h;
}
