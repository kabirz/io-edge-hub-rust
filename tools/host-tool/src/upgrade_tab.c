/* io-edge-hub 上位机 - Tab2 "固件升级"
 *
 * 程序化创建全部控件, 单一 g_upg 静态结构持有所有控件 HWND + UdpManager + CanManager
 * 实例 + 当前固件 image 缓冲. WM_CREATE 创建控件 + 两个 manager; WM_DESTROY 销毁.
 *
 * 通道: 单选 UDP / CAN. 切换时 ShowWindow 显示对应子区域 (IP / PCAN 设备行).
 *
 * 流程:
 *   1. 浏览 app.dfu.bin (裸镜像 + 64B ed25519 签名) → 全量入堆 →
 *      fw_image_validate_header 拒非法长度/旧 MCUboot 镜像; keyhash 为
 *      编译期常量 SHA-256(公钥) (与 proto::fw_upg::FW_KEYHASH 同源).
 *   2. 开始升级 → 校验输入 → 禁用开始/启用取消 → CreateThread 起 UDP 或 CAN worker.
 *   3. worker 全程只通过 PostMessage (WM_APP_UPG_PROGRESS/LOG/DONE) 与 UI 通信.
 *   4. UI 收 DONE → 恢复按钮 + MessageBox 结果 (升级成功后重启, 设备侧
 *      embassy-boot 换机 ~30s, 新镜像跑通 main 自动确认, 否则自动回滚).
 *
 * 布局 (主窗口 tab 显示区约 712x504):
 *   - 通道 groupbox: 通道单选 + (UDP: 目标IP+测试 / CAN: PCAN+波特率+连接)
 *   - 固件文件 groupbox: 路径 + 浏览 + fileinfo
 *   - 升级控制 groupbox: 开始 + 取消 + 进度条 + 状态文字
 *   - 操作日志 groupbox: 多行只读 EDIT (带时间戳)
 */
#include "udp_manager.h"   /* 须先于 windows.h 拉 winsock2.h (避免 winsock1 冲突) */
#include "can_manager.h"
#include "pcan_loader.h"   /* PCAN_BAUD_* 波特率 BTR 寄存器值 */
#include "fw_image.h"
#include "upgrade_tab.h"
#include "resource.h"
#include <commctrl.h>
#include <commdlg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <wchar.h>

/* ===== 静态状态: 所有控件 HWND + manager + image 缓冲 + 线程状态 ===== */
typedef struct {
	HWND hSelf;
	/* 通道单选 */
	HWND hChanUdp, hChanCan;
	/* 连接/断开 (UDP/CAN 通用) */
	HWND hConn;
	/* UDP 行: 目标 IP (单框) + 标签 */
	HWND hUdpLbl, hIp;
	/* CAN 行: 设备下拉 + 波特率下拉 + 刷新按钮 + 救援模式复选框 */
	HWND hCanLbl1, hCanDev, hCanLbl2, hCanBaud, hCanRefresh, hCanBoot;
	/* 版本信息行: label + 查询按钮 */
	HWND hVerLbl, hVersion, hGetVer;
	/* 文件 */
	HWND hFile, hBrowse, hFileInfo;
	/* 升级控制 */
	HWND hStart, hReboot, hProgress, hStatus, hLog;
	/* manager */
	UdpManager *udp;
	CanManager *can;
	bool udp_connected;   /* UDP 连接状态 (连接后启用查询/升级/重启) */
	bool can_connected;
	bool can_detected;
	int  can_channel;
	/* image 缓冲 (浏览时加载, 升级时消费, 切换文件/退出时释放) */
	uint8_t *img;
	uint32_t img_size;
	uint8_t  keyhash[32];
	bool has_keyhash;
	/* worker 输入 (UI 线程在 CreateThread 前缓存, worker 只读) */
	char  cur_ip[32];
	bool  cur_permanent;
	bool  cur_boot;       /* true=MCUboot 紧急救援模式 (CAN) */
	bool  cur_can;        /* true=本次升级走 CAN 通道 (DONE 弹窗按此分发重启) */
	bool  wait_reboot;    /* true=升级后已发重启命令, 定时器到点确认设备上线 */
	bool  pending_reboot; /* true=升级成功但尚未重启 (待用户/弹窗触发重启) */
	char  old_ver[80];    /* 升级前的设备版本 (重启确认后与新版拼 "旧 → 新") */
	/* 取消标志 + 线程句柄 */
	volatile LONG cancel;
	HANDLE thread;
} UpgradeTab;

static UpgradeTab g_upg;
static HFONT g_hFont = NULL;
static const wchar_t *UPGRADE_TAB_CLASS = L"ioEdgeHubUpgradeTabCls";
static BOOL g_classRegistered = FALSE;

/* 升级后重启确认定时器: 重启命令发出后到点查一次设备版本,
 * 收尾 "重启中" 状态 (embassy-boot 换机实测约 30s + 启动, 给 60s 余量) */
#define UPG_REBOOT_TIMER_ID   1
#define UPG_REBOOT_WAIT_MS    60000

/* 救援模式升级确认定时器: CONFIRM 后 MCUboot 直接验证并启动新固件
 * (无 swap, 几秒内上线), 5s 后查一次版本刷新显示 */
#define UPG_BOOT_OK_TIMER_ID  2
#define UPG_BOOT_OK_WAIT_MS   5000

/* PCAN 波特率 (与 pcan_loader.h BTR 寄存器值对应, can_manager 直传) */
static const struct { const wchar_t *label; uint32_t btr; } g_bauds[] = {
	{ L"250 kbps (默认)", PCAN_BAUD_250K },
	{ L"500 kbps",        PCAN_BAUD_500K },
	{ L"1000 kbps",       PCAN_BAUD_1M   },
	{ L"125 kbps",        PCAN_BAUD_125K },
	{ L"100 kbps",        PCAN_BAUD_100K },
	{ L"50 kbps",         PCAN_BAUD_50K  },
};
#define BAUD_COUNT (int)(sizeof(g_bauds) / sizeof(g_bauds[0]))

/* ===== 控件创建辅助 ===== */

static HWND create_label(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"STATIC", text,
		WS_CHILD | WS_VISIBLE, x, y, w, h,
		g_upg.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_edit(int x, int y, int w, int h, int id, DWORD extra)
{
	HWND hw = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | extra,
		x, y, w, h, g_upg.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_button(const wchar_t *text, int x, int y, int w, int h, int id)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		x, y, w, h, g_upg.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_groupbox(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
		x, y, w, h, g_upg.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

/* ===== 业务辅助 ===== */

/* 当前选中的通道: 0=UDP, 1=CAN */
static int current_channel(void)
{
	return (SendMessageW(g_upg.hChanCan, BM_GETCHECK, 0, 0) == BST_CHECKED) ? 1 : 0;
}

/* 按当前通道连接状态刷新按钮可用性.
 * - 未连接: 查询版本/开始升级/重启 全禁用 (连接按钮始终可用)
 * - 已连接: 查询版本/重启 恢复可用; 开始升级 还需已选有效固件文件. */
static void update_button_state(void)
{
	int can = current_channel();
	bool conn = can ? g_upg.can_connected : g_upg.udp_connected;
	EnableWindow(g_upg.hConn, TRUE);
	EnableWindow(g_upg.hGetVer, conn);
	EnableWindow(g_upg.hReboot, conn);
	EnableWindow(g_upg.hStart, conn && g_upg.img ? TRUE : FALSE);
}

/* 切换通道: 显示/隐藏 UDP 与 CAN 子区域. */
static void apply_channel_visibility(void)
{
	int can = current_channel();
	/* UDP 行 */
	ShowWindow(g_upg.hUdpLbl, can ? SW_HIDE : SW_SHOW);
	ShowWindow(g_upg.hIp, can ? SW_HIDE : SW_SHOW);
	/* CAN 行 */
	ShowWindow(g_upg.hCanLbl1,   can ? SW_SHOW : SW_HIDE);
	ShowWindow(g_upg.hCanDev,    can ? SW_SHOW : SW_HIDE);
	ShowWindow(g_upg.hCanLbl2,   can ? SW_SHOW : SW_HIDE);
	ShowWindow(g_upg.hCanBaud,   can ? SW_SHOW : SW_HIDE);
	ShowWindow(g_upg.hCanRefresh, can ? SW_SHOW : SW_HIDE);
	ShowWindow(g_upg.hCanBoot,   can ? SW_SHOW : SW_HIDE);
	/* 连接按钮文字随当前通道连接状态 */
	SetWindowTextW(g_upg.hConn,
		(can ? g_upg.can_connected : g_upg.udp_connected) ? L"断开" : L"连接");
	update_button_state();
}

/* UI 线程: 设置 fileinfo 静态文本 (默认黑色). */
static void set_fileinfo(const wchar_t *msg)
{
	SetWindowTextW(g_upg.hFileInfo, msg);
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

/* 日志框追加一行 (UI 线程: WM_APP_UPG_LOG 处理时调用). msg 已是用户字符串. */
static void log_append_ptr(const wchar_t *msg)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	wchar_t line[600];
	swprintf(line, 600, L"[%02d:%02d:%02d] %ls\r\n",
	         st.wHour, st.wMinute, st.wSecond, msg);
	int len = GetWindowTextLengthW(g_upg.hLog);
	SendMessageW(g_upg.hLog, EM_SETSEL, len, len);
	SendMessageW(g_upg.hLog, EM_REPLACESEL, 0, (LPARAM)line);
}

/* worker → UI: 投递一条日志 (堆字符串, UI free). */
static void post_log(const wchar_t *msg)
{
	wchar_t *dup = _wcsdup(msg);
	if (dup) {
		PostMessageW(g_upg.hSelf, WM_APP_UPG_LOG, 0, (LPARAM)dup);
	}
}

/* worker → UI: 投递完成. success=1 成功 / 0 失败. */
static void post_done(int success)
{
	PostMessageW(g_upg.hSelf, WM_APP_UPG_DONE, (WPARAM)success, 0);
}

/* worker → UI: 投递进度 (percent 0-100, stage 0=未用/1=发送数据/2=等待重启). */
static void post_progress(int percent, int stage)
{
	PostMessageW(g_upg.hSelf, WM_APP_UPG_PROGRESS, (WPARAM)percent, (LPARAM)stage);
}

/* 释放已加载 image 缓冲 (浏览新文件 / 退出时). */
static void free_image(void)
{
	if (g_upg.img) {
		free(g_upg.img);
		g_upg.img = NULL;
	}
	g_upg.img_size = 0;
	g_upg.has_keyhash = false;
}

/* ===== 浏览 + 载荷校验 ===== */

static void on_browse(void)
{
	wchar_t path[MAX_PATH] = {0};
	OPENFILENAMEW ofn;
	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = g_upg.hSelf;
	ofn.lpstrFilter = L"固件镜像 (*.bin)\0*.bin\0所有文件\0*.*\0";
	ofn.lpstrFile = path;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
	if (!GetOpenFileNameW(&ofn)) return;
	SetWindowTextW(g_upg.hFile, path);

	/* 释放旧缓冲 */
	free_image();

	/* 全量读入堆 */
	HANDLE hf = CreateFileW(path, GENERIC_READ, FILE_SHARE_READ, NULL,
	                        OPEN_EXISTING, 0, NULL);
	if (hf == INVALID_HANDLE_VALUE) {
		set_fileinfo(L"打开文件失败");
		EnableWindow(g_upg.hStart, FALSE);
		return;
	}
	DWORD size = GetFileSize(hf, NULL);
	if (size == INVALID_FILE_SIZE || size == 0) {
		CloseHandle(hf);
		set_fileinfo(L"文件为空或读取大小失败");
		EnableWindow(g_upg.hStart, FALSE);
		return;
	}
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf) {
		CloseHandle(hf);
		set_fileinfo(L"内存不足");
		EnableWindow(g_upg.hStart, FALSE);
		return;
	}
	DWORD rd = 0;
	BOOL ok = ReadFile(hf, buf, size, &rd, NULL);
	CloseHandle(hf);
	if (!ok || rd != size) {
		free(buf);
		set_fileinfo(L"读取文件不完整");
		EnableWindow(g_upg.hStart, FALSE);
		return;
	}

	/* 载荷校验: 裸镜像 + 尾部 64B ed25519 签名, 长度须在 (64B, 512K] */
	if (!fw_image_validate_header(buf, size)) {
		if (fw_image_is_mcuboot(buf, size)) {
			set_fileinfo(L"旧 MCUboot 镜像, 与本固件不匹配 (请用 app.dfu.bin)");
		} else {
			set_fileinfo(L"非固件载荷 (长度非法, 应为 app.dfu.bin)");
		}
		free(buf);
		EnableWindow(g_upg.hStart, FALSE);
		return;
	}

	/* keyhash 为编译期常量 (SHA-256 of ed25519 公钥), 与镜像无关 */
	g_upg.img = buf;
	g_upg.img_size = size;
	memcpy(g_upg.keyhash, fw_image_keyhash(), 32);
	g_upg.has_keyhash = true;

	wchar_t info[64];
	swprintf(info, 64, L"%u 字节 (含 64B 签名)", (unsigned)size);
	set_fileinfo(info);
	update_button_state();
}

/* ===== 版本查询 (UI 线程) ===== */

/* 按当前通道查询设备版本并刷新版本 label.
 * UDP: GET_VERSION (0x04) 到目标 IP; CAN: CanManager_GetVersion (0x101 cmd=2).
 * 失败时 label 显示 "(查询失败)" 并把详细错误写入日志框. */
static void on_query_version(void)
{
	char ver[80] = {0};
	bool ok = false;
	int can = current_channel();

	if (!can) {
		/* UDP: 取目标 IP (单框) */
		char ip[64] = {0};
		uint8_t ip4[4];
		if (!read_ip_edit(g_upg.hIp, ip4)) {
			SetWindowTextW(g_upg.hVersion, L"(请先填目标 IP)");
			return;
		}
		snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ip4[0], ip4[1], ip4[2], ip4[3]);
		ok = UdpManager_GetVersion(g_upg.udp, ip, ver, sizeof(ver));
	} else {
		/* CAN: 必须已连接 */
		if (!g_upg.can_connected) {
			SetWindowTextW(g_upg.hVersion, L"(请先连接 PCAN)");
			return;
		}
		ok = CanManager_GetVersion(g_upg.can, ver, sizeof(ver));
	}

	if (ok) {
		wchar_t wver[160] = {0};
		MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 160);
		SetWindowTextW(g_upg.hVersion, wver);
		/* 缓存最近一次查询到的版本: 升级前作为 "旧版本" 快照 */
		snprintf(g_upg.old_ver, sizeof(g_upg.old_ver), "%s", ver);
		log_append_ptr(L"版本查询成功");
	} else {
		SetWindowTextW(g_upg.hVersion, L"(查询失败)");
		wchar_t m[200];
		const char *e = can ? CanManager_GetLastError(g_upg.can)
		                    : UdpManager_GetLastError(g_upg.udp);
		wchar_t werr[160];
		MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 160);
		swprintf(m, 200, L"版本查询失败: %ls", werr);
		log_append_ptr(m);
	}
}

/* ===== CAN 连接 (UI 线程) ===== */

/* 刷新 PCAN 设备下拉: 枚举所有 PCAN-USB 通道 (名称格式与 handler-receiver 一致).
 * 更新 can_detected 标志并刷新按钮可用性. */
static void refresh_can_device(void)
{
	SendMessageW(g_upg.hCanDev, CB_RESETCONTENT, 0, 0);
	char names[16][32];
	int channels[16];
	int cnt = CanManager_DetectDevices(g_upg.can, names, channels, 16);
	if (cnt > 0) {
		for (int i = 0; i < cnt; i++) {
			wchar_t wname[32];
			MultiByteToWideChar(CP_ACP, 0, names[i], -1, wname, 32);
			SendMessageW(g_upg.hCanDev, CB_ADDSTRING, 0, (LPARAM)wname);
			SendMessageW(g_upg.hCanDev, CB_SETITEMDATA, i, (LPARAM)channels[i]);
		}
		SendMessageW(g_upg.hCanDev, CB_SETCURSEL, 0, 0);
		g_upg.can_detected = true;
		log_append_ptr(L"已刷新: 检测到 PCAN-USB 设备");
	} else {
		SendMessageW(g_upg.hCanDev, CB_ADDSTRING, 0, (LPARAM)L"(未检测到 PCAN-USB)");
		SendMessageW(g_upg.hCanDev, CB_SETCURSEL, 0, 0);
		g_upg.can_detected = false;
		log_append_ptr(L"已刷新: 未检测到 PCAN-USB 设备");
	}
	update_button_state();
}

/* 连接/断开 PCAN. 切换按钮文字 + 状态. */
static void on_can_connect(void)
{
	if (g_upg.can_connected) {
		CanManager_Disconnect(g_upg.can);
		g_upg.can_connected = false;
		g_upg.can_channel = -1;
		SetWindowTextW(g_upg.hConn, L"连接");
		EnableWindow(g_upg.hCanDev, TRUE);
		EnableWindow(g_upg.hCanBaud, TRUE);
		log_append_ptr(L"PCAN 已断开");
		update_button_state();
		return;
	}
	int sel = (int)SendMessageW(g_upg.hCanDev, CB_GETCURSEL, 0, 0);
	if (sel < 0) {
		MessageBoxW(g_upg.hSelf, L"请先点刷新检测 PCAN 设备", L"提示",
		            MB_ICONWARNING);
		return;
	}
	int channel = (int)SendMessageW(g_upg.hCanDev, CB_GETITEMDATA, sel, 0);
	int bsel = (int)SendMessageW(g_upg.hCanBaud, CB_GETCURSEL, 0, 0);
	uint32_t bitrate = (bsel >= 0 && bsel < BAUD_COUNT) ? g_bauds[bsel].btr
	                                                    : PCAN_BAUD_250K;
	if (!CanManager_Connect(g_upg.can, channel, bitrate)) {
		wchar_t m[256];
		const char *e = CanManager_GetLastError(g_upg.can);
		wchar_t werr[192];
		MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
		swprintf(m, 256, L"PCAN 连接失败: %ls", werr);
		MessageBoxW(g_upg.hSelf, m, L"连接失败", MB_ICONERROR);
		return;
	}
	g_upg.can_connected = true;
	g_upg.can_channel = channel;
	SetWindowTextW(g_upg.hConn, L"断开");
	log_append_ptr(L"PCAN 已连接, 查询设备版本...");
	EnableWindow(g_upg.hCanDev, FALSE);
	EnableWindow(g_upg.hCanBaud, FALSE);
	/* 连接成功后启用查询版本/重启, 并自动查询一次设备版本 */
	update_button_state();
	on_query_version();
}

/* ===== UDP 连接 (UI 线程) ===== */

/* UDP: 用目标 IP 调 GET_VERSION 握手, 成功即已连接 (与 tab1 连接语义一致). */
static void on_udp_connect(void)
{
	if (g_upg.udp_connected) {
		/* 断开 */
		g_upg.udp_connected = false;
		SetWindowTextW(g_upg.hConn, L"连接");
		SetWindowTextW(g_upg.hVersion, L"(未查询)");
		log_append_ptr(L"已断开连接");
		update_button_state();
		return;
	}
	uint8_t ip4[4];
	if (!read_ip_edit(g_upg.hIp, ip4)) {
		MessageBoxW(g_upg.hSelf, L"请填写目标设备 IP (如 192.168.12.101)", L"提示",
		            MB_ICONWARNING);
		return;
	}
	char ip[64];
	snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ip4[0], ip4[1], ip4[2], ip4[3]);
	char ver[80] = {0};
	if (!UdpManager_GetVersion(g_upg.udp, ip, ver, sizeof(ver))) {
		wchar_t m[256];
		const char *e = UdpManager_GetLastError(g_upg.udp);
		wchar_t werr[192];
		MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
		swprintf(m, 256, L"连接设备失败: %ls", werr);
		MessageBoxW(g_upg.hSelf, m, L"连接失败", MB_ICONERROR);
		return;
	}
	g_upg.udp_connected = true;
	SetWindowTextW(g_upg.hConn, L"断开");
	wchar_t wver[160];
	MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 160);
	SetWindowTextW(g_upg.hVersion, wver);
	log_append_ptr(L"连接成功, 已获取设备版本");
	update_button_state();
}

/* 连接按钮: 按当前通道分发到 UDP / CAN. */
static void on_connect(void)
{
	if (current_channel()) {
		on_can_connect();
	} else {
		on_udp_connect();
	}
}

/* ===== UDP worker 线程 =====
 * 全程只读 g_upg 缓存值 (cur_ip/cur_permanent/img), 仅通过 PostMessage 与 UI 交互. */

/* FW_DATA_V2 流式回调: 进度 (0-90% 数据发送阶段) + 取消 */
static void v2_progress(uint32_t off, void *ud)
{
	(void)ud;
	post_progress((int)((uint64_t)off * 90 / g_upg.img_size), 1);
}
static bool v2_cancel(void *ud)
{
	(void)ud;
	return InterlockedCompareExchange(&g_upg.cancel, 0, 0) != 0;
}

static DWORD WINAPI udp_upgrade_thread(LPVOID arg)
{
	(void)arg;
	post_log(L"UDP 升级开始");

	uint8_t status = 0;
	uint16_t v2_chunk = 0;
	uint32_t sz = g_upg.img_size;
	const char *ip = g_upg.cur_ip;

	/* keyhash 优先设备自报 (UDP 0x15, 与升级同通道, 换钥匙零同步);
	 * 失败退回内置常量 (过渡固件轮换场景仍可用) */
	uint8_t kh_buf[32];
	const uint8_t *kh = fw_image_keyhash();
	if (UdpManager_GetKeyhash(g_upg.udp, ip, kh_buf)) {
		kh = kh_buf;
		post_log(L"keyhash: 设备自报 (UDP 0x15)");
	} else {
		post_log(L"keyhash: 内置常量 (设备未应答 0x15)");
	}

	if (!UdpManager_FwStart(g_upg.udp, ip, sz, kh, &status, &v2_chunk)) {
		post_log(L"FW_START 无响应 (设备未开机或 IP 错误)");
		post_done(0);
		return 0;
	}
	if (status == 2) {
		post_log(L"keyhash 不匹配, 设备拒绝升级");
		post_done(0);
		return 0;
	}
	if (status != 1) {
		post_log(L"FW_START 失败 (设备忙或存储不足)");
		post_done(0);
		return 0;
	}

	if (v2_chunk >= 512) {
		/* 设备为新固件: 优先 FW_DATA_V2 窗口流水线 (RTT 被流水线掩盖) */
		int chunk = (v2_chunk > 1400) ? 1400 : v2_chunk;
		wchar_t m[128];

		swprintf(m, 128, L"FW_START 成功, DATA_V2 窗口模式 (8 x %dB)", chunk);
		post_log(m);
		if (!UdpManager_FwDataV2Stream(g_upg.udp, ip, g_upg.img, sz, chunk,
		                               v2_progress, NULL, v2_cancel)) {
			const char *e = UdpManager_GetLastError(g_upg.udp);
			wchar_t werr[192];

			MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
			swprintf(m, 128, L"FW_DATA_V2 失败: %ls", werr);
			post_log(m);
			post_done(0);
			return 0;
		}
	} else {
		/* 停等 FW_DATA: 设备为老固件 (无 v2_chunk 协商字段), 自动回退 */
		uint32_t off = 0;
		const uint32_t CHUNK = 511;

		post_log(L"FW_START 成功 (设备旧固件无 V2, 停等模式)");
		while (off < sz) {
			/* 检查取消 (每个 chunk 一次) */
			if (InterlockedCompareExchange(&g_upg.cancel, 0, 0)) {
				post_log(L"用户取消升级");
				post_done(0);
				return 0;
			}
			uint32_t n = sz - off;
			if (n > CHUNK) n = CHUNK;
			uint32_t roff = 0;
			if (!UdpManager_FwData(g_upg.udp, ip, g_upg.img + off, (int)n, &roff)) {
				wchar_t m[128];
				swprintf(m, 128, L"FW_DATA 失败 (offset=%u)", off);
				post_log(m);
				post_done(0);
				return 0;
			}
			off += n;
			/* 进度 0-90%: 数据发送阶段 */
			post_progress((int)((uint64_t)off * 90 / sz), 1);
		}
	}

	/* CRC + FwEnd. test 固定 0=永久升级 (不允许测试模式). */
	uint16_t crc = UdpManager_CRC16_CCITT(g_upg.img, sz);
	uint8_t test = 0;
	uint8_t result = 0;
	if (!UdpManager_FwEnd(g_upg.udp, ip, test, crc, &result) || result != 1) {
		post_log(L"FW_END 失败 (CRC 校验或设备端 ed25519 验签未通过)");
		post_done(0);
		return 0;
	}

	post_progress(100, 2);
	post_log(L"UDP 升级完成, 请重启设备完成 embassy-boot 换机 (可在弹窗或本页『重启』按钮操作)");
	post_done(1);
	return 0;
}

/* ===== CAN worker 线程 ===== */

/* CAN 升级进度回调 (CanManager_FirmwareUpgrade 调用, 在 worker 线程上下文).
 * 函数名避开 can_manager.h 的 typedef can_progress_cb. */
static void can_progress_handler(int percent, void *user)
{
	(void)user;
	if (percent < 0) percent = 0;
	if (percent > 100) percent = 100;
	/* stage=1 发送数据 (CAN 升级中) */
	post_progress(percent, 1);
}

static DWORD WINAPI can_upgrade_thread(LPVOID arg)
{
	(void)arg;
	post_log(L"CAN 升级开始");

	/* keyhash 优先设备自报 (CAN 0x101 cmd=4, 与 UDP 0x15 / 网页 /api/info
	 * 对齐, 换钥匙零同步); 旧固件无此命令时退回 exe 旁 ed25519.keyhash
	 * 文件, 最后退回内置常量 (过渡固件轮换场景仍可用) */
	uint8_t kh_buf[32];
	const uint8_t *kh = fw_image_keyhash();
	if (CanManager_GetKeyhash(g_upg.can, kh_buf)) {
		kh = kh_buf;
		post_log(L"keyhash: 设备自报 (CAN 0x101 cmd=4)");
	} else if (fw_image_keyhash_load_file(kh_buf)) {
		kh = kh_buf;
		post_log(L"keyhash: ed25519.keyhash (exe 同目录)");
	} else {
		post_log(L"keyhash: 内置常量");
	}
	bool permanent = g_upg.cur_permanent; /* true=永久 */

	/* MCUboot 紧急救援模式: 尽力发重启命令, 若设备死机则等待用户手动断电重启,
	 * 收到 bootloader 探测帧后应答, 之后 keyhash/START/DATA/CONFIRM 流程与
	 * app 升级完全共用. */
	if (g_upg.cur_boot) {
		post_log(L"MCUboot 救援模式: 已发送重启命令 (设备死机时无效), 等待 bootloader...");
		post_log(L"若设备未重启, 请立即手动断电/复位设备 (最多等待 60 秒)");
		if (!CanManager_EnterBoot(g_upg.can)) {
			wchar_t m[256];
			const char *e = CanManager_GetLastError(g_upg.can);
			wchar_t werr[192];
			MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
			swprintf(m, 256, L"进入 MCUboot 救援模式失败: %ls", werr);
			post_log(m);
			post_done(0);
			return 0;
		}
		post_log(L"bootloader 已应答 (0x106), 开始升级...");
	}

	bool ok = CanManager_FirmwareUpgrade(g_upg.can, g_upg.img, g_upg.img_size,
	                                     kh, permanent, can_progress_handler, NULL);
	if (!ok) {
		wchar_t m[256];
		const char *e = CanManager_GetLastError(g_upg.can);
		wchar_t werr[192];
		MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 192);
		swprintf(m, 256, L"CAN 升级失败: %ls", werr);
		post_log(m);
		post_done(0);
		return 0;
	}

	post_progress(100, 2);
	if (g_upg.cur_boot) {
		/* bootloader 模式: 数据写 slot0, CONFIRM 后 MCUboot 直接验证并启动
		 * 新固件 (无 swap 标记, 不走 SWAP_SCRATCH), 无需再发 REBOOT */
		post_log(L"救援模式完成: 已写 slot0, MCUboot 验证后启动新固件, 无需重启");
	} else {
		/* app 模式 (embassy-boot): 数据写 NOR DFU 暂存分区, CONFIRM 通过
		 * ed25519 验签后写 state SWAP 魔数, 须重启设备才触发逐页换机。
		 * 重启由升级完成弹窗的『立即重启』按钮 (或本页『重启』按钮) 触发 */
		post_log(L"CAN 升级完成, 请重启设备完成 embassy-boot 换机 DFU→active (可在弹窗或本页『重启』按钮操作)");
	}
	post_done(1);
	return 0;
}

/* ===== 开始升级 / 取消 (UI 线程) ===== */

static void on_start(void)
{
	if (!g_upg.img || g_upg.img_size == 0) {
		MessageBoxW(g_upg.hSelf, L"请先选择有效的固件文件", L"提示",
		            MB_ICONWARNING);
		return;
	}
	if (g_upg.thread) {
		MessageBoxW(g_upg.hSelf, L"已有升级任务在进行", L"提示",
		            MB_ICONWARNING);
		return;
	}

	int can = current_channel();

	if (!can) {
		g_upg.cur_can = false;
		/* UDP: 必须已连接 */
		if (!g_upg.udp_connected) {
			MessageBoxW(g_upg.hSelf, L"请先点 \"连接\" 连接设备", L"提示",
			            MB_ICONWARNING);
			return;
		}
		/* UDP: 检查目标 IP 已填 (单框) */
		uint8_t ip4[4];
		if (!read_ip_edit(g_upg.hIp, ip4)) {
			MessageBoxW(g_upg.hSelf, L"请填写目标设备 IP (如 192.168.12.101)", L"提示",
			            MB_ICONWARNING);
			return;
		}
		/* read_ip_edit 已校验过, 重新拼合法串覆盖, 避免传输层收到残缺串 */
		snprintf(g_upg.cur_ip, sizeof(g_upg.cur_ip),
		         "%d.%d.%d.%d", ip4[0], ip4[1], ip4[2], ip4[3]);
		/* UDP: 升级前查询一次设备版本, 刷新 label */
		on_query_version();
		g_upg.cur_permanent = true;
	} else {
		/* CAN: 检查已连接 */
		if (!g_upg.can_connected) {
			MessageBoxW(g_upg.hSelf, L"请先点 \"连接\" 接入 PCAN 设备", L"提示",
			            MB_ICONWARNING);
			return;
		}
		g_upg.cur_can = true;
		/* CAN: 永久升级 (无测试模式) + MCUboot 救援模式复选框 */
		g_upg.cur_permanent = true;
		g_upg.cur_boot = (SendMessageW(g_upg.hCanBoot, BM_GETCHECK, 0, 0) == BST_CHECKED);
		if (g_upg.cur_boot) {
			MessageBoxW(g_upg.hSelf,
				L"即将进入 MCUboot 紧急救援模式。\n"
				L"若设备死机无法响应重启命令，请立即手动给设备断电（或按复位键）重启，\n"
				L"程序会自动检测 bootloader 探测帧并开始刷机（最多等待 60 秒）。",
				L"MCUboot 救援模式", MB_ICONINFORMATION);
		}
	}

	/* 禁用开始/浏览, 重置进度条. 浏览须禁用: on_browse 会
	 * free_image() 后重赋 g_upg.img, 升级期间 worker 正读 img, 误触将
	 * use-after-free. */
	EnableWindow(g_upg.hStart, FALSE);
	EnableWindow(g_upg.hBrowse, FALSE);
	SendMessageW(g_upg.hProgress, PBM_SETPOS, 0, 0);
	SetWindowTextW(g_upg.hStatus, L"升级中...");
	InterlockedExchange(&g_upg.cancel, 0);
	g_upg.wait_reboot = false;
	g_upg.pending_reboot = false;
	KillTimer(g_upg.hSelf, UPG_REBOOT_TIMER_ID);
	/* 快照当前版本 label 对应的版本串作 "旧版本" (label 可能含提示文案,
	 * 直接用缓存的查询结果, 未查询过则为空, 重启确认时不拼箭头) */

	DWORD tid = 0;
	HANDLE h = CreateThread(NULL, 0,
	                        can ? can_upgrade_thread : udp_upgrade_thread,
	                        NULL, 0, &tid);
	if (!h) {
		MessageBoxW(g_upg.hSelf, L"创建升级线程失败", L"错误", MB_ICONERROR);
		EnableWindow(g_upg.hBrowse, TRUE);
		update_button_state();
		return;
	}
	g_upg.thread = h;
}

/* 重启设备: UDP 通道走 UdpManager_Reboot (0x05), CAN 通道走 CanManager_Reboot.
 * 成功后若处于 "升级成功 (待重启)" 场景, 进入重启中状态并起定时器确认. */
static void on_reboot(void)
{
	int can = current_channel();
	bool ok;
	wchar_t m[220];
	if (!can) {
		if (!g_upg.udp_connected) {
			MessageBoxW(g_upg.hSelf, L"请先点 \"连接\" 连接设备", L"提示",
			            MB_ICONWARNING);
			return;
		}
		uint8_t ip4[4];
		if (!read_ip_edit(g_upg.hIp, ip4)) {
			MessageBoxW(g_upg.hSelf, L"请先填写目标设备 IP (如 192.168.12.101)", L"提示",
			            MB_ICONWARNING);
			return;
		}
		char ip[64] = {0};
		snprintf(ip, sizeof(ip), "%d.%d.%d.%d", ip4[0], ip4[1], ip4[2], ip4[3]);
		ok = UdpManager_Reboot(g_upg.udp, ip);
		if (!ok) {
			log_append_ptr(L"重启命令发送失败 (设备无响应)");
		}
	} else {
		if (!g_upg.can_connected) {
			MessageBoxW(g_upg.hSelf, L"请先点 \"连接\" 接入 PCAN 设备", L"提示",
			            MB_ICONWARNING);
			return;
		}
		ok = CanManager_Reboot(g_upg.can);
		if (!ok) {
			const char *e = CanManager_GetLastError(g_upg.can);
			wchar_t werr[128];
			MultiByteToWideChar(CP_UTF8, 0, e, -1, werr, 128);
			swprintf(m, 220, L"重启失败: %ls", werr);
			log_append_ptr(m);
		}
	}
	if (!ok) {
		return;
	}
	log_append_ptr(L"重启命令已发送");
	if (g_upg.pending_reboot) {
		/* 升级后待重启场景: 进入 "重启中" + 定时器到点查版本收尾 */
		g_upg.pending_reboot = false;
		g_upg.wait_reboot = true;
		SetWindowTextW(g_upg.hStatus, L"升级成功 (重启中)");
		log_append_ptr(L"embassy-boot 换机中 (约 30-60 秒, 稍后自动确认)");
		SetTimer(g_upg.hSelf, UPG_REBOOT_TIMER_ID, UPG_REBOOT_WAIT_MS, NULL);
	}
}

/* ===== WM_COMMAND 分发 ===== */

static void on_command(WPARAM wParam)
{
	WORD id = LOWORD(wParam);
	WORD code = HIWORD(wParam);

	/* 通道单选切换 */
	if (id == IDC_UPG_CHAN_UDP && code == BN_CLICKED) {
		apply_channel_visibility();
		return;
	}
	if (id == IDC_UPG_CHAN_CAN && code == BN_CLICKED) {
		apply_channel_visibility();
		refresh_can_device();   /* 切到 CAN 通道时自动扫描设备 */
		return;
	}
	if (code != BN_CLICKED) return;

	switch (id) {
	case IDC_UPG_BROWSE:    on_browse(); break;
	case IDC_UPG_GETVER:    on_query_version(); break;
	case IDC_UPG_START:     on_start(); break;
	case IDC_UPG_REBOOT:    on_reboot(); break;
	case IDC_UPG_CAN_CONN:  on_connect(); break;
	case IDC_UPG_CAN_REFRESH: refresh_can_device(); break;
	}
}

/* ===== WM_CREATE: 创建所有控件 ===== */

static void create_controls(HWND hWnd)
{
	g_upg.hSelf = hWnd;
	g_hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

	/* 行坐标基准 */
	int gx = 12, gw = 776;

	/* ===== 升级通道 groupbox ===== */
	create_groupbox(L"升级通道", gx, 4, gw, 100);
	/* 行1: 通道单选 */
	create_label(L"通道:", gx + 12, 32, 40, 14);
	g_upg.hChanUdp = CreateWindowExW(0, L"BUTTON", L"UDP",
		WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP,
		gx + 56, 28, 60, 22, hWnd, (HMENU)(INT_PTR)IDC_UPG_CHAN_UDP, g_hInst, NULL);
	g_upg.hChanCan = CreateWindowExW(0, L"BUTTON", L"CAN (PCAN)",
		WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
		gx + 120, 28, 120, 22, hWnd, (HMENU)(INT_PTR)IDC_UPG_CHAN_CAN, g_hInst, NULL);
	SendMessageW(g_upg.hChanUdp, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	SendMessageW(g_upg.hChanCan, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	SendMessageW(g_upg.hChanUdp, BM_SETCHECK, BST_CHECKED, 0); /* 默认 UDP */
	/* 行1: 连接/断开 (UDP/CAN 通用, 未连接时其他确认按钮置灰) */
	g_upg.hConn = create_button(L"连接", gx + 250, 28, 80, 24, IDC_UPG_CAN_CONN);

	/* 行2: UDP 目标 IP (单框, 默认显示).
	 * 数据发送优先 V2 窗口流水线 (设备新固件), 老固件自动回退停等模式 */
	g_upg.hUdpLbl = create_label(L"目标 IP:", gx + 12, 62, 56, 14);
	g_upg.hIp = create_edit(gx + 70, 58, 140, 22, IDC_UPG_IP1, 0);
	SetWindowTextW(g_upg.hIp, L"192.168.12.101");

	/* 行2 (重叠位置, 默认隐藏): CAN PCAN 设备 + 波特率 + 刷新 + 连接 */
	g_upg.hCanLbl1 = create_label(L"PCAN 设备:", gx + 12, 62, 64, 14);
	g_upg.hCanDev = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
		gx + 78, 58, 140, 200, hWnd, (HMENU)(INT_PTR)IDC_UPG_CAN_DEV, g_hInst, NULL);
	SendMessageW(g_upg.hCanDev, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	g_upg.hCanLbl2 = create_label(L"波特率:", gx + 226, 62, 48, 14);
	g_upg.hCanBaud = CreateWindowExW(0, L"COMBOBOX", L"",
		WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
		gx + 274, 58, 130, 200, hWnd, (HMENU)(INT_PTR)IDC_UPG_CAN_BAUD, g_hInst, NULL);
	SendMessageW(g_upg.hCanBaud, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	for (int i = 0; i < BAUD_COUNT; i++) {
		SendMessageW(g_upg.hCanBaud, CB_ADDSTRING, 0, (LPARAM)g_bauds[i].label);
	}
	SendMessageW(g_upg.hCanBaud, CB_SETCURSEL, 0, 0); /* 默认 250k */
	g_upg.hCanRefresh = create_button(L"刷新", gx + gw - 96, 58, 80, 24, IDC_UPG_CAN_REFRESH);
	/* MCUboot 紧急救援模式: 仅旧 C/Zephyr 固件支持 0x106/0x107 探测应答;
	 * embassy-boot 固件无此机制, 勾选后会在探测阶段超时 */
	g_upg.hCanBoot = CreateWindowExW(0, L"BUTTON", L"救援模式 (仅旧 MCUboot 固件)",
		WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
		gx + 412, 58, 200, 22, hWnd, (HMENU)(INT_PTR)IDC_UPG_CAN_BOOT, g_hInst, NULL);
	SendMessageW(g_upg.hCanBoot, WM_SETFONT, (WPARAM)g_hFont, TRUE);

	/* 默认 UDP, 隐藏 CAN 行 */
	apply_channel_visibility();

	/* ===== 版本信息行: label + 版本号 label + 查询按钮 ===== */
	g_upg.hVerLbl = create_label(L"设备版本:", gx + 12, 116, 60, 14);
	g_upg.hVersion = create_label(L"(未查询)", gx + 76, 114, 380, 14);
	g_upg.hGetVer = create_button(L"查询版本", gx + gw - 96, 112, 80, 24, IDC_UPG_GETVER);

	/* ===== 固件文件 groupbox ===== */
	/* ===== 固件升级 groupbox (固件文件 + 升级控制合并) ===== */
	create_groupbox(L"固件升级", gx, 150, gw, 130);
	/* 行1: 固件文件路径 + 浏览 + 文件信息 */
	create_label(L"路径:", gx + 12, 176, 36, 14);
	g_upg.hFile = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | ES_READONLY,
		gx + 50, 172, 440, 22, hWnd, (HMENU)(INT_PTR)IDC_UPG_FILE, g_hInst, NULL);
	SendMessageW(g_upg.hFile, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	g_upg.hBrowse = create_button(L"浏览...", gx + 498, 172, 70, 22, IDC_UPG_BROWSE);
	g_upg.hFileInfo = create_label(L"(未选择)", gx + 576, 176, 200, 14);
	/* 行2: 进度 + 状态 (左侧), 开始升级 + 重启 (右侧右对齐) */
	create_label(L"进度:", gx + 12, 246, 36, 14);
	g_upg.hProgress = CreateWindowExW(0, PROGRESS_CLASSW, L"",
		WS_CHILD | WS_VISIBLE, gx + 50, 244, 260, 18,
		hWnd, (HMENU)(INT_PTR)IDC_UPG_PROGRESS, g_hInst, NULL);
	SendMessageW(g_upg.hProgress, PBM_SETRANGE, 0, MAKELPARAM(0, 100));
	SendMessageW(g_upg.hProgress, PBM_SETPOS, 0, 0);
	g_upg.hStatus = create_label(L"就绪", gx + 318, 246, 268, 14);
	g_upg.hStart = create_button(L"开始升级", gx + gw - 180, 242, 80, 24, IDC_UPG_START);
	g_upg.hReboot = create_button(L"重启", gx + gw - 96, 242, 80, 24, IDC_UPG_REBOOT);
	EnableWindow(g_upg.hStart, FALSE);

	/* ===== 操作日志 groupbox + 多行只读 EDIT ===== */
	create_groupbox(L"操作日志", gx, 300, gw, 468);
	g_upg.hLog = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY |
		ES_AUTOVSCROLL | WS_VSCROLL,
		gx + 12, 320, gw - 24, 440,
		hWnd, (HMENU)(INT_PTR)IDC_UPG_LOG, g_hInst, NULL);
	SendMessageW(g_upg.hLog, WM_SETFONT, (WPARAM)g_hFont, TRUE);

	/* 初始探测一次 PCAN 设备 (供切换到 CAN 时已有列表) */
	refresh_can_device();
}

/* ===== 窗口过程 ===== */

static LRESULT CALLBACK upg_wndproc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		g_hInst = ((LPCREATESTRUCT)lParam)->hInstance;
		create_controls(hWnd);
		g_upg.udp = UdpManager_Create();
		g_upg.can = CanManager_Create();
		g_upg.thread = NULL;
		g_upg.cancel = 0;
		g_upg.udp_connected = false;
		g_upg.wait_reboot = false;
		g_upg.pending_reboot = false;
		if (!g_upg.udp) {
			log_append_ptr(L"错误: UdpManager 创建失败");
		}
		if (!g_upg.can) {
			log_append_ptr(L"错误: CanManager 创建失败");
		}
		log_append_ptr(L"就绪. 请选择 .bin 固件文件");
		return 0;
	case WM_COMMAND:
		on_command(wParam);
		return 0;
	case WM_SIZE:
		/* 控件保持固定位置 (与 tab1 一致). */
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

	/* ===== 自定义消息: worker → UI ===== */
	case WM_APP_UPG_PROGRESS: {
		int percent = (int)wParam;
		int stage = (int)lParam;
		SendMessageW(g_upg.hProgress, PBM_SETPOS, percent, 0);
		wchar_t s[64];
		if (stage == 1) {
			swprintf(s, 64, L"发送数据 %d%%", percent);
		} else if (stage == 2) {
			wcscpy(s, L"等待设备重启 (embassy-boot 换机)");
		} else {
			swprintf(s, 64, L"%d%%", percent);
		}
		SetWindowTextW(g_upg.hStatus, s);
		return 0;
	}
	case WM_APP_UPG_LOG: {
		const wchar_t *msg = (const wchar_t *)lParam;
		if (msg) {
			log_append_ptr(msg);
			free((void *)msg);
		}
		return 0;
	}
	case WM_APP_UPG_DONE: {
		int success = (int)wParam;
		/* 关闭线程句柄 */
		if (g_upg.thread) {
			CloseHandle(g_upg.thread);
			g_upg.thread = NULL;
		}
		/* 恢复按钮 (与 on_start 的禁用对称: start + 浏览) */
		EnableWindow(g_upg.hBrowse, TRUE);
		update_button_state();
		if (success) {
		if (g_upg.cur_boot) {
			/* MCUboot 救援模式: 数据已写 slot0, CONFIRM 后 MCUboot 直接
			 * 验证并启动新固件, 无需重启设备 */
			SetWindowTextW(g_upg.hStatus, L"升级成功");
			post_log(L"MCUboot 救援模式升级成功, 无需重启设备");
			MessageBoxW(g_hMain,
				L"升级成功 (MCUboot 救援模式)\n\n"
				L"新固件已写入 slot0，MCUboot 将直接验证并启动，\n"
				L"无需重启设备。",
				L"升级完成", MB_ICONINFORMATION | MB_OK);
			/* MCUboot 验证+启动新固件需数秒, 5s 后自动查版本刷新显示 */
			SetTimer(g_upg.hSelf, UPG_BOOT_OK_TIMER_ID, UPG_BOOT_OK_WAIT_MS, NULL);
			} else {
				/* CAN/UDP app 模式 (embassy-boot): 数据在 NOR DFU 暂存分区,
				 * 已写 state SWAP 魔数, 须重启设备才完成逐页换机 → 弹窗提示 +
				 * 立即重启按钮. pending_reboot 标记 "升级成功待重启": 弹窗选否后
				 * 手动点『重启』也能进入 重启中+定时器确认 流程 */
				SetWindowTextW(g_upg.hStatus, L"升级成功 (待重启)");
				g_upg.pending_reboot = true;
				if (MessageBoxW(g_hMain,
					L"升级成功！\n\n"
					L"新固件已写入 DFU 暂存分区，需要重启设备完成 embassy-boot 固件换机 (约 30 秒)。\n"
					L"是否立即重启设备？\n\n"
					L"(选『否』可稍后用本页『重启』按钮手动重启)",
					L"升级完成 - 请重启设备", MB_ICONQUESTION | MB_YESNO) == IDYES) {
					bool ok = g_upg.cur_can ? CanManager_Reboot(g_upg.can)
					                        : UdpManager_Reboot(g_upg.udp, g_upg.cur_ip);
					if (ok) {
						g_upg.pending_reboot = false;
						SetWindowTextW(g_upg.hStatus, L"升级成功 (重启中)");
						log_append_ptr(L"重启命令已发送, embassy-boot 换机中 (约 30-60 秒, 稍后自动确认)");
						/* 40s 后查版本确认设备上线, 避免 "重启中" 状态永久悬挂 */
						g_upg.wait_reboot = true;
						SetTimer(g_upg.hSelf, UPG_REBOOT_TIMER_ID,
						         UPG_REBOOT_WAIT_MS, NULL);
					} else {
						log_append_ptr(L"重启命令发送失败, 请手动断电重启设备");
					}
				} else {
					log_append_ptr(L"请稍后重启设备完成 embassy-boot 换机 (本页『重启』按钮或断电重启)");
				}
			}
		} else {
			SetWindowTextW(g_upg.hStatus, L"升级失败");
			g_upg.pending_reboot = false;
			MessageBoxW(g_hMain, L"升级失败 (详见日志)", L"结果", MB_ICONERROR);
		}
		/* 升级流程已结束 (含弹窗交互): 进度条清零, 不再保留 100% 满条 */
		SendMessageW(g_upg.hProgress, PBM_SETPOS, 0, 0);
		return 0;
	}

	case WM_TIMER:
		/* 升级后重启确认: 设备应已完成 embassy-boot 换机并带新固件上线,
		 * 查一次版本收尾; 未响应则提示用户手动确认 */
		if (wParam == UPG_REBOOT_TIMER_ID) {
			KillTimer(hWnd, UPG_REBOOT_TIMER_ID);
			if (!g_upg.wait_reboot) {
				return 0;
			}
			g_upg.wait_reboot = false;
			/* 用户已又开始新升级: 不打扰 */
			if (g_upg.thread) {
				return 0;
			}
			char ver[80] = {0};
			bool up;
			if (g_upg.cur_can) {
				up = g_upg.can_connected &&
				     CanManager_GetVersion(g_upg.can, ver, sizeof(ver));
			} else {
				up = g_upg.udp_connected &&
				     UdpManager_GetVersion(g_upg.udp, g_upg.cur_ip, ver, sizeof(ver));
			}
			if (up) {
				wchar_t wver[160] = {0};
				wchar_t m[260];
				MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 160);
				if (g_upg.old_ver[0] != '\0' &&
				    strcmp(g_upg.old_ver, ver) != 0) {
					/* 升级前查询过版本且与新版不同: 显示 "旧 → 新" */
					wchar_t wold[160] = {0};
					MultiByteToWideChar(CP_UTF8, 0, g_upg.old_ver, -1, wold, 160);
					swprintf(m, 260, L"%ls  →  %ls", wold, wver);
					SetWindowTextW(g_upg.hVersion, m);
					swprintf(m, 260, L"设备已重启, 固件已更新: %ls → %ls", wold, wver);
				} else {
					SetWindowTextW(g_upg.hVersion, wver);
					swprintf(m, 260, L"设备已重启, 新固件运行中: %ls", wver);
				}
				log_append_ptr(m);
				SetWindowTextW(g_upg.hStatus, L"升级成功");
			} else {
				log_append_ptr(L"设备重启后未响应版本查询 (可能仍在交换), 请稍后手动确认");
				SetWindowTextW(g_upg.hStatus, L"升级成功 (请确认设备已重启)");
			}
			return 0;
		}
		/* 救援模式升级确认: MCUboot 已直接启动新固件, 查版本刷新显示
		 * (含 "旧 → 新" 版本对比) */
		if (wParam == UPG_BOOT_OK_TIMER_ID) {
			KillTimer(hWnd, UPG_BOOT_OK_TIMER_ID);
			/* 用户已又开始新升级: 不打扰 */
			if (g_upg.thread) {
				return 0;
			}
			char ver[80] = {0};
			bool up = CanManager_GetVersion(g_upg.can, ver, sizeof(ver));
			if (up) {
				wchar_t wver[160] = {0};
				wchar_t m[260];
				MultiByteToWideChar(CP_UTF8, 0, ver, -1, wver, 160);
				if (g_upg.old_ver[0] != '\0' &&
				    strcmp(g_upg.old_ver, ver) != 0) {
					wchar_t wold[160] = {0};
					MultiByteToWideChar(CP_UTF8, 0, g_upg.old_ver, -1, wold, 160);
					swprintf(m, 260, L"%ls  →  %ls", wold, wver);
					SetWindowTextW(g_upg.hVersion, m);
					swprintf(m, 260, L"新固件已启动: %ls → %ls", wold, wver);
				} else {
					SetWindowTextW(g_upg.hVersion, wver);
					swprintf(m, 260, L"新固件已启动: %ls", wver);
				}
				log_append_ptr(m);
			} else {
				log_append_ptr(L"新固件版本查询未响应 (MCUboot 可能仍在验证), 请稍后手动查询");
			}
		}
		return 0;

	case WM_DESTROY:
		/* 取消挂起的重启确认定时器 */
		KillTimer(hWnd, UPG_REBOOT_TIMER_ID);
		KillTimer(hWnd, UPG_BOOT_OK_TIMER_ID);
		g_upg.wait_reboot = false;
		g_upg.pending_reboot = false;
		/* 等 worker 退出 (UI 销毁时通常已 DONE; 防御性等待避免悬挂线程) */
		if (g_upg.thread) {
			InterlockedExchange(&g_upg.cancel, 1);
			DWORD wres = WaitForSingleObject(g_upg.thread, 2000);
			if (wres == WAIT_OBJECT_0) {
				/* worker 已干净退出: 关闭句柄, 后续可安全销毁 manager/image */
				CloseHandle(g_upg.thread);
				g_upg.thread = NULL;
			} else {
				/* WAIT_TIMEOUT: worker 仍在阻塞 (典型为 CAN 升级的
				 * CanManager_FirmwareUpgrade — 纯阻塞调用, 无法中断).
				 * worker 仍在读 g_upg.img / 操作 g_upg.can, 此时若销毁
				 * manager 或释放 image 将触发 use-after-free. 故仅关闭线程
				 * 句柄 (让进程可退出), 跳过 manager/image 释放与 CAN 断开 —
				 * 这是有意为之的有界泄漏: 资源在进程终止时由 OS 回收.
				 * (泄漏永远安全; 释放另一线程正在用的内存永不安全.) */
				CloseHandle(g_upg.thread);
				g_upg.thread = NULL;
				return 0;
			}
		}
		if (g_upg.can_connected) {
			CanManager_Disconnect(g_upg.can);
			g_upg.can_connected = false;
		}
		if (g_upg.udp) {
			UdpManager_Destroy(g_upg.udp);
			g_upg.udp = NULL;
		}
		if (g_upg.can) {
			CanManager_Destroy(g_upg.can);
			g_upg.can = NULL;
		}
		free_image();
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

/* ===== 公共 API ===== */

HWND UpgradeTab_Create(HWND hParent, HINSTANCE hInst)
{
	g_hInst = hInst;

	if (!g_classRegistered) {
		WNDCLASSW wc = {0};
		wc.lpfnWndProc = upg_wndproc;
		wc.hInstance = hInst;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
		wc.lpszClassName = UPGRADE_TAB_CLASS;
		RegisterClassW(&wc);
		g_classRegistered = TRUE;
	}

	HWND h = CreateWindowExW(0, UPGRADE_TAB_CLASS, L"",
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
		0, 0, 700, 500, hParent, NULL, hInst, NULL);
	return h;
}
