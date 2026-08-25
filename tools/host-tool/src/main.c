/* 数据采集卡软件 - Win32 GUI 主入口
 * Tab1: 参数设置
 * Tab2: 固件升级 (UDP + CAN)
 * Tab3: Modbus 调试 (TCP + RTU)
 * Tab4: 历史记录解析
 */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <commctrl.h>
#include <stdio.h>
#include <stdarg.h>
#include "app.h"
#include "config_tab.h"
#include "upgrade_tab.h"
#include "modbus_tab.h"
#include "history_tab.h"
#include "resource.h"

#pragma comment(lib, "comctl32.lib")

HINSTANCE g_hInst;
HWND g_hMain;
static HWND g_hTab;
static HWND g_hTabDlg[TAB_COUNT];
static HWND g_hStatus;
static CRITICAL_SECTION g_logCs;

/* AppLog_Printf: 状态栏临时显示 (简单实现, 后续 tab 自己有日志框). */
void AppLog_Printf(const wchar_t *fmt, ...)
{
	wchar_t buf[512];
	va_list ap;
	va_start(ap, fmt);
	vswprintf(buf, 512, fmt, ap);
	va_end(ap);

	EnterCriticalSection(&g_logCs);
	if (g_hStatus) {
		SendMessageW(g_hStatus, SB_SETTEXTW, 0, (LPARAM)buf);
	}
	LeaveCriticalSection(&g_logCs);
}

/* 主窗口 on WM_SIZE: tab 控件 + 状态栏自适应. */
static void main_on_size(int cx, int cy)
{
	if (g_hStatus) {
		SendMessageW(g_hStatus, WM_SIZE, 0, 0);
	}
	RECT rcStatus = {0};
	GetWindowRect(g_hStatus, &rcStatus);
	int statusH = rcStatus.bottom - rcStatus.top;

	if (g_hTab) {
		MoveWindow(g_hTab, 0, 0, cx, cy - statusH, TRUE);
	}
	/* tab 子对话框贴 tab 控件显示区 */
	if (g_hTab) {
		RECT rcDisp = {0};
		TabCtrl_GetItemRect(g_hTab, 0, &rcDisp);
		int top = rcDisp.bottom + 4;
		for (int i = 0; i < TAB_COUNT; i++) {
			if (g_hTabDlg[i]) {
				MoveWindow(g_hTabDlg[i], 4, top, cx - 8, cy - statusH - top - 4, TRUE);
			}
		}
	}
}

static LRESULT CALLBACK main_wndproc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE: {
		g_hTab = CreateWindowExW(0, WC_TABCONTROLW, L"",
			WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | TCS_TABS,
			0, 0, 0, 0, hWnd, (HMENU)0, g_hInst, NULL);
		/* tab 标题用常规 GUI 字体 (默认系统字体可能渲染为粗体) */
		SendMessageW(g_hTab, WM_SETFONT,
			(WPARAM)GetStockObject(DEFAULT_GUI_FONT), TRUE);

		TCITEMW it = { .mask = TCIF_TEXT };
		it.pszText = (LPWSTR)L"参数设置";
		TabCtrl_InsertItem(g_hTab, TAB_CONFIG, &it);
		it.pszText = (LPWSTR)L"固件升级";
		TabCtrl_InsertItem(g_hTab, TAB_UPGRADE, &it);
		it.pszText = (LPWSTR)L"Modbus 调试";
		TabCtrl_InsertItem(g_hTab, TAB_MODBUS, &it);
		it.pszText = (LPWSTR)L"历史记录";
		TabCtrl_InsertItem(g_hTab, TAB_HISTORY, &it);

		g_hStatus = CreateWindowExW(0, STATUSCLASSNAMEW, L"就绪",
			WS_CHILD | WS_VISIBLE | SBARS_SIZEGRIP,
			0, 0, 0, 0, hWnd, (HMENU)IDC_STATUSBAR, g_hInst, NULL);

		g_hTabDlg[TAB_CONFIG]  = ConfigTab_Create(g_hTab, g_hInst);
		g_hTabDlg[TAB_UPGRADE] = UpgradeTab_Create(g_hTab, g_hInst);
		g_hTabDlg[TAB_MODBUS]  = ModbusTab_Create(g_hTab, g_hInst);
		g_hTabDlg[TAB_HISTORY] = HistoryTab_Create(g_hTab, g_hInst);
		for (int i = 0; i < TAB_COUNT; i++) {
			ShowWindow(g_hTabDlg[i], i == TAB_CONFIG ? SW_SHOW : SW_HIDE);
		}
		return 0;
	}
	case WM_NOTIFY: {
		LPNMHDR pnmh = (LPNMHDR)lParam;
		if (pnmh->hwndFrom == g_hTab && pnmh->code == TCN_SELCHANGE) {
			int sel = TabCtrl_GetCurSel(g_hTab);
			for (int i = 0; i < TAB_COUNT; i++) {
				ShowWindow(g_hTabDlg[i], i == sel ? SW_SHOW : SW_HIDE);
			}
		}
		return 0;
	}
	case WM_SIZE:
		main_on_size(LOWORD(lParam), HIWORD(lParam));
		return 0;
	case WM_GETMINMAXINFO: {
		MINMAXINFO *mmi = (MINMAXINFO *)lParam;
		mmi->ptMinTrackSize.x = 800;
		mmi->ptMinTrackSize.y = 840;
		return 0;
	}
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrev, PWSTR cmdLine, int show)
{
	INITCOMMONCONTROLSEX icc = { .dwSize = sizeof(icc), .dwICC = ICC_TAB_CLASSES | ICC_BAR_CLASSES | ICC_LISTVIEW_CLASSES };
	InitCommonControlsEx(&icc);

	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);

	InitializeCriticalSection(&g_logCs);
	g_hInst = hInstance;

	WNDCLASSEXW wc = { .cbSize = sizeof(wc) };
	wc.lpfnWndProc = main_wndproc;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
	wc.lpszClassName = L"ioEdgeHubMainCls";
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCEW(IDI_APP_ICON));
	RegisterClassExW(&wc);

	g_hMain = CreateWindowExW(0, wc.lpszClassName, L"数据采集卡软件 v" APP_VERSION_W,
		WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
		0, 0, 800, 840,
		NULL, NULL, hInstance, NULL);

	/* 主窗口居中到屏幕工作区 (排除任务栏). 在 ShowWindow 之前调整, 避免可见位置跳变. */
	RECT rcWnd, rcWork;
	GetWindowRect(g_hMain, &rcWnd);
	SystemParametersInfoW(SPI_GETWORKAREA, 0, &rcWork, 0);
	int wndW = rcWnd.right - rcWnd.left;
	int wndH = rcWnd.bottom - rcWnd.top;
	int cx = rcWork.left + ((rcWork.right - rcWork.left) - wndW) / 2;
	int cy = rcWork.top  + ((rcWork.bottom - rcWork.top) - wndH) / 2;
	SetWindowPos(g_hMain, NULL, cx, cy, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	ShowWindow(g_hMain, show);
	UpdateWindow(g_hMain);

	MSG msg;
	while (GetMessageW(&msg, NULL, 0, 0)) {
		TranslateMessage(&msg);
		DispatchMessageW(&msg);
	}

	DeleteCriticalSection(&g_logCs);
	WSACleanup();
	return (int)msg.wParam;
}
