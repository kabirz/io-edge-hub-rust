#ifndef APP_H
#define APP_H

#include <windows.h>

/* 应用版本号宽字符串 (CMakeLists 注入 APP_VERSION_MAJOR/MINOR/PATCH).
 * 双层宏字符串化 + L"" 拼接 (MSVC 不支持 L#x). */
#define ZC_STR2(x) #x
#define ZC_STR(x)  ZC_STR2(x)
#define APP_VERSION_W L"" ZC_STR(APP_VERSION_MAJOR) L"." \
                      L"" ZC_STR(APP_VERSION_MINOR) L"." \
                      L"" ZC_STR(APP_VERSION_PATCH)

/* 工作线程 → UI 线程 自定义消息 */
#define WM_APP_UPG_PROGRESS  (WM_APP + 1)  /* wParam=0-100, lParam=阶段码 */
#define WM_APP_UPG_LOG       (WM_APP + 2)  /* lParam=堆字符串指针, UI 收到 free */
#define WM_APP_UPG_DONE      (WM_APP + 3)  /* wParam=1 成功 / 0 失败 */

/* 全局实例 (main.c 定义) */
extern HINSTANCE g_hInst;
extern HWND g_hMain;

/* 公共日志: 向主窗口底部状态栏临时显示 + 控制台打印 (后续 tab 各自维护日志框).
 * 线程安全: 内部临界区. */
void AppLog_Printf(const wchar_t *fmt, ...);

/* tab 工厂: 在主窗口 tab 控件内创建子对话框, 返回子窗口 HWND.
 * 每个 tab 模块在各自 .c 实现. */
HWND ConfigTab_Create(HWND hParent, HINSTANCE hInst);
HWND UpgradeTab_Create(HWND hParent, HINSTANCE hInst);
HWND ModbusTab_Create(HWND hParent, HINSTANCE hInst);
HWND HistoryTab_Create(HWND hParent, HINSTANCE hInst);

#endif /* APP_H */
