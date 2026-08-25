/* io-edge-hub 上位机 - Tab4 "历史记录"
 *
 * 解析设备导出的历史记录文件 (data_*.raw, LittleFS, 经 FTP 下载):
 *   DI 记录 10B: [type u16=1][ts u32][di_en u16][di_val u16]  (小端)
 *   AI 记录 16B: [type u16=2][ts u32][ai_en u16][ai_val[4] u16]
 * 与固件 include/init.h 的 struct his_data (__packed) 完全一致.
 *
 * 界面: 打开 .raw → 解析全部记录 → ListView 虚拟模式展示 (序号/时间/类型/值详情),
 *       可导出 CSV. 记录可能达数万条, 虚拟模式 (LVS_OWNERDATA) 下控件只为
 *       可见行经 LVN_GETDISPINFOW 按需取文本, 避免逐条插入卡死界面.
 */
#include "history_tab.h"
#include "resource.h"
#include <commctrl.h>
#include <commdlg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <wchar.h>

/* 记录类型 (与固件 init.h 一致) */
#define HIST_DI_TYPE 1
#define HIST_AI_TYPE 2
#define HIST_DI_LEN  10
#define HIST_AI_LEN  16

/* ===== 静态状态 ===== */
typedef struct {
	HWND hSelf;
	HWND hOpen, hExport, hInfo;
	HWND hList;
	HWND hLog;
} HistoryTab;

typedef struct {
	uint8_t  type;       /* HIST_DI_TYPE / HIST_AI_TYPE */
	uint32_t ts;         /* Unix 秒 */
	uint16_t en;         /* DI 使能 / AI 使能 */
	uint16_t di_val;     /* DI 值 bitmap */
	uint16_t ai_val[4];  /* AI 值 (AI1/2 mA, AI3/4 V, 0.01 精度) */
} HistRec;

static HistoryTab g_his;
static HFONT g_hFont = NULL;
static const wchar_t *HISTORY_TAB_CLASS = L"ioEdgeHubHistoryTabCls";
static BOOL g_classRegistered = FALSE;

static HistRec *g_recs;
static int g_rec_count;
static int g_rec_cap;

/* ===== 控件创建辅助 (与其它 tab 一致) ===== */

static HWND create_label(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"STATIC", text,
		WS_CHILD | WS_VISIBLE, x, y, w, h,
		g_his.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_button(const wchar_t *text, int x, int y, int w, int h, int id)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		x, y, w, h, g_his.hSelf, (HMENU)(INT_PTR)id, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

static HWND create_groupbox(const wchar_t *text, int x, int y, int w, int h)
{
	HWND hw = CreateWindowExW(0, L"BUTTON", text,
		WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
		x, y, w, h, g_his.hSelf, NULL, g_hInst, NULL);
	SendMessageW(hw, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	return hw;
}

/* ===== 日志 ===== */

static void log_append(const wchar_t *msg)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	wchar_t line[600];
	swprintf(line, 600, L"[%02d:%02d:%02d] %ls\r\n",
	         st.wHour, st.wMinute, st.wSecond, msg);
	int len = GetWindowTextLengthW(g_his.hLog);
	SendMessageW(g_his.hLog, EM_SETSEL, len, len);
	SendMessageW(g_his.hLog, EM_REPLACESEL, 0, (LPARAM)line);
}

/* ===== 解析 ===== */

static uint16_t rd_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 追加一条记录到 g_recs. */
static void rec_add(uint8_t type, uint32_t ts, uint16_t en,
                    uint16_t di_val, const uint16_t ai_val[4])
{
	if (g_rec_count >= g_rec_cap) {
		int nc = g_rec_cap ? g_rec_cap * 2 : 4096;
		HistRec *nr = (HistRec *)realloc(g_recs, (size_t)nc * sizeof(HistRec));
		if (!nr) return;
		g_recs = nr;
		g_rec_cap = nc;
	}
	HistRec *r = &g_recs[g_rec_count++];
	r->type = type;
	r->ts = ts;
	r->en = en;
	r->di_val = di_val;
	if (ai_val) memcpy(r->ai_val, ai_val, sizeof(r->ai_val));
	else memset(r->ai_val, 0, sizeof(r->ai_val));
}

/* 解析原始缓冲: 顺序遍历记录, 遇到截断/未知类型即停.
 * 返回解析出的记录数; 更新 DI/AI 计数 (可为 NULL). */
static int parse_history(const uint8_t *buf, size_t size, int *out_di, int *out_ai)
{
	size_t off = 0;
	int di = 0, ai = 0;
	while (off + 2 <= size) {
		uint16_t type = rd_le16(buf + off);
		if (type == HIST_DI_TYPE) {
			if (off + HIST_DI_LEN > size) break;   /* 末段截断 */
			rec_add(HIST_DI_TYPE, rd_le32(buf + off + 2),
			        rd_le16(buf + off + 6), rd_le16(buf + off + 8), NULL);
			di++;
			off += HIST_DI_LEN;
		} else if (type == HIST_AI_TYPE) {
			if (off + HIST_AI_LEN > size) break;
			uint16_t ai_val[4];
			for (int i = 0; i < 4; i++) ai_val[i] = rd_le16(buf + off + 8 + i * 2);
			rec_add(HIST_AI_TYPE, rd_le32(buf + off + 2),
			        rd_le16(buf + off + 6), 0, ai_val);
			ai++;
			off += HIST_AI_LEN;
		} else {
			break;   /* 未知类型 / 文件末尾垃圾, 停止 */
		}
	}
	if (out_di) *out_di = di;
	if (out_ai) *out_ai = ai;
	return di + ai;
}

/* ===== 时间格式化 ===== */

static void fmt_time_w(uint32_t ts, wchar_t *out, int cap)
{
	time_t t = (time_t)ts;
	struct tm tmv;
	if (localtime_s(&tmv, &t) != 0) {
		swprintf(out, cap, L"-");
		return;
	}
	wcsftime(out, (size_t)cap, L"%Y-%m-%d %H:%M:%S", &tmv);
}

static void fmt_time_a(uint32_t ts, char *out, int cap)
{
	time_t t = (time_t)ts;
	struct tm tmv;
	if (localtime_s(&tmv, &t) != 0) {
		out[0] = '\0';
		return;
	}
	strftime(out, (size_t)cap, "%Y-%m-%d %H:%M:%S", &tmv);
}

static void fmt_bin16(uint16_t v, wchar_t *out)
{
	for (int i = 0; i < 16; i++) {
		out[i] = (v & (1u << (15 - i))) ? L'1' : L'0';
	}
	out[16] = L'\0';
}

/* ===== ListView (LVS_OWNERDATA 虚拟模式) ===== */

/* 生成第 row 行第 col 列的显示文本 (0=序号 1=时间 2=类型 3=值详情).
 * 仅在控件请求可见行时调用. */
static void rec_get_text(int row, int col, wchar_t *out, int cap)
{
	const HistRec *r = &g_recs[row];
	switch (col) {
	case 0:
		swprintf(out, cap, L"%d", row + 1);
		break;
	case 1:
		fmt_time_w(r->ts, out, cap);
		break;
	case 2:
		swprintf(out, cap, L"%ls", r->type == HIST_DI_TYPE ? L"DI" : L"AI");
		break;
	default:
		if (r->type == HIST_DI_TYPE) {
			wchar_t bin[17];
			fmt_bin16(r->di_val, bin);
			swprintf(out, cap, L"使能=0x%04X  值=0x%04X  DI1-16: %ls",
			         r->en, r->di_val, bin);
		} else {
			swprintf(out, cap,
			         L"AI1=%.2fmA  AI2=%.2fmA  AI3=%.2fV  AI4=%.2fV  (使能=0x%04X)",
			         r->ai_val[0] / 100.0, r->ai_val[1] / 100.0,
			         r->ai_val[2] / 100.0, r->ai_val[3] / 100.0, r->en);
		}
		break;
	}
}

/* 虚拟模式: 只需告知记录总数, 行文本由 LVN_GETDISPINFOW 按需生成,
 * 数万条记录也是瞬间完成, 滚动不卡. */
static void populate_listview(void)
{
	ListView_SetItemCountEx(g_his.hList, g_rec_count,
	                        LVSICF_NOINVALIDATEALL | LVSICF_NOSCROLL);
}

/* ===== 打开文件 / 解析 ===== */

static void on_open(void)
{
	wchar_t path[MAX_PATH] = {0};
	OPENFILENAMEW ofn;
	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = g_his.hSelf;
	ofn.lpstrFilter = L"历史记录 (*.raw)\0*.raw\0所有文件\0*.*\0";
	ofn.lpstrFile = path;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
	if (!GetOpenFileNameW(&ofn)) return;

	HANDLE hf = CreateFileW(path, GENERIC_READ, FILE_SHARE_READ, NULL,
	                        OPEN_EXISTING, 0, NULL);
	if (hf == INVALID_HANDLE_VALUE) {
		log_append(L"打开文件失败");
		return;
	}
	DWORD size = GetFileSize(hf, NULL);
	if (size == INVALID_FILE_SIZE || size == 0) {
		CloseHandle(hf);
		log_append(L"文件为空或读取大小失败");
		return;
	}
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf) {
		CloseHandle(hf);
		log_append(L"内存不足");
		return;
	}
	DWORD rd = 0;
	BOOL ok = ReadFile(hf, buf, size, &rd, NULL);
	CloseHandle(hf);
	if (!ok || rd != size) {
		free(buf);
		log_append(L"读取文件不完整");
		return;
	}

	/* 释放旧记录, 重新解析 */
	free(g_recs);
	g_recs = NULL;
	g_rec_count = 0;
	g_rec_cap = 0;
	int di = 0, ai = 0;
	int n = parse_history(buf, size, &di, &ai);
	free(buf);

	populate_listview();

	wchar_t info[160];
	swprintf(info, 160, L"%ls  记录 %d 条 (DI=%d, AI=%d)",
	         path, n, di, ai);
	SetWindowTextW(g_his.hInfo, info);

	wchar_t m[128];
	swprintf(m, 128, L"已解析 %d 条记录 (DI=%d, AI=%d)", n, di, ai);
	log_append(m);
}

/* ===== 导出 CSV ===== */

static void on_export_csv(void)
{
	if (g_rec_count == 0) {
		MessageBoxW(g_his.hSelf, L"请先打开并解析历史记录文件", L"提示",
		            MB_ICONWARNING);
		return;
	}
	wchar_t path[MAX_PATH] = {0};
	OPENFILENAMEW ofn;
	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = g_his.hSelf;
	ofn.lpstrFilter = L"CSV 文件 (*.csv)\0*.csv\0所有文件\0*.*\0";
	ofn.lpstrFile = path;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrDefExt = L"csv";
	ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
	if (!GetSaveFileNameW(&ofn)) return;

	FILE *fp = _wfopen(path, L"wb");
	if (!fp) {
		log_append(L"创建 CSV 文件失败");
		return;
	}
	fwrite("\xEF\xBB\xBF", 1, 3, fp);   /* UTF-8 BOM */
	fputs("时间,类型,DI使能,DI值,AI1,AI2,AI3,AI4\r\n", fp);
	char line[512];
	for (int i = 0; i < g_rec_count; i++) {
		const HistRec *r = &g_recs[i];
		char tstr[32];
		fmt_time_a(r->ts, tstr, sizeof(tstr));
		if (r->type == HIST_DI_TYPE) {
			snprintf(line, sizeof(line), "%s,DI,0x%04X,0x%04X,,,,\r\n",
			         tstr, r->en, r->di_val);
		} else {
			snprintf(line, sizeof(line), "%s,AI,,,%.2f,%.2f,%.2f,%.2f\r\n",
			         tstr,
			         r->ai_val[0] / 100.0, r->ai_val[1] / 100.0,
			         r->ai_val[2] / 100.0, r->ai_val[3] / 100.0);
		}
		fputs(line, fp);
	}
	fclose(fp);
	log_append(L"CSV 导出完成");
}

/* ===== WM_COMMAND ===== */

static void on_command(WPARAM wParam)
{
	WORD id = LOWORD(wParam);
	WORD code = HIWORD(wParam);
	if (code != BN_CLICKED) return;
	switch (id) {
	case IDC_HIST_OPEN:   on_open(); break;
	case IDC_HIST_EXPORT: on_export_csv(); break;
	}
}

/* ===== WM_CREATE: 控件 ===== */

static void create_controls(HWND hWnd)
{
	g_his.hSelf = hWnd;
	g_hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

	int gx = 12, gw = 776;

	/* 行1: 打开 + 导出 + 文件信息 */
	g_his.hOpen = create_button(L"打开文件...", gx + 12, 8, 90, 24, IDC_HIST_OPEN);
	g_his.hExport = create_button(L"导出 CSV", gx + 108, 8, 80, 24, IDC_HIST_EXPORT);
	g_his.hInfo = create_label(L"(未打开文件)", gx + 196, 12, 560, 14);

	/* 记录 ListView (虚拟模式: 数据在 g_recs, 控件按需取可见行文本) */
	g_his.hList = CreateWindowExW(0, WC_LISTVIEWW, L"",
		WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_SINGLESEL | LVS_SHOWSELALWAYS |
		WS_BORDER | LVS_OWNERDATA,
		gx + 12, 40, gw - 24, 480, hWnd,
		(HMENU)(INT_PTR)IDC_HIST_LIST, g_hInst, NULL);
	SendMessageW(g_his.hList, WM_SETFONT, (WPARAM)g_hFont, TRUE);
	ListView_SetExtendedListViewStyle(g_his.hList,
		LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);

	LVCOLUMNW col;
	memset(&col, 0, sizeof(col));
	col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;
	col.cx = 60;  col.pszText = (LPWSTR)L"序号";   col.iSubItem = 0;
	ListView_InsertColumn(g_his.hList, 0, &col);
	col.cx = 150; col.pszText = (LPWSTR)L"时间";   col.iSubItem = 1;
	ListView_InsertColumn(g_his.hList, 1, &col);
	col.cx = 50;  col.pszText = (LPWSTR)L"类型";   col.iSubItem = 2;
	ListView_InsertColumn(g_his.hList, 2, &col);
	col.cx = 480; col.pszText = (LPWSTR)L"值详情"; col.iSubItem = 3;
	ListView_InsertColumn(g_his.hList, 3, &col);

	/* 日志 */
	create_groupbox(L"操作日志", gx, 540, gw, 320);
	g_his.hLog = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
		WS_CHILD | WS_VISIBLE | ES_MULTILINE | ES_READONLY |
		ES_AUTOVSCROLL | WS_VSCROLL,
		gx + 12, 560, gw - 24, 292,
		hWnd, (HMENU)(INT_PTR)IDC_HIST_LOG, g_hInst, NULL);
	SendMessageW(g_his.hLog, WM_SETFONT, (WPARAM)g_hFont, TRUE);

	log_append(L"就绪. 请打开设备导出的历史记录文件 (.raw)");
}

/* ===== 窗口过程 ===== */

static LRESULT CALLBACK hist_wndproc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		g_hInst = ((LPCREATESTRUCT)lParam)->hInstance;
		create_controls(hWnd);
		return 0;
	case WM_COMMAND:
		on_command(wParam);
		return 0;
	case WM_NOTIFY: {
		NMHDR *hdr = (NMHDR *)lParam;
		if (hdr->hwndFrom != g_his.hList)
			break;
		if (hdr->code == LVN_GETDISPINFOW) {
			NMLVDISPINFOW *di = (NMLVDISPINFOW *)lParam;
			if ((di->item.mask & LVIF_TEXT) && di->item.cchTextMax > 0)
				rec_get_text(di->item.iItem, di->item.iSubItem,
				             di->item.pszText, di->item.cchTextMax);
			return 0;
		}
		if (hdr->code == LVN_ODFINDITEMW) {
			/* 键盘输入跳转: 在序号列做前缀匹配, 与普通模式行为一致 */
			NMLVFINDITEMW *fi = (NMLVFINDITEMW *)lParam;
			if (fi->lvfi.flags & (LVFI_STRING | LVFI_PARTIAL)) {
				wchar_t txt[16];
				size_t len = wcslen(fi->lvfi.psz);
				for (int i = 0; i < g_rec_count; i++) {
					rec_get_text(i, 0, txt, 16);
					if (_wcsnicmp(txt, fi->lvfi.psz, len) == 0)
						return i;
				}
			}
			return -1;
		}
		break;
	}
	case WM_SIZE:
		/* 控件保持固定位置 (与其它 tab 一致). */
		return 0;
	case WM_CTLCOLORDLG:
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	case WM_CTLCOLORSTATIC: {
		HDC hdc = (HDC)wParam;
		HWND hCtrl = (HWND)lParam;
		if (GetWindowLongPtrW(hCtrl, GWL_STYLE) & ES_READONLY) {
			SetBkMode(hdc, OPAQUE);
			SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
			SetBkColor(hdc, GetSysColor(COLOR_WINDOW));
			return (LRESULT)GetSysColorBrush(COLOR_WINDOW);
		}
		SetBkMode(hdc, TRANSPARENT);
		SetTextColor(hdc, GetSysColor(COLOR_WINDOWTEXT));
		return (LRESULT)GetSysColorBrush(COLOR_BTNFACE);
	}
	case WM_DESTROY:
		free(g_recs);
		g_recs = NULL;
		g_rec_count = 0;
		g_rec_cap = 0;
		return 0;
	}
	return DefWindowProcW(hWnd, msg, wParam, lParam);
}

/* ===== 公共 API ===== */

HWND HistoryTab_Create(HWND hParent, HINSTANCE hInst)
{
	g_hInst = hInst;

	if (!g_classRegistered) {
		WNDCLASSW wc = {0};
		wc.lpfnWndProc = hist_wndproc;
		wc.hInstance = hInst;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
		wc.lpszClassName = HISTORY_TAB_CLASS;
		RegisterClassW(&wc);
		g_classRegistered = TRUE;
	}

	HWND h = CreateWindowExW(0, HISTORY_TAB_CLASS, L"",
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
		0, 0, 700, 500, hParent, NULL, hInst, NULL);
	return h;
}
