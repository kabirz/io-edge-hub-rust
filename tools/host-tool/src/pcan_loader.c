/*
 * PCANBasic.dll 动态加载器
 * 运行时按需加载 PCAN 驱动 DLL, 避免编译期硬依赖
 */
#include "pcan_loader.h"
#include <stdio.h>

/* PCANBasic API 函数指针 (加载后指向 DLL 导出符号) */
pfnCAN_Initialize       Pcan_Initialize = NULL;
pfnCAN_Uninitialize     Pcan_Uninitialize = NULL;
pfnCAN_Read             Pcan_Read = NULL;
pfnCAN_Write            Pcan_Write = NULL;
pfnCAN_FilterMessages   Pcan_FilterMessages = NULL;
pfnCAN_LookUpChannel    Pcan_LookUpChannel = NULL;
pfnCAN_GetErrorText     Pcan_GetErrorText = NULL;

static HMODULE g_hModule = NULL;

/* 符号名 -> 函数指针地址 的映射表, 表驱动加载避免重复代码 */
static struct {
	const char *name;
	void **ptr;
} g_procs[] = {
	{ "CAN_Initialize",     (void **)&Pcan_Initialize },
	{ "CAN_Uninitialize",   (void **)&Pcan_Uninitialize },
	{ "CAN_Read",           (void **)&Pcan_Read },
	{ "CAN_Write",          (void **)&Pcan_Write },
	{ "CAN_FilterMessages", (void **)&Pcan_FilterMessages },
	{ "CAN_LookUpChannel",  (void **)&Pcan_LookUpChannel },
	{ "CAN_GetErrorText",   (void **)&Pcan_GetErrorText },
	{ NULL, NULL }
};

bool PcanLoader_Load(void)
{
	if (g_hModule) {
		return true;
	}

	g_hModule = LoadLibraryW(L"PCANBasic.dll");
	if (!g_hModule) {
		return false;
	}

	for (int i = 0; g_procs[i].name; i++) {
		*g_procs[i].ptr = GetProcAddress(g_hModule, g_procs[i].name);
		if (!*g_procs[i].ptr) {
			PcanLoader_Unload();
			return false;
		}
	}

	return true;
}

void PcanLoader_Unload(void)
{
	if (g_hModule) {
		FreeLibrary(g_hModule);
		g_hModule = NULL;
	}
	for (int i = 0; g_procs[i].name; i++) {
		*g_procs[i].ptr = NULL;
	}
}
