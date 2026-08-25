/* tools/host-tool 协议层联机自测 (控制台).
 *
 * 直接链接本工具的 src/udp_manager.c + src/fw_image.c —— 验证的正是
 * 上位机 exe 实际使用的协议代码 (GUI 无法自动化, 这里覆盖同一调用序列):
 *
 *   GET_VERSION -> FW_START(常量 keyhash) -> FW_DATA_V2 窗口流 ->
 *   FW_END(CRC, 设备端 ed25519 验签) -> REBOOT -> 轮询 GET_VERSION 等换机完成
 *
 * 用法: protocol_test.exe <ip> <app.dfu.bin>
 * 退出码: 0=全流程成功; 1=失败 (错误信息打到 stdout).
 * 注意: 会触发一次真实换机 (同镜像升级, 新镜像跑通 main 自动确认, 安全).
 */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "udp_manager.h"
#include "fw_image.h"

/* 进度回调: ud 传总长度 */
static void prog(uint32_t off, void *ud)
{
    uint32_t total = (uint32_t)(uintptr_t)ud;
    printf("  DATA_V2 %u/%u\r", off, total);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <ip> <app.dfu.bin>\n", argv[0]);
        return 1;
    }
    const char *ip = argv[1];

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    /* 读载荷 */
    FILE *f = fopen(argv[2], "rb");
    if (!f) { printf("open %s failed\n", argv[2]); return 1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *img = (uint8_t *)malloc((size_t)sz);
    if (fread(img, 1, (size_t)sz, f) != (size_t)sz) { printf("read failed\n"); return 1; }
    fclose(f);
    printf("payload: %ld B\n", sz);

    if (!fw_image_validate_header(img, (size_t)sz)) {
        printf("fw_image_validate_header REJECTED (len)\n");
        return 1;
    }
    if (fw_image_is_mcuboot(img, (size_t)sz)) {
        printf("fw_image_is_mcuboot REJECTED (old MCUboot image)\n");
        return 1;
    }
    printf("fw_image: payload ok, keyhash %02x%02x%02x%02x...\n",
           fw_image_keyhash()[0], fw_image_keyhash()[1],
           fw_image_keyhash()[2], fw_image_keyhash()[3]);

    UdpManager *m = UdpManager_Create();
    char ver[64] = {0};
    if (!UdpManager_GetVersion(m, ip, ver, sizeof(ver))) {
        printf("GET_VERSION failed: %s\n", UdpManager_GetLastError(m));
        return 1;
    }
    printf("GET_VERSION: %s\n", ver);

    /* START (keyhash 常量) */
    DWORD t0 = GetTickCount();
    uint8_t status = 0;
    uint16_t chunk = 0;
    if (!UdpManager_FwStart(m, ip, (uint32_t)sz, fw_image_keyhash(), &status, &chunk)) {
        printf("FW_START no reply: %s\n", UdpManager_GetLastError(m));
        return 1;
    }
    printf("FW_START: status=%u v2_chunk=%u (erase %lums)\n",
           status, chunk, GetTickCount() - t0);
    if (status != 1) { printf("FW_START rejected\n"); return 1; }

    /* DATA_V2 窗口流 */
    t0 = GetTickCount();
    if (!UdpManager_FwDataV2Stream(m, ip, img, (uint32_t)sz, chunk > 1400 ? 1400 : chunk,
                                   prog, (void *)(uintptr_t)sz, NULL)) {
        printf("\nFW_DATA_V2 failed: %s\n", UdpManager_GetLastError(m));
        return 1;
    }
    printf("\nFW_DATA_V2 done in %lums\n", GetTickCount() - t0);

    /* END (设备端 CRC 回读 + ed25519 验签) */
    t0 = GetTickCount();
    uint8_t result = 0;
    uint16_t crc = UdpManager_CRC16_CCITT(img, (size_t)sz);
    if (!UdpManager_FwEnd(m, ip, 0, crc, &result)) {
        printf("FW_END no reply: %s\n", UdpManager_GetLastError(m));
        return 1;
    }
    printf("FW_END: ok=%u (verify %lums)\n", result, GetTickCount() - t0);
    if (result != 1) { printf("FW_END rejected (crc/signature)\n"); return 1; }

    /* REBOOT + 等换机完成 */
    UdpManager_Reboot(m, ip);
    printf("REBOOT sent, waiting for the embassy-boot swap (~30-40s)...\n");
    t0 = GetTickCount();
    for (;;) {
        Sleep(2000);
        char v2[64] = {0};
        if (UdpManager_GetVersion(m, ip, v2, sizeof(v2))) {
            printf("ONLINE after %lums: %s\n", GetTickCount() - t0, v2);
            UdpManager_Destroy(m);
            WSACleanup();
            printf("PASS\n");
            return 0;
        }
        if (GetTickCount() - t0 > 120000) {
            printf("device not back within 120s\n");
            return 1;
        }
    }
}
