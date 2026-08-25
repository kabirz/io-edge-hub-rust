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
#include <string.h>
#include "udp_manager.h"
#include "fw_image.h"
#include "can_manager.h"
#include "pcan_loader.h"

/* 进度回调: ud 传总长度 */
static void prog(uint32_t off, void *ud)
{
    uint32_t total = (uint32_t)(uintptr_t)ud;
    printf("  DATA_V2 %u/%u\r", off, total);
    fflush(stdout);
}

/* CAN 通道自测: 探测 PCAN → 连接 → GET_VERSION + GET_KEYHASH(cmd=4)。
 * 用法: protocol_test.exe --can [bitrate_kbps] (默认 250) */
static int can_probe(const char *baud_arg)
{
    uint32_t bitrate = PCAN_BAUD_250K;
    if (baud_arg) {
        int kbps = atoi(baud_arg);
        if (kbps == 500) bitrate = PCAN_BAUD_500K;
        else if (kbps == 125) bitrate = PCAN_BAUD_125K;
        else if (kbps == 100) bitrate = PCAN_BAUD_100K;
        else if (kbps == 50) bitrate = PCAN_BAUD_50K;
        else if (kbps == 1000) bitrate = PCAN_BAUD_1M;
    }
    CanManager *can = CanManager_Create();
    char names[16][32];
    int channels[16];
    int cnt = CanManager_DetectDevices(can, names, channels, 16);
    if (cnt <= 0) {
        printf("PCAN detect: none (%s)\n", CanManager_GetLastError(can));
        return 1;
    }
    printf("PCAN detect: %d device(s), using %s @ %uk\n",
           cnt, names[0], baud_arg ? (unsigned)atoi(baud_arg) : 250u);
    if (!CanManager_Connect(can, channels[0], bitrate)) {
        printf("PCAN connect failed: %s\n", CanManager_GetLastError(can));
        return 1;
    }
    char ver[80] = {0};
    if (CanManager_GetVersion(can, ver, sizeof(ver))) {
        printf("CAN GET_VERSION: %s\n", ver);
    } else {
        printf("CAN GET_VERSION failed: %s\n", CanManager_GetLastError(can));
    }
    uint8_t kh[32];
    if (CanManager_GetKeyhash(can, kh)) {
        printf("CAN GET_KEYHASH (0x101 cmd=4): ");
        for (int i = 0; i < 32; i++) printf("%02x", kh[i]);
        printf("\n");
        const uint8_t *baked = fw_image_keyhash();
        printf("matches baked constant: %s\n",
               memcmp(kh, baked, 32) == 0 ? "YES" : "NO (rotated key?)");
        CanManager_Disconnect(can);
        return 0;
    }
    printf("CAN GET_KEYHASH failed: %s\n", CanManager_GetLastError(can));
    CanManager_Disconnect(can);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--can") == 0) {
        return can_probe(argc >= 3 ? argv[2] : NULL);
    }
    if (argc < 3) {
        printf("usage: %s <ip> <app.dfu.bin>\n"
               "       %s --can [bitrate_kbps]   (PCAN detect + keyhash)\n",
               argv[0], argv[0]);
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

    /* keyhash 设备自报 (UDP 0x15), 失败退回内置常量 */
    uint8_t kh[32];
    const uint8_t *keyhash = fw_image_keyhash();
    if (UdpManager_GetKeyhash(m, ip, kh)) {
        keyhash = kh;
        printf("GET_KEYHASH: %02x%02x%02x%02x... (device)\n", kh[0], kh[1], kh[2], kh[3]);
    } else {
        printf("GET_KEYHASH: no reply, baked constant (%s)\n", UdpManager_GetLastError(m));
    }

    /* START (设备自报 keyhash) */
    DWORD t0 = GetTickCount();
    uint8_t status = 0;
    uint16_t chunk = 0;
    if (!UdpManager_FwStart(m, ip, (uint32_t)sz, keyhash, &status, &chunk)) {
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
