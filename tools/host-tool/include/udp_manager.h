#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

/* winsock2 必须在 windows.h 之前, 否则 winsock1 冲突 */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdint.h>
#include <stdbool.h>

#define IOEDGE_UDP_PORT        8600   /* 固件配置/UDP升级端口 */
#define IOEDGE_UDP_REPLY_PORT  8601   /* GET_IP 跨网段回复端口 */
#define IOEDGE_UDP_TIMEOUT_MS  1000   /* 单条命令同步等待超时 */

/* opaque 句柄 */
typedef struct UdpManager UdpManager;

UdpManager *UdpManager_Create(void);
void UdpManager_Destroy(UdpManager *m);
const char *UdpManager_GetLastError(UdpManager *m);

/* --- 升级命令 (0x01-0x03, 小端; 0x04/0x05 同端口) --- */
/* FwStart: 发 [0x01][size LE32][keyhash 32B 可选], 回 [0x01][status][v2_chunk LE16 可选].
 * keyhash=NULL 时不带 (兼容旧设备); status: 0=失败 1=成功 2=keyhash 不匹配.
 * out_v2_chunk: 新固件回复携带 DATA_V2 单帧最大数据量 (协商值, 可为 NULL 忽略);
 * 老固件回复无该字段 → 填 0, 调用方应回退停等 FW_DATA 模式. */
bool UdpManager_FwStart(UdpManager *m, const char *ip, uint32_t img_size,
                        const uint8_t keyhash[32], uint8_t *out_status,
                        uint16_t *out_v2_chunk);
/* FwData: 发 [0x02][data<=511B], 回 [0x02][offset LE32]. */
bool UdpManager_FwData(UdpManager *m, const char *ip, const uint8_t *data, int len,
                       uint32_t *out_offset);
/* FwEnd: 发 [0x03][test 1B][crc16 LE16], 回 [0x03][result 1B]. */
bool UdpManager_FwEnd(UdpManager *m, const char *ip, uint8_t test, uint16_t crc16,
                      uint8_t *out_result);

/* FW_DATA_V2 (0x06) 窗口流水线进度/取消回调 */
typedef void (*UdpProgressFn)(uint32_t offset, void *user_data);
typedef bool (*UdpCancelFn)(void *user_data);
/* FwDataV2Stream: 每帧 [0x06][offset LE32][data<=chunk], 连发 8 帧不等回复,
 * 按回复中的期望 offset go-back-N 重传 (设备端按 offset 去重).
 * chunk 取 FwStart 协商值 (<=1400). progress 每窗口回调已确认字节数;
 * cancel 返回 true 时中止. 返回 false 时错误见 GetLastError. */
bool UdpManager_FwDataV2Stream(UdpManager *m, const char *ip,
                               const uint8_t *data, uint32_t total, int chunk,
                               UdpProgressFn progress, void *user_data,
                               UdpCancelFn cancel);

/* --- 配置命令 (0x10+, 大端) --- */
bool UdpManager_SetIp(UdpManager *m, const char *ip, uint8_t ip4[4], uint8_t *out_ok);  /* 0x10 */
bool UdpManager_GetIp(UdpManager *m, const char *ip, uint8_t ip4[4]);                    /* 0x11 */
bool UdpManager_SetModbus(UdpManager *m, const char *ip, uint8_t slave_id,
                          uint16_t baud, uint8_t *out_ok);                               /* 0x12 */
bool UdpManager_GetModbus(UdpManager *m, const char *ip, uint8_t *out_slave,
                          uint16_t *out_baud);                                           /* 0x13 */
bool UdpManager_SetTime(UdpManager *m, const char *ip, uint32_t unix_ts, uint8_t *out_ok); /* 0x14 */
/* GET_IP (0x11, broadcast-allowed): 向所有本机网卡子网定向广播发送, 单播+8601 监听回复.
 * out 一次性填所有回复: "a.b.c.d" 一行一条, '\n' 分隔.
 * out_cap 为 out 缓冲字节. 返回 true=至少发现 1 台. */
bool UdpManager_Discover(UdpManager *m, char *out, int out_cap, int *out_count);
bool UdpManager_FactoryReset(UdpManager *m, const char *ip, uint8_t *out_ok);            /* 0x19 */
bool UdpManager_GetVersion(UdpManager *m, const char *ip, char *out_ver, int out_cap);   /* 0x04 */
bool UdpManager_Reboot(UdpManager *m, const char *ip);                                   /* 0x05 */

/* CRC16-CCITT (poly 0x1021, init 0x0000), 与 Zephyr crc16_ccitt 对齐 (UDP 升级用). */
uint16_t UdpManager_CRC16_CCITT(const uint8_t *data, size_t len);

#endif /* UDP_MANAGER_H */
