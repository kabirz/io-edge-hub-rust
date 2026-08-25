#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <windows.h>
#include <stdint.h>
#include <stdbool.h>

/* io-edge-hub CAN 固件升级帧 ID (固件 libs/can_fw_upgrade/can_fw_upgrade.c) */
#define CAN_ID_IO_CMD      0x101   /* 主→设 命令: data_32[0]=cmd, data_32[1]=arg (LE32) */
#define CAN_ID_IO_RESP     0x102   /* 设→主 回复: data_32[0]=code, data_32[1]=offset (LE32) */
#define CAN_ID_IO_DATA     0x103   /* 主→设 固件数据 (≤8B 原始) */
#define CAN_ID_IO_KEYHASH  0x104   /* 主→设 keyhash 分片: data[0]=seq, data[1..7]=7B */
#define CAN_ID_IO_VERSION  0x105   /* 设→主 版本分片: data[0]=seq, data[1..7]=ASCII */
#define CAN_ID_IO_BOOT_PROBE 0x106 /* 设→主 MCUboot 探测: [0..3]="BTO1", [4..6]=vM.m.p */
#define CAN_ID_IO_BOOT_ACK   0x107 /* 主→设 探测应答 (任意 1B) */

/* MCUboot 启动探测帧 magic ("BTO1", LE32) */
#define IO_FW_BOOT_PROBE_MAGIC  0x42544F31u

#define CAN_IO_DEFAULT_BITRATE  250000

/* 固件升级命令码 (0x101 data_32[0]) */
enum io_fw_cmd {
	IO_FW_CMD_START_UPDATE = 0,
	IO_FW_CMD_CONFIRM,
	IO_FW_CMD_VERSION,
	IO_FW_CMD_REBOOT,
	IO_FW_CMD_KEYHASH,            /* 设备自报 keyhash (embassy-boot 固件) */
};

/* 固件升级回复码 (0x102 data_32[0]) */
enum io_fw_code {
	IO_FW_CODE_OFFSET = 0,        /* 流控: 已写入 offset 字节 */
	IO_FW_CODE_UPDATE_SUCCESS,    /* 数据全部写完 */
	IO_FW_CODE_VERSION,           /* 版本查询: arg=字符串总长, 后续跟 0x105 分片 */
	IO_FW_CODE_CONFIRM,           /* 确认成功: arg=0x55AA55AA */
	IO_FW_CODE_FLASH_ERROR,
	IO_FW_CODE_TRANSFER_ERROR,
	IO_FW_CODE_KEYHASH_ERROR,
	IO_FW_CODE_KEYHASH,           /* keyhash 查询: arg=32, 后续 5 帧 0x105 分片 */
};

/* CONFIRM 成功标志 (0x102 data_32[1]) */
#define IO_FW_CONFIRM_MAGIC  0x55AA55AAu

/* 不透明句柄 */
typedef struct CanManager CanManager;

/* 进度回调: percent 0-100, user 透传 */
typedef void (*can_progress_cb)(int percent, void *user);

/* 生命周期 */
CanManager *CanManager_Create(void);
void CanManager_Destroy(CanManager *m);
const char *CanManager_GetLastError(CanManager *m);

/* 设备探测: 枚举系统内所有 PCAN-USB 通道 (Pcan_LookUpChannel).
 * out_names[i] 填 "PCAN-USB: %02Xh" (名称格式与 handler-receiver 一致),
 * out_channels[i] 填通道句柄 (Connect 用). 返回设备数 (0..16). */
int CanManager_DetectDevices(CanManager *m, char out_names[][32], int out_channels[], int max);

/* 连接指定通道 (channel=PCAN 通道句柄, 0-based; bitrate 如 250000).
 * 失败 last_error 填 PCAN 状态码. */
bool CanManager_Connect(CanManager *m, int channel, uint32_t bitrate);
void CanManager_Disconnect(CanManager *m);
bool CanManager_IsConnected(const CanManager *m);

/* 完整升级流程 (阻塞, 调用方在 worker 线程调):
 * 1. keyhash!=NULL → 发 5 帧 0x104 (1B seq + 7B chunk)
 * 2. START (0x101 cmd=0, arg=size) → 等 OFFSET(0)/FLASH_ERROR(4)/KEYHASH_ERROR(6)
 * 3. 流式 0x103 (8B/帧), 每 64B 设备回 OFFSET 做流控, 总量满回 UPDATE_SUCCESS(1)
 * 4. CONFIRM (0x101 cmd=1, arg=permanent?1:0) → 等 CONFIRM(3, arg=0x55AA55AA)/TRANSFER_ERROR(5)
 * progress 回调 0-100, user 透传 */
bool CanManager_FirmwareUpgrade(CanManager *m, const uint8_t *img, uint32_t size,
                                const uint8_t keyhash[32], bool permanent,
                                can_progress_cb progress, void *user);

/* 进入 MCUboot bootloader 紧急救援模式 (阻塞, 调用方在 worker 线程调):
 * 1. 尽力发一次 REBOOT (0x101 cmd=3): 设备软死机/正常运行时可靠, 硬死机无效
 * 2. 60s 内持续轮询 0x106 探测帧 (data[0..3]="BTO1", data[4..6]=vM.m.p);
 *    设备死机时需用户手动断电/复位重启, 主机全程监听不会错过 bootloader
 *    的 ~500ms 探测窗口, 调用方应在此之前提示用户手动重启
 * 3. 回 0x107 应答 (1B), 设备随即进入 ~15s 固件升级等待窗口
 * 之后调 FirmwareUpgrade 即可 (keyhash/START/DATA/CONFIRM 协议在 app 与
 * bootloader 共用); bootloader 模式数据写 slot0, CONFIRM 后 MCUboot 直接
 * 验证并启动新固件 (无 swap, 无需重启).
 * 成功返回 true. */
bool CanManager_EnterBoot(CanManager *m);

/* 查询版本字符串 (0x101 cmd=2 → 0x102 code=2 + 0x105 分片拼接).
 * 成功 true, out_ver 填 NUL 终止串. */
bool CanManager_GetVersion(CanManager *m, char *out_ver, int out_cap);

/* 查询设备 keyhash (0x101 cmd=4 → 0x102 code=7,arg=32 + 5 帧 0x105 分片,
 * 帧 [seq][≤7B 原始字节] 定位 seq*7)。升级前向设备自报取指纹 —— 换签名
 * 钥匙只改固件, 本工具无需跟改。成功 true, out_keyhash 填 32B。 */
bool CanManager_GetKeyhash(CanManager *m, uint8_t out_keyhash[32]);

/* 重启 (0x101 cmd=3), 设备收到即重启, 回复不可靠 → 不强求回复. */
bool CanManager_Reboot(CanManager *m);

#endif /* CAN_MANAGER_H */
