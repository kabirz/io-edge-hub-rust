#ifndef PCAN_LOADER_H
#define PCAN_LOADER_H

#include <windows.h>
#include <stdint.h>
#include <stdbool.h>

/* PCAN 通道句柄与状态码类型 */
typedef uint32_t TPCANHandle;
typedef uint32_t TPCANStatus;

/* PCAN 状态码 (仅列举本工具用到的) */
#define PCAN_ERROR_OK           0x00000
#define PCAN_ERROR_XMTFULL      0x00001
#define PCAN_ERROR_OVERRUN      0x00002
#define PCAN_ERROR_BUSLIGHT     0x00004
#define PCAN_ERROR_BUSPASSIVE   0x00400
#define PCAN_ERROR_BUSOFF       0x00008

/* 默认通道句柄, 表示未占用 */
#define PCAN_NONEBUS            0x00U

/* PCAN 波特率 (BTR0BTR1 寄存器值) */
#define PCAN_BAUD_1M            0x0014
#define PCAN_BAUD_500K          0x001C
#define PCAN_BAUD_250K          0x011C
#define PCAN_BAUD_125K          0x031C
#define PCAN_BAUD_100K          0x432F
#define PCAN_BAUD_95K           0xC34E
#define PCAN_BAUD_83K           0x852B
#define PCAN_BAUD_50K           0x472F

/* 通道信息结构 (CAN_GetAttachedChannels 用) */
typedef struct {
	uint32_t channel;
	uint32_t channel_condition;
} TPCANChannelInformation;

/* CAN 消息结构 */
typedef struct {
	uint32_t id;
	uint8_t msgtype;
	uint8_t len;
	uint8_t data[8];
	uint64_t timestamp;
} TPCANMsg;

/* 带时间戳的消息封装 (CAN_Read 用) */
typedef struct {
	TPCANMsg msg;
	uint64_t timestamp;
} TPCANTimestampMsg;

/* PCANBasic API 函数指针类型 (__stdcall 调用约定) */
typedef TPCANStatus (__stdcall *pfnCAN_Initialize)(uint32_t channel, uint32_t baudrate,
                     uint32_t hwType, uint32_t ioPort,
                     uint16_t interrupt);
typedef TPCANStatus (__stdcall *pfnCAN_Uninitialize)(uint32_t channel);
typedef TPCANStatus (__stdcall *pfnCAN_Read)(uint32_t channel, TPCANMsg *msg,
                      TPCANTimestampMsg *timestamp);
typedef TPCANStatus (__stdcall *pfnCAN_Write)(uint32_t channel, TPCANMsg *msg);
typedef TPCANStatus (__stdcall *pfnCAN_FilterMessages)(uint32_t channel, uint32_t fromID,
                        uint32_t toID, uint8_t mode);
typedef TPCANStatus (__stdcall *pfnCAN_LookUpChannel)(char *szLookup, TPCANHandle *channel);
typedef TPCANStatus (__stdcall *pfnCAN_GetErrorText)(TPCANStatus error, uint16_t language,
                        char *buffer);

/* 动态加载/卸载 PCANBasic.dll */
bool PcanLoader_Load(void);
void PcanLoader_Unload(void);

/* PCANBasic API 函数指针 (加载后非 NULL 即可用) */
extern pfnCAN_Initialize       Pcan_Initialize;
extern pfnCAN_Uninitialize     Pcan_Uninitialize;
extern pfnCAN_Read             Pcan_Read;
extern pfnCAN_Write            Pcan_Write;
extern pfnCAN_FilterMessages   Pcan_FilterMessages;
extern pfnCAN_LookUpChannel    Pcan_LookUpChannel;
extern pfnCAN_GetErrorText     Pcan_GetErrorText;

#endif /* PCAN_LOADER_H */
