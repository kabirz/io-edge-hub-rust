#ifndef MODBUS_CLIENT_H
#define MODBUS_CLIENT_H

/* winsock2 必须在 windows.h 之前, 否则 winsock1 冲突 (TCP 路径需要 winsock,
 * RTU 路径需要 windows.h 的 CreateFileW/DCB/COMMTIMEOUTS). */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum { MB_TCP, MB_RTU } MbTransport;

typedef struct MbClient MbClient;

MbClient *MbClient_Create(void);
void MbClient_Destroy(MbClient *m);
const char *MbClient_GetLastError(MbClient *m);
bool MbClient_IsConnected(const MbClient *m);
/* 上一次失败的事务是否为 "对端完全无响应" (等待响应超时, 未收到任何字节).
 * 用于调用方区分 "设备不说话" (重试/逐个读只会逐个超时, 应提前放弃) 与
 * "设备有应答但拒绝/校验错" (回退逐个读仍可能成功). */
bool MbClient_LastNoResponse(const MbClient *m);
MbTransport MbClient_GetTransport(const MbClient *m);

/* TCP: 连 ip:port (默认 502), 后续读写用 unit_id */
bool MbClient_ConnectTcp(MbClient *m, const char *ip, uint16_t port, uint8_t unit_id);
/* RTU: 打开串口 (如 L"\\\\.\\COM3"), 8N1, baud */
bool MbClient_ConnectRtu(MbClient *m, const wchar_t *com_port, uint32_t baud, uint8_t unit_id);
void MbClient_Disconnect(MbClient *m);

/* FC01 读线圈 (DO). out_bits: 每元素 0/1, 调用者按 qty 分配. */
bool MbClient_ReadCoils(MbClient *m, uint16_t addr, uint16_t qty, uint8_t *out_bits);
/* FC02 读离散输入 (DI) */
bool MbClient_ReadDiscreteInputs(MbClient *m, uint16_t addr, uint16_t qty, uint8_t *out_bits);
/* FC03 读保持寄存器. out_regs: 调用者按 qty 分配 uint16_t[] */
bool MbClient_ReadHolding(MbClient *m, uint16_t addr, uint16_t qty, uint16_t *out_regs);
/* FC04 读输入寄存器 (AI) */
bool MbClient_ReadInput(MbClient *m, uint16_t addr, uint16_t qty, uint16_t *out_regs);
/* FC05 写单线圈 */
bool MbClient_WriteSingleCoil(MbClient *m, uint16_t addr, bool on);
/* FC06 写单保持寄存器 */
bool MbClient_WriteSingleReg(MbClient *m, uint16_t addr, uint16_t value);
/* FC16 写多保持寄存器 */
bool MbClient_WriteMultiReg(MbClient *m, uint16_t addr, uint16_t qty, const uint16_t *values);

#endif /* MODBUS_CLIENT_H */
