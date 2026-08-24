# io-edge-hub: C (FreeRTOS) 与 Rust (embassy) 固件对比

同一块 LCKFB STM32F407VET6 + W5500 + W25Q128 硬件,同一 e2e 套件
(93 项,`tests/e2e`),同一上位机 (Windows, Python) 实测。日期: 2026-08-23。

## 功能对等性

| 套件 | 项数 | C | Rust |
|---|---|---|---|
| basic | 9 | 通过 | 通过 |
| udp (8600) | 8 | 通过 | 通过 |
| modbus_tcp | 14 | 通过 | 通过 |
| modbus_rtu (COM10) | 3 | 通过 | 通过 |
| web (:80) | 17 | 通过 | 通过 |
| websocket | 3 | 通过 | 通过 |
| history (littlefs) | 5 | 通过 | 通过 |
| uart shell | 6 | 通过 | 通过 |
| ftp (:21, 3 并发) | 14 | 通过 | 通过 |
| stress (含 3 并发 FTP/1MB/混合负载 30s) | 10 | 通过 | 通过 |
| reboot | 3 | 通过 | 通过 |
| fw_upgrade (含真实 MCUboot 换机 ×2) | 3 | 通过 | 通过 |
| **合计** | **93** | **93** | **93** |

- Rust 全量 93 项单次 pytest 运行全绿(5m18s)。
- 升级通道互通: Rust 固件可通过自身 UDP v2 / WS / CAN 通道刷写, 也可刷回 C。
- CAN: 0x101-0x105 升级协议已移植; PCAN→设备方向实测收帧正常,
  设备→PCAN 方向 C/Rust 两版固件均无波形到达(物理层待排查, 非固件回归)。
- 盘上兼容: Rust 直接挂载 C 固件写过的 littlefs 分区并续写同一 history 文件。

## 性能实测

| 指标 | C | Rust | 备注 |
|---|---|---|---|
| 签名镜像体积 | 329,732 B | 205,708 B | Rust -38%(无 mbedtls/RSA) |
| FTP STOR 1 MiB (单连接) | 42 KB/s | 14 KB/s | Rust 页编程等待路径有 ~3x 优化空间 |
| FTP RETR 1 MiB (单连接) | 318 KB/s | 365 KB/s | NOR 读瓶颈, Rust 略快 |
| 3 客户端并行 STOR 128 KiB ×3 | ~11 s | ~11 s | 持平(均 NOR 写瓶颈) |
| 固件推送 UDP v2 (1400 B 窗口) | 56 KB/s | 42 KB/s | 同数量级 |
| MCUboot swap + 重启离线窗口 | 24 s | 21 s | 同数量级 |
| 历史记录 (DI 10 B @10 Hz) | 写缓冲, 事件级 sync | 同 C | 逐条 sync 会造成 NOR 擦除风暴(已修复为 C 语义) |

Rust FTP STOR 慢的已知原因: `w25q::wait_not_busy` 每页编程后固定 ~1 ms
轮询间隔, 4 Ki 页写入被放大到 ~18 ms/页; C 的轮询更紧。后续可改为
先短自旋再延迟, 预期可追平 C (不影响 e2e, 全部超时余量充足)。

## 可靠性结论

Rust 版在以下方面与 C 版行为一致(均为本次移植中实测修复):

1. littlefs 语义: 历史记录缓冲写 + 事件级 flush(C 不逐条 sync);
   逐条 sync 会以 ~8 条/次的频率重写整个 inline 文件标签并触发目录
   压实, 形成 NOR 擦除风暴并饿死存储任务(故障现象: 任意 FTP/存储
   操作超时, IWDG 复位循环)。
2. FTP/Modbus 会话上限拒绝器: 只在满载窗口内监听, 空闲 50 ms 内撤
   防; 常驻监听会截走下一个合法连接(C 语义)。
3. FTP 槽位文件句柄: 传输异常终止时由下一次同槽位 open 前统一关闭
   (dl+wr 双向), 防止 littlefs mlist 重入自环。
4. NOR I/O 不在临界区内执行: 擦除/编程 10-100 ms 级, 关中断会丢
   UART/W5500/升级窗口数据。

## 构建产物

| | C | Rust |
|---|---|---|
| 工具链 | arm-none-eabi-gcc + Makefile + FreeRTOS | rustup stable, thumbv7em-none-eabihf, embassy 0.6 |
| 构建命令 | make | `cargo build --release` + `python tools\sign.py` |
| 烧写 | 同 (probe-rs / ST-LINK @ 0x08010000) | 同 |
| 签名/引导 | imgtool RSA-2048 + MCUboot SWAP_USING_SCRATCH (不变) | 复用同一 MCUboot + 同一密钥 |

## 已知差异 / 后续

- Rust FTP STOR 吞吐 (优化项, 见上)。
- CAN 设备→PCAN 方向两版固件均不通, 待台架检查收发器/终端电阻/接线。
- Zephyr 版三方互刷验证未做(仓库无 Zephyr 构建产物)。
