# io-edge-hub-rust

io-edge-hub 边缘采集网关固件的 **Rust / embassy 重写版**,与 C(FreeRTOS)版本行为逐级对齐:
同一套 NOR 数据分区布局(配置/littlefs 盘上兼容,C 版写的文件本版可直接挂载读写),
93 项 e2e 测试(直接复用 C 仓库套件)全绿。
引导器为 **embassy-boot**(Rust 原生,`feat/embassy-boot` 分支起替换 MCUboot):
boot/ACTIVE 在片内 flash,DFU/STATE 在外置 W25Q,试用启动 + 自动回滚。

- 参考固件(C/FreeRTOS):`C:\Users\jxwaz\code\io-edge-hub-freertos`
- 协议权威源(Zephyr):`C:\Users\jxwaz\code\app\apps`(`libs/can_fw_upgrade` 等)
- 硬件:LCKFB STM32F407VET6 探索板 + W5500(SPI2) + W25Q128 NOR(SPI1)

## 功能总览(与 C 版对齐验收)

| 功能 | 说明 |
|---|---|
| 网络 | W5500 MACRAW → embassy-net(smoltcp),静态 IP(默认 192.168.12.101),MAC 由 UID 派生 |
| UDP :8600 | 全部配置命令(0x01-0x14/0x19)、v2 大块升级通道(1400B go-back-N)、广播 8601 |
| Modbus TCP :502 | FC01-08/15/16,最多 2 主站,第 3 个连接 accept 后复位 |
| Modbus RTU | USART2 + DE(PA1),9600-可配,t3.5 `read_until_idle` 判帧 |
| HTTP :80 | gzip SPA、JSON API、POST 命令、keep-alive/pipelining、2 连接上限 |
| WebSocket | `/ws` 单会话,1s io/regs + 10s info 推送,固件升级二进制通道 |
| FTP :21 | RFC 959:PASV/EPSV/PORT/EPRT、TYPE A/I、REST、APPE、3 会话 + 421 拒绝 |
| 历史 | `data_MMDD_HHMMSS.raw`,DI 10B / AI 16B 记录,1MB×10 轮转,断电续写同名 |
| 存储 | W25Q128:IOCF A/B 配置(32K×2)+ littlefs(0xF0000 起,盘上兼容 C 版) |
| CAN 升级 | 0x101-0x105 协议(Zephyr 权威语义:64B 流控),全速 ~16s/206KB,配置 can_id/can_baud |
| Shell | USART1 115200 `io> `,完整行编辑/历史/Tab 补全(830 行 1:1) |
| 看门狗 | IWDG 30s,3s 喂狗;心跳 LED 300ms/2.7s |
| 升级引导 | embassy-boot:页 = 128K(片内最大扇区),UDP/WS/CAN 三通道推送,试用启动 10s 后自动确认、失败回滚 |

## 仓库结构

```
├── Cargo.toml              # workspace: proto / firmware / littlefs2-sys / boot
├── crates/
│   ├── proto/              # no_std 纯逻辑库(host cargo test 覆盖 C 的 HOST_TEST)
│   │   ├── regmap / mb_server / mbtcp_adu / rtu_frame    # Modbus 全套
│   │   ├── udp_cfg / fw_upg                                # UDP 命令、升级会话/CRC16/分区表
│   │   ├── config_store / history / web_json / ws          # 配置编解码/历史记录/JSON/WS 帧
│   │   └── crc / time_math / adc_math
│   ├── boot/               # embassy-boot 引导器(bin,片内 0x08000000,128K 预留)
│   ├── firmware/           # embassy 应用(ACTIVE 0x08020000,384K)
│   │   ├── main.rs         # 时钟树/任务布局/心跳(试用启动确认)/panic
│   │   ├── net.rs httpd.rs ftpd.rs mbtcp.rs               # 网络服务
│   │   ├── storage.rs w25q.rs                              # littlefs + NOR + 升级 RPC
│   │   ├── fw.rs fw_can.rs                                 # 升级会话 + CAN 通道
│   │   ├── shell.rs uart_raw.rs log.rs                     # 控制台
│   │   ├── sampling.rs io_gpio.rs rtu.rs systime.rs reboot.rs appstate.rs
│   └── littlefs2-sys/      # littlefs 2.11 vendored + libc shims
├── tools/make_images.py    # objcopy 出 boot.bin/app.bin/full.bin(无签名步骤)
├── build/                  # 产物 + CAN 测试脚本(can_upgrade.py 等)
└── docs/
    ├── code-walkthrough.md     # ★ 全部源码的代码详解
    ├── embassy.md              # 按子模块的 embassy 详细文档
    └── comparison-c-vs-rust.md # C/Rust 行为与性能对比报告
```

## 分区布局(embassy-boot)

| 分区 | 介质 | 地址 | 大小 | 说明 |
|---|---|---|---|---|
| BOOTLOADER | 片内 | 0x08000000 | 128K | 引导器本体 ~8K,余量给调试构建 |
| ACTIVE | 片内 | 0x08020000 | 384K | 应用槽 = 三个 128K 扇区(页边界=物理扇区) |
| STATE | W25Q | 0x000000 | 4K | swap/revert 进度 |
| DFU | W25Q | 0x001000 | 512K | 升级暂存(≥ ACTIVE + 1 页) |
| 配置 A/B | W25Q | 0xE0000/0xE8000 | 32K×2 | 不变 |
| littlefs | W25Q | 0xF0000 | ~15M- | 不变 |

embassy-boot 页大小 = max(ACTIVE::ERASE_SIZE, DFU::ERASE_SIZE) = 128K(F407 内部
flash 最大扇区);ACTIVE 取整扇区使每次页擦除恰好对应一个物理扇区。

## 构建与烧写

依赖:rustup(stable,`rustup target add thumbv7em-none-eabihf`)、probe-rs、Python 3。

```bat
cd io-edge-hub-rust
cargo build --release          # 同时构建 io-edge-hub-boot 与 io-edge-hub-fw
python tools\make_images.py    # → build\boot.bin / app.bin / full.bin

:: 首次(或换引导器):整机镜像一次烧入
probe-rs download --chip STM32F407VETx --binary-format bin ^
    --base-address 0x08000000 build\full.bin

:: 只更新应用(引导器保留):
probe-rs download --chip STM32F407VETx --binary-format bin ^
    --base-address 0x08020000 build\app.bin
probe-rs reset --chip STM32F407VETx
```

应用是普通二进制(无 imgtool 头/签名);完整性由升级会话的读回 CRC16 校验,
坏镜像由 embassy-boot 的试用启动 + 回滚兜底。OTA 推送(UDP v2 / WS / CAN)
写入外部 DFU 槽,重启后由引导器交换。

## 测试

```bat
:: 主机单测(proto,移植自 C 的 HOST_TEST):
cargo test -p io-edge-hub-proto --target x86_64-unknown-linux-gnu

:: e2e(直接复用 C 仓库 93 项套件,设备任意固件自动识别):
cd C:\Users\jxwaz\code\io-edge-hub-freertos\tests\e2e
python -m pytest --fw-image C:\Users\jxwaz\code\io-edge-hub-rust\build\app.bin ^
    --rs485-port COM10          # RTU 需要 USB-RS485 (COM10); UART 控制台为 COM9

:: CAN 升级全流程(keyhash→START→64B 流控推送→CONFIRM→REBOOT→换机):
cd C:\Users\jxwaz\code\io-edge-hub-rust\build
python can_upgrade.py             # TOOLMODE=1 模拟 io-edge-hub 上位机的 8 帧/应答节奏
```

## 版本与发布

- 版本串 `vM.m.p_<git6>`,由 `VERSION` 文件 + `crates/firmware/build.rs` 生成,烧进固件
  (UDP 0x04 / HTTP /api/info / CAN VERSION 均可读)
- CAN 协议细节(64B 流控、0x106/0x107 bootloader 救援模式预留)以 Zephyr
  `libs/can_fw_upgrade` 为权威源,上位机工具为 `~/code/io-edge-hub`

## 文档

- **[docs/code-walkthrough.md](docs/code-walkthrough.md) — 代码详解**(逐文件讲解全部
  源码:proto 纯逻辑库、firmware 各任务/驱动、并发模型与内存布局、C 移植保留的怪癖清单)
- **[docs/embassy.md](docs/embassy.md) — 按子模块的 embassy 详细文档**(执行模型、中断绑定、
  同步原语、各外设驱动用法、内存布局、以及本项目踩过的 8 个坑)
- [docs/comparison-c-vs-rust.md](docs/comparison-c-vs-rust.md) — 与 C 版的逐功能对比和性能基准
