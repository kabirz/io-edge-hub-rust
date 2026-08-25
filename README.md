# io-edge-hub-rust

io-edge-hub 边缘采集网关固件的 **Rust / embassy 重写版**,与 C(FreeRTOS)版本行为逐级对齐:
同一份 MCUboot bootloader、同一套 NOR 分区布局、同一个签名密钥,C 版写的文件本版可直接挂载读写,
93 项 e2e 测试(直接复用 C 仓库套件)全绿。

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
| MCUboot | 同 boot.bin + 同 RSA-2048 密钥,trailer 手写(SWAP_USING_SCRATCH),UDP/WS/CAN 三通道升级 |

## 仓库结构

```
├── Cargo.toml              # workspace: proto / firmware / littlefs2-sys
├── memory.x                # FLASH 0x08010200/0x6FE00, RAM 128K, CCRAM 64K
├── crates/
│   ├── proto/              # no_std 纯逻辑库(host cargo test 覆盖 C 的 HOST_TEST)
│   │   ├── regmap / mb_server / mbtcp_adu / rtu_frame    # Modbus 全套
│   │   ├── udp_cfg / fw_upg                                # UDP 命令、升级状态机/CRC16/TLV
│   │   ├── config_store / history / web_json / ws          # 配置编解码/历史记录/JSON/WS 帧
│   │   └── crc / time_math / adc_math
│   ├── firmware/           # embassy 应用(见 docs/embassy.md 逐模块文档)
│   │   ├── main.rs         # 时钟树/任务布局/心跳/panic
│   │   ├── net.rs httpd.rs ftpd.rs mbtcp.rs               # 网络服务
│   │   ├── storage.rs w25q.rs                              # littlefs + NOR
│   │   ├── fw.rs fw_can.rs                                 # 升级会话 + CAN 通道
│   │   ├── shell.rs uart_raw.rs log.rs                     # 控制台
│   │   ├── sampling.rs io_gpio.rs rtu.rs systime.rs reboot.rs appstate.rs
│   └── littlefs2-sys/      # littlefs 2.11 vendored + libc shims
├── tools/sign.py           # imgtool 签名(参数与 C 完全一致)
├── build/                  # 产物 + CAN 测试脚本(can_upgrade.py 等)
└── docs/
    ├── embassy.md          # ★ 按子模块的 embassy 详细文档
    └── comparison-c-vs-rust.md  # C/Rust 行为与性能对比报告
```

## 构建与烧写

依赖:rustup(stable,`rustup target add thumbv7em-none-eabihf`)、probe-rs、Python 3.12(imgtool)。

```bat
cd io-edge-hub-rust
cargo build --release
python tools\sign.py        # → build\app.signed.bin(应用)+ build\full.bin(整机)

:: 烧写应用(设备保留原 MCUboot,app 烧到 0x08010000):
probe-rs download --chip STM32F407VETx --binary-format bin ^
    --base-address 0x08010000 build\app.signed.bin
probe-rs reset --chip STM32F407VETx
```

`full.bin`(boot 64K + app)= 整机制造镜像。签名私钥 `keys/root-rsa2048.pem`
不入仓(已 gitignore,绝不提交)。

## 测试

```bat
:: 主机单测(proto,移植自 C 的 HOST_TEST):
cargo test -p io-edge-hub-proto

:: e2e(直接复用 C 仓库 93 项套件,设备任意固件自动识别):
cd C:\Users\jxwaz\code\io-edge-hub-freertos\tests\e2e
python -m pytest --fw-image C:\Users\jxwaz\code\io-edge-hub-rust\build\app.signed.bin ^
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
