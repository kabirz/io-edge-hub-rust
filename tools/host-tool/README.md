# io-edge-hub 上位机(host-tool,embassy-boot 协议版)

从 `~/code/io-edge-hub`(C/Win32 原版)整树移植,UI 与操作习惯完全一致,
仅按本仓库固件(feat/embassy-boot 分支)的升级协议改造。原生 Win32 C GUI,
4 个 tab:参数设置 / 固件升级 / Modbus 调试 / 历史记录。

## 与原版的差异(全部集中在升级协议)

| 项 | 原版(MCUboot) | 本版(embassy-boot) |
|---|---|---|
| 升级载荷 | imgtool 签名镜像(magic+TLV) | `app.dfu.bin` = 裸镜像 + 尾部 64B ed25519 签名 |
| 载荷校验 | 解析 MCUboot 头 + TLV info | 长度 ∈ (64B, 512K];识别旧 MCUboot magic 并明确拒绝 |
| keyhash | 从镜像 KEYHASH TLV 提取 | **设备自报**:UDP 通道升级前问设备(0x15);CAN 通道读 exe 旁 `ed25519.keyhash`;兜底内置常量 |
| 验签 | bootloader 端 RSA | 设备应用端 salty ed25519(FW_END 时,约 1s) |
| 换机语义 | slot1 + trailer,SWAP_USING_SCRATCH | DFU 暂存 + state 魔数,重启后逐页互换(约 30s) |
| 升级后 | 重启即换 | 重启换机,新镜像跑通 main 自动确认,否则自动回滚 |

改动文件:`src/fw_image.c`、`include/fw_image.h`(重写)、`src/upgrade_tab.c`
(浏览校验/keyhash 取用 + 用户可见文案)、`src/udp_manager.c/h`(GET_KEYHASH)、
`CMakeLists.txt`(工程名 `io-edge-hub-host`)。其余源文件(CAN/Modbus/历史解析)
wire 协议与本固件一致,原样拷贝。

换钥匙只改固件一处(`proto::fw_upg` + 重编);本工具 UDP 通道向设备要 keyhash,
CAN 通道把 `tools/gen_ed25519.py` 产出的 `keys/ed25519.keyhash` 拷到 exe 旁即可。
内置常量仅作过渡期兜底(旧钥签名的过渡固件仍可用它完成轮换)。

注:CAN 救援模式勾选框仅对旧 C/Zephyr 固件有效(0x106/0x107 探测应答);
embassy-boot 固件的 bootloader 不含 CAN,勾选后会在探测阶段超时。

## 构建

需要 CMake ≥ 3.25 + Visual Studio(MSVC):

    tools\build.bat

产物 `out\bin\Release\io-edge-hub-host.exe`。CAN 升级需安装 PCAN-Basic
驱动(运行时动态加载 PCANBasic.dll)。

## 协议层联机自测

`protocol_test` 目标随 CMake 一起编译(`tests/protocol_test.c` 与本工具
自己的 `src/udp_manager.c`、`src/fw_image.c` 链成控制台程序——GUI 无法
自动化,这里跑的是同一份协议代码的完整调用序列):

    tools\build.bat
    out\bin\Release\protocol_test.exe 192.168.12.101 ..\..\build\app.dfu.bin

流程 = GET_VERSION → FW_START(常量 keyhash) → FW_DATA_V2 窗口流 →
FW_END(设备端 ed25519 验签) → REBOOT → 轮询等换机完成。注意会触发一次
真实换机(同镜像安全,自动确认)。2026-08-25 实测 ×2(经转发链路):传输
218,740B 用时 3.1s,验签 ~1s,重启换机 32s 后上线,均 PASS。
