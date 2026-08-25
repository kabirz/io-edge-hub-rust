# io-edge-hub 上位机(host-tool,embassy-boot 协议版)

从 `~/code/io-edge-hub`(C/Win32 原版)整树移植,UI 与操作习惯完全一致,
仅按本仓库固件(feat/embassy-boot 分支)的升级协议改造。原生 Win32 C GUI,
4 个 tab:参数设置 / 固件升级 / Modbus 调试 / 历史记录。

## 与原版的差异(全部集中在升级协议)

| 项 | 原版(MCUboot) | 本版(embassy-boot) |
|---|---|---|
| 升级载荷 | imgtool 签名镜像(magic+TLV) | `app.dfu.bin` = 裸镜像 + 尾部 64B ed25519 签名 |
| 载荷校验 | 解析 MCUboot 头 + TLV info | 长度 ∈ (64B, 512K];识别旧 MCUboot magic 并明确拒绝 |
| keyhash | 从镜像 KEYHASH TLV 提取 | 编译期常量 SHA-256(ed25519 公钥) |
| 验签 | bootloader 端 RSA | 设备应用端 salty ed25519(FW_END 时,约 1s) |
| 换机语义 | slot1 + trailer,SWAP_USING_SCRATCH | DFU 暂存 + state 魔数,重启后逐页互换(约 30s) |
| 升级后 | 重启即换 | 重启换机,新镜像跑通 main 自动确认,否则自动回滚 |

改动文件:`src/fw_image.c`、`include/fw_image.h`(重写)、`src/upgrade_tab.c`
(浏览校验/keyhash 取用 + 用户可见文案)、`CMakeLists.txt`(工程名
`io-edge-hub-host`)。其余源文件(UDP/CAN/Modbus/历史解析)wire 协议与本
固件一致,原样拷贝。

keyhash 常量与固件 `proto::fw_upg::FW_KEYHASH`、`tools/fwupd_udp.py` 同源;
换钥匙(`tools/gen_ed25519.py`)时三处同步更新并重编两端。

注:CAN 救援模式勾选框仅对旧 C/Zephyr 固件有效(0x106/0x107 探测应答);
embassy-boot 固件的 bootloader 不含 CAN,勾选后会在探测阶段超时。

## 构建

需要 CMake ≥ 3.25 + Visual Studio(MSVC):

    tools\build.bat

产物 `out\bin\Release\io-edge-hub-host.exe`。CAN 升级需安装 PCAN-Basic
驱动(运行时动态加载 PCANBasic.dll)。
