# 固件升级详解:embassy-boot 原理、协议与操作

> 适用分支:`feat/embassy-boot`(2026-08)。本页是升级子系统的唯一完整参考:
> 启动/换机原理、载荷格式、UDP/WS/CAN 三通道逐字节协议、操作方法、
> 故障排查与实测数据。分区常量的权威源是 `crates/proto/src/fw_upg.rs`。

---

## 1. 总览

```
                ┌──────────── 片内 Flash 512K ────────────┐
  0x08000000    │ boot (sector 0-4, ≤128K)                │  embassy-boot, 实测 8952 B
  0x08020000    │ active (sector 5-7, 384K = 3×128K 页)   │  运行中的 app(裸镜像)
                └─────────────────────────────────────────┘
                ┌──────────── W25Q128 NOR 16M ────────────┐
  0x000000      │ DFU 512K(暂存新镜像)                  │  ≥ active + 1 页(硬约束)
  0x080000      │ state 4K(swap 魔数 + 进度索引)        │
  0x0E0000      │ cfg A/B(32K×2,不变)                   │
  0x0F0000      │ littlefs 历史区(不变)                 │
                └─────────────────────────────────────────┘
```

一次升级的完整链路:

```
上位机                     设备(app)                        设备(bootloader)
──────                     ──────────                        ───────────────
START(size+keyhash)  ──►  擦 DFU 512K
DATA*(载荷分片)      ──►  页缓冲写入 DFU
END(crc16)           ──►  回读 CRC + ed25519 验签          (验签失败 → 拒绝,一切如初)
                           state 写 SWAP 魔数(0xF0)
REBOOT               ──►  同步历史 → 100ms → 复位 ──────►  逐页互换 active↔DFU(~30s,
                                                              断电可续)
                      ◄─  跳转新镜像 ─────────────────────  新 app 启动
GET_VERSION 轮询     ──►  应答版本
                           main 末尾 boot_confirm():
                           state 写 BOOT 魔数(0xD0)
                           —— 若新镜像跑不到这里,下次复位自动回滚旧镜像
```

三通道(UDP/WS/CAN)只是传输层不同,落在设备侧同一个会话门面
(`crates/firmware/src/fw.rs`),因此验签、状态机、回滚行为完全一致。

---

## 2. 原理

### 2.1 为什么是这张分区表

- **换机页大小 = max(两个 flash 的擦除粒度)**。embassy-boot 以"页"为单位
  搬运;F4 片内扇区不均匀(4×16K + 1×64K + 3×128K),embassy 驱动对外报告的
  ERASE_SIZE 是最大扇区 128K,所以换机页 = 128K。
- **active 必须是 128K 的整倍数且扇区对齐** → 只能取 0x08020000 起 3 页 = 384K
  (sector 5-7)。bootloader 区域因此是 0x08000000-0x0801FFFF(128K,实际只
  用 ~9K,余量留足)。
- **DFU ≥ active + 1 页**(embassy-boot `assert_partitions` 硬校验):
  384K + 128K = 512K,DFU 取 NOR 起始 512K。
- **state 需求极小**:`2 + 4×(active/页)` 字节 = 2+12 = 14 B,取 NOR 上
  0x80000 处一个 4K 扇区。cfg A/B 与 littlefs 布局**原样不动**,历史文件和
  配置跨换机保留。

### 2.2 换机算法(swap-by-copy,断电安全)

以 active 3 页(P0-P2)、DFU 4 页为例,记 active 第 i 页为 `Ai`、DFU 第 i 页
为 `Di`。从**最后一页往回**搬:

```
步骤1: A2 → D3   (active 旧页暂存到 DFU 的“多出的一页”)
步骤2: D2 → A2   (新镜像第2页落位)         进度索引 +2
步骤3: A1 → D2, D1 → A1                    +2
步骤4: A0 → D1, D0 → A0                    +2  → active 已是新镜像
```

- 每完成一个"半步"就向 **state 分区的进度索引区**写入一个 0x00 字节
  (NOR 只能 1→0,无需擦除即可顺序推进)。
- 任意时刻断电,下次启动 bootloader 读进度索引,从断点继续,而不是从头再搬。
- 换机完成后旧镜像恰好位于 DFU 分区——这就是**回滚**的来源。

### 2.3 state 分区布局与状态机

`STATE_OFF=0x80000, STATE_SIZE=0x1000`(NOR 绝对地址),字节语义
(WRITE_SIZE=1,与 embassy-boot 0.7 `BlockingFirmwareState` 逐字节一致):

| 偏移 | 内容 |
|---|---|
| 0 | 状态魔数(见下表) |
| 1 | 进度索引有效性:0xFF=有效,0x00=已作废 |
| 2.. | 进度索引(第 k 个 0x00 字节 = 已完成 k 个半步) |

| 魔数(字节 0) | 含义 |
|---|---|
| 0xD0 `BOOT` | 常态:直接启动 active |
| 0xF0 `SWAP` | app 已确认新镜像:下次启动执行换机;换机后仍是 0xF0 |
| 0xC0 `REVERT` | 曾换机但新镜像未确认,bootloader 已回滚 |
| 0xE0 `DFU_DETACH` | USB DFU 预留(本设备未用) |
| 其它/擦除态 0xFF | 视同 BOOT |

**试运行语义**(取代 MCUboot 的 test/perm 位):

1. app 验签通过 → `boot_set_pending()` 把 state[0] 写成 0xF0(先作废进度
   索引再整扇区擦除后写魔数,断电中断也安全);
2. 复位 → bootloader 见 SWAP:若未换过 → 执行换机 → 跳新镜像;若已换过
   (说明上次启动后一直没确认)→ **执行回滚**,state 置 0xC0;
3. 新镜像在 `main` **末尾**调用 `boot_confirm()`(跑通了时钟/存储/网络/全部
   任务 spawn 才会到达)→ state[0] 写回 0xD0,换机定局;
4. 新镜像若在 main 完成前死机/挂起 → IWDG(30s)或人工复位 → 回滚旧镜像。
   坏镜像最多存活一个启动周期,不会变砖。

### 2.4 签名方案

- 密钥:**ed25519**(`tools/gen_ed25519.py` 生成,私钥 `keys/ed25519.key`
  不入仓;公钥 32B + 其 SHA-256 keyhash 固化在 `proto::fw_upg`)。
- 载荷尾部 64B 签名 = 对 `SHA-512(镜像)` 做**纯 Ed25519** 签名
  (RFC 8032 PureEdDSA,消息就是那 64 字节摘要——与 embassy-boot/salty
  的 `verify(message = sha512(fw))` 约定一致,不是 Ed25519ph)。
- **验签在应用侧**(salty 库,~1s),通过才允许写 SWAP 魔数;bootloader
  本身零密码学(所以只有 9K)。威胁模型不变:攻击者没有私钥就无法让设备
  进入换机流程,MCUboot 时代的 RSA/KEYHASH 校验等价地由 ed25519+keyhash
  承担(keyhash 在 START 时即校验,错钥连 DFU 擦除都不会发生)。
- 传输完整性另有两层:各通道自身的 CRC16(UDP/WS)与写入后整镜像回读 CRC
  (CAN 无 CRC,直接以验签兜底)。

---

## 3. 升级载荷格式

`build/app.dfu.bin`(218,740 B 实测 = 218,676 镜像 + 64 签名):

```
┌─────────────────────────────┬────────────────────────┐
│ raw app binary(向量表在首) │ ed25519 签名 64B       │
└─────────────────────────────┴────────────────────────┘
  0                                len-64           len
```

- 三通道传输的都是**这个文件本身**;`size` = 文件长度(含签名)。
- 设备侧 `finish()` 按长度切出镜像与签名,回读 CRC 覆盖整个载荷。
- 大小约束:65 B < size ≤ 512K(`fw_upg::payload_ok`)。
- CRC16:反射多项式 0x1021、初值 0(Zephyr `sys/crc.h crc16_ccitt`),
  主机按整个文件计算。

---

## 4. 传输协议

### 4.0 通用规则

- 升级命令仅接受**同 /24 子网**来源(跨网段只有 0x11 GET_IP 放行)。
- 会话期间历史记录暂停写入(storage 让路,防 NOR 冲突)。
- 除 REBOOT(CAN)外每个命令都有应答;超时重发是安全的(见各通道)。

### 4.1 UDP :8600(主通道)

请求 = `[cmd][payload...]`,应答同端口。

| 命令 | 请求字节 | 应答 | 说明 |
|---|---|---|---|
| 0x01 START | `[01][size LE32][keyhash 32B 可选]` | `[01][status][chunk LE16]` | status: 1=ok / 2=keyhash 不符 / 0=其它(尺寸非法、擦除失败、busy)。keyhash 缺省时不校验(仅本机调试用)。chunk 恒 1400(v2 块大小)。应答前完成 DFU 整擦(~2s,超时给足) |
| 0x02 DATA(legacy) | `[02][data ≤511B]` | `[02][received LE32]` | 顺序追加,无偏移字段,兼容旧上位机 |
| 0x06 DATA_V2 | `[06][offset LE32][data ≤1400B]` | `[06][received LE32]` | **仅当 offset == received 才写入**;乱序/重复静默丢弃。主机用 go-back-N:连发 8 块窗口,按应答回退重发 |
| 0x03 END | `[03][test u8][crc LE16]` | `[03][ok]` | ok=1:CRC 回读 + ed25519 验签 + SWAP 魔数全部通过。test 字节保留兼容,新方案忽略(恒为试运行+自动确认)。耗时 ~1s |
| 0x05 REBOOT | `[05]` | `[05 01]` | 应答在网后才复位:历史同步 → 100ms → 复位 |
| 0x04 GET_VERSION | `[04]` | 定长 14B:`[04]["v"][maj]['.'][min]['.'][pat]['_'][git 6B ASCII]` | 例 `04 76 30 2e 33 2e 30 5f 61 61 30 64 65 32` = `v0.3.0_aa0de2`;换机后轮询它等设备回来 |

诊断(不经会话门面,随时可用):

- `[0xFA]` → `[FA][rx_cnt LE32][rx_got LE32]`:shell RX 计数。
- `[0xFB]` → `[FB][16×LE32]`:`FW_DBG`,最近一次 `finish()` 的失败详情:

| d[0] | 含义 | 关键字段 |
|---|---|---|
| 1 | 尾页冲刷失败 | d[1]=已写字节 |
| 2 | 长度不符 | d[1]=written d[2]=total |
| 3 | 回读开始快照 | d[1]=total d[4..8]=首 16B |
| 4 | 回读 NOR 失败 | — |
| 5 | CRC 不符 | d[1]=计算值 d[2]=期望值 d[8..12]=尾 16B |
| 6 | **ed25519 验签失败** | d[1]=total |
| 7 | 成功 | d[1]=total d[2]=crc d[4]=1(签名通过) |

### 4.2 WebSocket `/ws`(Web 页面通道)

连接 `ws://<ip>/ws` 后:

1. 发文本帧 `{"cmd":"fw_start","size":<文件字节数>,"keyhash":"<base64(32B),可选>"}`(keyhash 44 字符含 `=`);
   应答文本帧 `{"ok":true}` 或 `{"ok":false,"err":"bad size" | "keyhash mismatch" | "already in progress" | "erase/init"}`。
   已有会话占用时先 `{"cmd":"fw_end"}` 清掉。
2. 连续发**二进制帧**按序传载荷(单帧载荷 ≤10256B,网页端每帧 10240B;设备 CRC 在接收侧累计)。
3. 发 `{"cmd":"fw_end"}`;设备做尺寸核对 + CRC + 验签 + SWAP 魔数,应答
   `{"ok":true}` 后**自动**走 ~3s 优雅重启,无需再发 REBOOT。
   失败 err:`"no data" | "size mismatch" | "crc mismatch"(验签失败也是它) | "boot_request"`。
4. 之后浏览器轮询 `/api/info` 等设备回来(uptime 归零)。

注:文本帧限 255B、二进制帧由解析器缓冲;升级期间设备的 1s io/regs、10s info
推送继续,网页可显示进度。

### 4.3 CAN(bxCAN,与 Zephyr `libs/can_fw_upgrade` 语义对齐)

滤波器:标准帧,业务 ID 精确匹配 + 0x100-0x1FF 段升级帧。波特率
{50,100,125,250,500,1000}k 可配,非法回落 250k。

| 帧 ID | 方向 | DLC | 载荷 | 说明 |
|---|---|---|---|---|
| 0x101 | 主→设 | 8 | `[cmd LE32][arg LE32]` | 平台命令 |
| 0x102 | 设→主 | 8 | `[code LE32][arg LE32]` | 平台应答 |
| 0x103 | 主→设 | ≤8 | 原始数据 | 顺序追加(无偏移) |
| 0x104 | 主→设 | ≤8 | `[seq][keyhash 分片 ≤7B]` | keyhash 32B 按 7B×5 片(seq 0-4) |
| 0x105 | 设→主 | ≤8 | `[seq][版本串分片 ≤7B]` | VERSION 应答的分片序列 |

命令字(0x101 的 cmd):

| cmd | arg | 行为 |
|---|---|---|
| 0 START | size(含 64B 签名) | **无条件重开会话**(先 abort 清残留);若 5 片 keyhash 已收齐则校验。应答 `CODE_OFFSET(0),0`;keyhash 错 → `CODE_KEYHASH_ERROR(6)` |
| 1 CONFIRM | 0 | `finish(None)`:无 CRC 比对,直接回读 + **ed25519 验签** + SWAP 魔数。成功应答 `CODE_CONFIRM(3), 0x55AA55AA`;失败 `CODE_TRANSFER_ERROR(5)` |
| 2 VERSION | — | 先 `CODE_VERSION(2), 串长`,随后 0x105 分片发出 `v0.3.0_abc123` |
| 3 REBOOT | — | **无应答**:100ms 挥手窗口 → 历史同步 → 500ms → 复位 |

数据流控(0x103):设备每收到 **64B 整倍数**回一帧 `CODE_OFFSET(0), received`;
最后一个字节收满时改回 `CODE_UPDATE_SUCCESS(1), total`。写入失败回
`CODE_FLASH_ERROR(4)`;未 START 先发数据回 `CODE_TRANSFER_ERROR(5)`。

上位机参考节奏:`build/can_upgrade.py`(模拟 8 帧/应答的窗口节奏,全速
250kbps 推 206KB 约 16s)。

---

## 5. 操作方法

### 5.1 构建与签名(Windows 主机,先激活 zephyr venv)

```bat
cd io-edge-hub-rust
cargo build --release
python tools\gen_ed25519.py      :: 仅首次:生成 keys/ed25519.key/.pub
python tools\sign.py
```

产物(`build/`):

| 文件 | 用途 |
|---|---|
| `boot.bin`(8,952B) | bootloader 裸镜像(0x08000000) |
| `app.bin` | 应用裸镜像(0x08020000) |
| `app.dfu.bin` | **升级载荷**(三通道通用,含 64B 签名) |
| `full.bin` / `full.hex` | 整机制造镜像 = boot 补 0xFF 到 0x08020000 + app.bin |

换密钥流程:重跑 `gen_ed25519.py` → 把打印的 PUBKEY_HEX/KEYHASH_HEX 更新到
`crates/proto/src/fw_upg.rs`(`FW_PUBKEY`/`FW_KEYHASH`)→ 重编两个固件。

### 5.2 SWD 烧写(制造/恢复)

```bat
:: 整机(覆盖 bootloader + app,0x08000000 起):
tools\flash_full.cmd                     :: ST-LINK_CLI
python tools\flash.py --build            :: 或 cargo-flash(探针所在机器)

:: 仅应用(保留在板 bootloader):
tools\flash_app.cmd                      :: app.bin → 0x08020000
```

bench(Linux 10.84.9.190)上:

```bash
scp build/full.bin bench:/tmp/ && ssh bench \
  "cargo flash --chip STM32F407VETx --path /tmp/full.bin \
   --binary-format bin --base-address 0x8000000"
```

### 5.3 UDP 命令行升级

```bash
python tools/fwupd_udp.py 192.168.12.101 build/app.dfu.bin
```

一条命令完成 START/DATA_V2/END/REBOOT 全流程并等待换机结束(实测:传输 4.9s →
验签 1.0s → 换机 ~30s → ONLINE,全程 ~37s)。无第三方依赖。

### 5.4 Web 页面升级

浏览器打开 `http://192.168.12.101/`,固件页选择 `app.dfu.bin` 上传;
页面经 WS 通道完成 4.2 流程并自动等待设备回来。

### 5.5 CAN 升级

```bash
cd build && python can_upgrade.py    # 交互参数见脚本头注释
```

流程:keyhash 5 片(0x104)→ START(0x101,cmd 0)→ 0x103 按流控推全文件 →
CONFIRM(cmd 1,等 0x55AA55AA)→ REBOOT(cmd 3,无应答,直接等设备回来)。

### 5.6 串口观测(USART1 @115200)

| 日志行 | 阶段 |
|---|---|
| `io-edge-hub boot v0.3.0_<git6>` | bootloader 横幅 |
| `nor: w25q128 ok` / `nor: absent -> booting active without swap` | NOR 探测 |
| `boot: swap done (confirm in app or revert)` | 本次完成了换机 |
| `boot: reverted to previous image` | 新镜像未确认,已回滚 |
| `boot: prepare error -> booting anyway` | state 读写异常(等效断电,续传语义) |
| `fwupg: start` / `fwupg: end` | app 会话起止 |
| `fw: boot confirmed` | main 末尾确认成功(换机定局) |
| `fw: boot confirm FAILED (will revert)` | 确认写失败,下次复位回滚 |

---

## 6. 实测数据(2026-08-25,218,740B 载荷)

| 阶段 | 耗时 |
|---|---|
| UDP DATA_V2 传输(1400B×8 窗口) | 4.9 s(~44 KB/s) |
| END:整载荷回读 CRC + salty ed25519 验签 | 1.0 s |
| bootloader 换机(3×128K 页互换,片内+SPI NOR) | 29 s |
| **单次升级总时长(END→在线)** | **~37 s** |
| bootloader 体积 | 8,952 B(对比 MCUboot 29,844 B) |
| 验签引入的 app 体积增量(salty+SHA-512) | +8.6 KB(app 总 218,088 B) |

---

## 7. 故障排查

| 现象 | 查什么 |
|---|---|
| START 应答 status=2 / err "keyhash mismatch" / CAN CODE 6 | 载荷与固件密钥不配套:确认 `app.dfu.bin` 由当前 `keys/ed25519.key` 签出、`FW_KEYHASH` 常量已同步 |
| END ok=0 / WS "crc mismatch" | 发 `0xFB` 看 FW_DBG:d[0]=5 CRC 传输误码;d[0]=6 验签失败(载荷被改动或签名算法不对——必须签 SHA-512 摘要) |
| REBOOT 后设备一直不回来 | 串口看 bootloader 横幅是否出现;换机 ~30s 内 `GET_VERSION` 无应答是正常的,超过 150s 才算失败 |
| 换机后又回到旧版本 | 串口应有 `boot: reverted`:新镜像在 main 末尾前挂了(或 `fw: boot confirm FAILED`),查新镜像本身 |
| UDP 全部无应答 | 跨网段被丢弃(仅 0x11 放行);或 IP 已被 SET_IP 改动,待重启生效 |
| CAN 推送极慢 | 上位机没按"每 8 帧等一帧 OFFSET"节奏,烧超时(见 fw_can.rs ACK_INTERVAL 注释) |

---

## 8. 与 MCUboot 方案对照

| | 旧(MCUboot C) | 新(embassy-boot) |
|---|---|---|
| bootloader | C,29,844 B,boot 64K 扇区 | Rust,8,952 B,≤128K 区域 |
| 镜像格式 | imgtool 头 512B + TLV(RSA-2048) | 裸 bin + 64B ed25519 签名 |
| 验签位置 | bootloader(每次换机) | app(finish 时,salty) |
| 暂存/换机 | slot1 + scratch(SWAP_USING_SCRATCH) | DFU 512K + state 进度索引(逐半步断电续传) |
| 确认/回滚 | trailer image_ok/swap_type,手写 | state 魔数 + main 末尾自动确认,未确认自动回滚 |
| 应用链接地址 | 0x08010200(448K slot) | 0x08020000(384K,向量表在分区首) |
| 三通道 wire 协议 | 0x01/02/03/05/06、WS、CAN 0x101-0x105 | **完全相同**(仅载荷与 keyhash 语义变化) |
| NOR cfg/littlefs | 0xE0000 / 0xF0000 | 不变(数据跨方案保留) |
