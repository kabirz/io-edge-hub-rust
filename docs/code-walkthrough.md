# io-edge-hub-rust 代码详解

本文逐模块讲解本仓库的全部源码:每个文件做什么、关键函数/数据结构的设计意图、
模块之间如何协作,以及代码中那些"看起来奇怪但有其原因"的细节。
与 [embassy.md](embassy.md)(框架用法视角)和
[comparison-c-vs-rust.md](comparison-c-vs-rust.md)(与 C 版对比)互补;
本文以**代码本身**为线索。

- 目标硬件:LCKFB STM32F407VET6(168 MHz,512 K Flash,128 K SRAM + 64 K CCM)
  + W5500(SPI2) + W25Q128 NOR(SPI1)
- 固件形态:no_std + embassy 异步执行器;引导器 embassy-boot(boot/ACTIVE 片内,DFU/STATE 外置)
- 行为基准:C/FreeRTOS 版固件,93 项 e2e 全绿,盘上格式兼容

## 目录

1. [总览:三层结构与一条主线](#1-总览三层结构与一条主线)
2. [Workspace 与构建系统](#2-workspace-与构建系统)
3. [crates/proto:纯逻辑库(可主机测试)](#3-cratesproto纯逻辑库可主机测试)
   - [bytes / crc / time_math / adc_math](#31-基础算法bytes--crc--time_math--adc_math)
   - [regmap:寄存器表——全系统的单一事实源](#32-regmap寄存器表全系统的单一事实源)
   - [mb_server / mbtcp_adu / rtu_frame:Modbus 三层](#33-mb_server--mbtcp_adu--rtu_framemodbus-三层)
   - [udp_cfg:UDP :8600 配置协议](#34-udp_cfgudp-8600-配置协议)
   - [config_store:A/B 槽配置编解码](#35-config_storea-b-槽配置编解码)
   - [history:历史记录编码与文件名](#36-history历史记录编码与文件名)
   - [web_json / ws:Web 支撑](#37-web_json--wsweb-支撑)
   - [fw_upg:升级协议原语](#38-fw_upg升级协议原语)
   - [ftp:PORT/EPRT 解析](#39-ftpporteprt-解析)
4. [crates/firmware:embassy 应用](#4-cratesfirmwareembassy-应用)
   - [main.rs:启动序列与任务布局](#41-mainrs启动序列与任务布局)
   - [appstate.rs:全局状态与 Hooks 桥接](#42-appstaters全局状态与-hooks-桥接)
   - [net.rs:W5500 组网与 UDP 服务](#43-netrsw5500-组网与-udp-服务)
   - [storage.rs:NOR 唯一所有者(本项目最核心的文件)](#44-storagersnor-唯一所有者本项目最核心的文件)
   - [w25q.rs:NOR 驱动](#45-w25qrsnor-驱动)
   - [升级体系:embassy-boot 引导器 + fw.rs + fw_can.rs](#46-升级体系embassy-boot-引导器--fwrs--fw_canrs-can-通道)
   - [httpd.rs / ftpd.rs / mbtcp.rs:TCP 服务三件套](#47-httpdrs--ftpdrs--mbtcprstcp-服务三件套)
   - [shell.rs / uart_raw.rs / log.rs:控制台三件套](#48-shellrs--uart_rawrs--logrs控制台三件套)
   - [sampling.rs / io_gpio.rs / systime.rs:IO 与时间](#49-samplingrs--io_gpiors--systimersio-与时间)
   - [reboot.rs / stackmark.rs:辅助设施](#410-rebootrs--stackmarkrs辅助设施)
5. [crates/littlefs2-sys:vendored littlefs](#5-crateslittlefs2-sysvendored-littlefs)
6. [横切设计:并发模型与内存布局](#6-横切设计并发模型与内存布局)
7. [从 C 移植时刻意保留的行为怪癖清单](#7-从-c-移植时刻意保留的行为怪癖清单)

---

## 1. 总览:三层结构与一条主线

```
┌─────────────────────────────────────────────────────────────┐
│ crates/boot       embassy-boot 引导器(bin,独立链接)         │
├─────────────────────────────────────────────────────────────┤
│ crates/firmware   embassy 任务、外设驱动、业务粘合            │
│                   (不可主机测试,只能上硬件/e2e)              │
├─────────────────────────────────────────────────────────────┤
│ crates/proto      协议与纯逻辑:解码器、状态机、编解码、数学    │
│                   (零依赖,cargo test 直接在主机跑;           │
│                    fw_upg::partitions 与引导器共用)           │
├─────────────────────────────────────────────────────────────┤
│ crates/littlefs2-sys  vendored littlefs 2.11 + 手写 FFI 绑定 │
└─────────────────────────────────────────────────────────────┘
```

分层原则:**凡是能用"输入字节 → 输出字节/状态变化"表达的逻辑都下沉到 proto**,
固件层只做 I/O 和副作用。这样 C 版 `HOST_TEST` 的全部测试用例可以原样移植成
`cargo test`,协议正确性不需要烧板验证。

全系统的**数据主线是寄存器表**(proto/regmap.rs):

- 配置参数 = holding[0x00..0x12](DO、采样开关、波特率、IP、时间……)
- 采样数据 = input[0..6](版本、AI×4、DI 位图)
- 所有写路径(Modbus FC06/16、Web、UDP、Shell)都收敛到同一个
  `RegMap::io_write_holding`,副作用通过 `RegHooks` 回调触发——
  所以不管命令从哪个口进来,行为完全一致;
- 配置持久化(config_store)、历史记录(history)都是寄存器表的投影。

---

## 2. Workspace 与构建系统

`Cargo.toml`(workspace 根)三个要点:

```toml
members = ["crates/proto", "crates/firmware", "crates/littlefs2-sys"]
exclude = ["build/lfstest"]
[patch.crates-io]                     # 上游 littlefs2-sys 被本仓库版本顶替
littlefs2-sys = { path = "crates/littlefs2-sys" }
[profile.release]
opt-level = "s"                       # 面向体积优化(Flash 只有 ~448K 可用)
lto = "fat"
codegen-units = 1
```

### build.rs(crates/firmware)

构建脚本做两件事:

1. **生成 `fw_version.rs`**:读根目录 `VERSION`(当前 `0.3.0 dev`)+
   `git rev-parse --short=6` 拼出版本串 `vM.m.p_<git6>`,同时导出
   `FW_MAJOR/MINOR/PATCH`(供 Modbus 版本寄存器打包成
   `maj<<12|min<<8|patch`)和 `FW_BUILD`。注意 rerun-if-changed 监视的是
   `.git/refs/` 目录而非 HEAD 文件——commit 后 HEAD 内容不变
   (仍是 `ref: refs/heads/main`),只有分支 ref 变化。
2. **gzip 内嵌 SPA**:`assets/index.html` → gzip → `OUT_DIR/index_html.gz`,
   httpd 用 `include_bytes!` 编进镜像(C 版 `web_index_gz.h` 的等价物)。

另外它还把 `memory.x`/`ccram.x` 的字节数折进生成的常量里:cargo 不追踪
`-T` 链接脚本的变化,改了链接脚本却命中旧缓存会导致"链接成功但启动即死",
把它变成一个会过期的生成文件可以强制重链。

### 链接脚本(memory.x / ccram.x)

```
FLASH : ORIGIN = 0x08020000, LENGTH = 0x60000   /* embassy-boot ACTIVE 槽 */
RAM   : ORIGIN = 0x20000000, LENGTH = 128K
CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
```

应用直接链在 ACTIVE 槽起点(普通 bin,无镜像头、无签名步骤);片内
0x08000000..0x08020000 是引导器 BOOT 区(crates/boot,见 §4.6 的分区表)。

`ccram.x` 定义 `.ccm.bss (NOLOAD)` 段并 `INSERT AFTER .got`。两个细节:

- **锚在 .got 之后而不是 .bss 之后**:cortex-m-rt 的启动清零循环把
  `__ebss` 当作 RAM 末尾,若把段插在 .bss 后面,`__ebss` 会被推过插入的段,
  清零循环一路写穿到 0x10000000 的 CCRMAM 地址洞直接 HardFault。
  插在最后一个输出段 .got 之后则不影响 `__ebss`;
- **NOLOAD 意味着运行时不清零**,所以 main() 里要手工清零
  (见 §4.1),因为放进去的 `StaticCell` 依赖零值做 double-init 检查。

---

## 3. crates/proto:纯逻辑库(可主机测试)

`#![cfg_attr(not(test), no_std)]`:库本体 no_std,只在跑测试时用 std,
因此可以用 `std::vec::Vec` 写 mock。整个 crate **没有任何依赖**。

### 3.1 基础算法(bytes / crc / time_math / adc_math)

| 模块 | 内容 | 关键点 |
|---|---|---|
| `bytes.rs` | `get_be16/32`、`put_be16/32`、`get/put_le32` | Modbus 用 BE,盘上格式用 LE |
| `crc.rs` | `crc16_modbus`(poly 0xA001 反射,init 0xFFFF)| Modbus RTU 帧校验,"123456789" → 0x4B37 |
| | `crc32_ieee`(poly 0xEDB88320 反射)| config_store 槽校验 |
| | `crc16_ccitt_seed`(poly 0x8408 反射,init 0)| Zephyr `crc16()` 语义,升级协议;增量计算:`seed` 参数保证分段 == 整段 |
| `time_math.rs` | Hinnant civil↔unix 互转 | 合法窗口 2000-01-01..2100-01-01(`TS_MIN/MAX`),上电兜底 `TS_DEFAULT`=2020-01-01 |
| `adc_math.rs` | `ai_convert(ch, raw)` | 12bit raw → mV(3300/4096)→ 工程值。AI0/1 是 4-20mA(系数 0.7414 mA/V→7414/10000),AI2/3 是 0-10V(3704)。**刻意保持 C 的两步截断除法**,满量程金标 2445/1221 与 C 测试一致 |

### 3.2 regmap:寄存器表——全系统的单一事实源

```rust
pub struct RegMap {
    pub holding: [u16; 18],   // 参数(DO/使能/周期/CAN/RS485/IP/时间/保存/重启)
    pub input:   [u16; 6],    // 数据(版本/AI0-3/DI 位图)
}
```

18 个 holding 寄存器的地址表(`HOLDING_*_IDX`)就是外部世界看到的 Modbus 地址:
0x00=DO 输出,0x03/0x04=DI/AI 采样周期 ms,0x06/0x07=CAN id/波特率,
0x08/0x09=RS485 波特率/从站号,0x0A..0x0D=IP 四个八位组,
0x0E/0x0F=时间戳高低字,0x10=保存触发,0x11=重启触发。

三类访问接口对应三种语义:

- `get_holding/update_holding`:裸读写,**无副作用**。给采样任务写 input、
  UDP SET_IP 批量写 IP 用;
- `io_read_holding`:读侧特化——0x0E/0x0F 返回 `hooks.now_epoch()` 的实时值,
  时间戳寄存器永远"活"的;
- `io_write_holding`:写侧特化,**所有带副作用的写都走这里**:

```rust
if a == reg { return Ok(()); }        // 同值写提前返回,跳过一切副作用
match addr {
    HOLDING_DO_IDX         => hooks.set_do(reg as u8),
    HOLDING_HISTORY_ENABLE => hooks.history_enable_write(...),
    HOLDING_TIMESTAMP_LO   => hooks.set_timestamp(hi<<16 | lo),
    HOLDING_CONFIG_SAVE    => { self.holding[a]=0; hooks.holding_save(); }
    HOLDING_REBOOT_IDX     => { self.holding[a]=0;
                                if reg!=0 { sync(); reboot_cold(); } }
    ...
}
```

注意两个语义:同值写不触发副作用(避免 Web 轮询反复擦 NOR);0x10/0x11
是"触发型"寄存器,写入后自动清零,再写同值仍会再次触发。

`RegHooks` 是个全默认实现的 trait(`set_do/set_timestamp/holding_save/
history_sync/reboot_cold/now_epoch`...),主机测试给空实现或计数 mock,
固件层由 appstate.rs 的 `Hooks` 结构实现真身(§4.2)。这是整个移植里
"逻辑与 I/O 解耦"的枢纽。

`ip_addr_valid`:末字节不得为 0/255,首字节不得为 0/127/≥224——
C 版 udp_cfg.c 的白名单逐条照搬。

### 3.3 mb_server / mbtcp_adu / rtu_frame:Modbus 三层

```
TCP:  mbtcp.rs(连接管理)→ mbtcp_adu(MBAP 帧)→ mb_server(PDU)→ regmap
RTU:  rtu.rs(UART+t3.5) → rtu_frame(帧+CRC+过滤)→ mb_server(PDU)→ regmap
```

`mb_server.rs` 只看 PDU(fc + data),支持 FC01/02/03/04/05/06/08/15/16:

- 返回 0 表示**静默丢弃**(如 data 长度不对),区别于异常响应;
- FC08 维护 5 个诊断计数器(BusMsg/CrcErr/Exc/SrvMsg/NoResp),子功能
  0x000A 清零、0x000B-0x000F 读数;每进入一次 decode 自动 BusMsg+SrvMsg 各 +1;
- ≥5000 的 FP 扩展区读请求回 ILLEGAL_FC(uC/Modbus 遗留);
- FC16 保留 C 版的整数除法怪癖:`num_bytes/reg_qty != 2`(而非乘法反推)
  判长度错误——奇数长度恰好被误收的报文行为与 C 一致;
- 异常码只有三种:0x01 ILLEGAL_FC / 0x02 ILLEGAL_DATA_ADDR / 0x03 ILLEGAL_DATA_VAL。

`mbtcp_adu.rs` 处理 MBAP 外壳,顺序敏感(注释明确写出):
MBAP 长度钳位 256 → **proto != 0 先于广播检查**回 Server Device Failure
(异常码 0x04,回显原 proto)→ unit != 0 正常解码回复(回显原 unit)→
unit == 0 广播:副作用照常执行但不回复,NoResp 计数。

`rtu_frame.rs` 是 RTU 从机帧状态机:

- `rtu_t35_ms(baud)`:>19200 固定 2ms,否则按"38.5 个位时间"上取整到 ms
  (9600→5ms,19200→3ms);
- `rx_feed` 只组装不处理,溢出置标志后丢弃后续字节直到 reset;
- `t35_expired` 静默到期时一次性处理:先快照再 reset(**处理期间到达的
  字节属于下一帧**)→ 长度/溢出检查 → CRC16 校验(错:CrcErr 计数静默)→
  unit 过滤(unit≠0 且≠自己:静默)→ 送 PDU 解码 → 单播才组回复帧
  `[unit][PDU][CRC LE]`。

### 3.4 udp_cfg:UDP :8600 配置协议

命令集(首字节 cmd):已知命令 0x10 SET_IP / 0x11 GET_IP / 0x12 SET_MODBUS /
0x13 GET_MODBUS / 0x14 SET_TIME / 0x19 FACTORY_RESET / 0x05 REBOOT /
0x04 GET_VERSION;0x01-0x03/0x06 属于升级通道(由固件层拦截);未知命令
**返回 0 = 保持沉默**(不发任何包)。

核心契约写在模块头注释里——**两步重启**:

> 本层只做"擦配置 → 写回复 → 置 pending 标志";传输层必须先把回复发上线,
> 再 flush 历史、再重启(`take_reboot_pending`)。

FACTORY_RESET 有防误触的两步确认:`factory_pending_ms` 记录第一次命令时间,
5s 内的第二条才真正擦除;超过 5s 则重新计时。这里保留了 C 的一个边界怪癖:
`now_ms.wrapping_sub(pending_ms) > 5000`,开机头 5 秒内 pending_ms=0,
**单条命令立即生效**(有专门的单测 `factory_reset_single_command_quirk_within_boot_5s`
锁住这个行为)。

GET_VERSION 回复固定 14 字节拼出 `vM.m.p_git6`;跨网段只允许 GET_IP
(`udp_cmd_bcast_allowed`)。

### 3.5 config_store:A/B 槽配置编解码

盘上一个槽 40 字节(W25Q 分区内的 0xE0000(A)/0xE8000(B),各 32K):

```
[0..4)   magic "IOCF"
[4..8)   generation u32 LE     ← 高者胜出
[8..10)  len u16 LE (=26)
[10..36) IoCfg:13 个 packed u16 LE(di_en/ai_en/di_si/ai_si/his/can_id/
         can_bps/rs485_bps/slave_id/ip[4])
[36..40) crc32_ieee([0..36)) LE
```

可靠性设计:**写顺序 header → body → CRC(last)**。掉电撕写最多留下
"CRC 不匹配"的槽,对端槽仍然有效;`config_store_init` 双槽读取后
generation 高者胜、有效者胜、双无效回落出厂默认。`config_store_save_gen`
总是写到**非活动槽**再翻转指针,永不原地覆盖。

`Flash` trait(read/write/erase)让主机测试用一个模拟 NOR 语义
(write 只能清位)的 FakeFlash 覆盖撕写场景。

### 3.6 history:历史记录编码与文件名

两种定长记录(小端,PC 工具直接可解析):

- DI:10 B `[type=1 u16][timestamp u32][di_en u16][di_value u16]`
- AI:16 B `[type=2 u16][timestamp u32][ai_en u16][ai_value[4] u16]`

`make_hist_name(unix)` 产出 `data_MMDD_HHMMSS.raw`(20 字节定长数组)。
时间转换 `unix + 8*3600` —— 与 C 相同的**手动 UTC+8 偏移**,不做真实时区;
日期用 Hinnant 算法,所有分量 clamp 到合法范围(epoch 0 也不会产生乱码文件名)。

### 3.7 web_json / ws:Web 支撑

`web_json.rs` 不做完整 JSON 解析,只做平面 JSON 的字段提取:
`json_get_i32`(接受 true/false → 1/0,严格数字校验)、`json_get_str`、
`url_query_get`(a=1&b=2)。字符串值扫描会跟踪引号与转义,防止值里的逗号
截断。`history_web_name_valid` 要求 `data_` 前缀 + 白名单字符集 +
长度 6..31——这是下载/删除接口的路径穿越防线(`../etc/passwd` 直接拒绝)。

`ws.rs` 自带 SHA-1(标准向量测试)+ base64 编码,`ws_accept_key` 完成 RFC 6455
握手;`ws_frame_hdr` 编码服务端帧头(126/64 位长度分档);
`WsParser::feed` 是逐字节状态机(Header→Len16/Len64→Mask→Payload),客户端帧
强制解掩码,超长(plen > PAYLOAD_MAX=10K+16)返回 false 要求关会话。
完成一帧即回调 event,调用方同步入队、异步刷出(见 §4.7)。

### 3.8 fw_upg:升级协议原语

- **分区表 `partitions`**:embassy-boot 四分区的唯一权威定义(BOOT/ACTIVE
  片内,STATE/DFU 外置 W25Q),固件与引导器共用。页大小 =
  max(ACTIVE::ERASE_SIZE=128K 片内最大扇区, DFU::ERASE_SIZE=4K) = 128K;
  单测锁死全部 embassy-boot 不等式(ACTIVE 整扇区、DFU ≥ ACTIVE+1 页、
  STATE 容量 ≥ 2+2×页数);
- **CRC16-CCITT**(反射 0x1021/init 0,KERMIT,check 0x2189):与上位机
  助手完全一致。终验 CRC 对的是**从 DFU 槽读回的字节**而非发送侧,
  即校验的是编程结果本身;
- **KEYHASH**:`FW_KEYHASH` 是签名公钥 DER 的 SHA-256。MCUboot 时代它是
  镜像 TLV 校验;现在降级为 START 命令的"对设备"门禁(客户端可选携带,
  不匹配即拒收),不再做镜像级密码学验证;
- `b64_decode`:WS fw_start 的 keyhash 字段(44 字符 → 32 字节)。

### 3.9 ftp:PORT/EPRT 解析

纯解析函数 + 加固注释:先校验 part 数量再索引,杜绝 `PORT 1,2,3` 的
越界 panic 和 `PORT a,b,c,d,e,f,g` 的 heapless Vec 溢出——这两类输入是
**认证前**可达的,曾经能把设备打进看门狗复位。容忍尾部单个空 part
(匹配 C sscanf 的宽松风格),拒绝中途空 part 与多余数据。

---

## 4. crates/firmware:embassy 应用

### 4.1 main.rs:启动序列与任务布局

main() 开头是一段容易被误解的手写寄存器序列,原因写在注释里:
引导器(embassy-boot)跳转前屏蔽中断且 cortex-m-rt 不会清它,
而 bootloader 的跳转只清了 NVIC ICPR[0]。于是:

```text
SCB.VTOR ← 0x0802_0000          // ACTIVE 槽起点 = 我们的向量表
NVIC ICER[0..3] ← 全 1           // 关闭 bootloader 留下的使能
NVIC ICPR[0..3] ← 全 1           // 清 pending
EXTI RTSR/FTSR/IMR ← 0, PR ← 全 1 // 抹掉 loader 的 EXTI 配置
cortex_m::interrupt::enable()    // 最后才开中断
```

不开这步,一个 bootloader 留下的 pending EXTI9_5 中断会在开中断瞬间
落进我们的 DefaultHandler 死循环。

随后:`.ccm.bss` 手工清零(volatile 逐字)→ `stackmark::init()` 填栈水印
图案(必须在任何任务 spawn 前,否则会把运行时的帧也算进 boot 深度)→
`embassy_stm32::init`(时钟:HSE 13MHz ÷13 ×336 ÷2 = SYSCLK 168MHz,
APB1 42/APB2 84,LSE 走 RTC)→ `uart_raw::init()` + shell spawn →
DO8(PD7-14)+LED8 镜像(PE8-15)→ RTC 初始化 → W25Q 探测 +
`storage::NOR` 注入 + `boot_config_load()`(阻塞读两个 40B 槽,
**必须在 net 之前**,IP 来自配置)→ storage_task spawn。

之后按依赖顺序 spawn 全部任务(net::setup 需要 IP;heartbeat 需要 stack
句柄做 netmon):

| 任务 | 数量 | 文件 |
|---|---|---|
| shell_task | 1 | shell.rs |
| storage_task | 1 | storage.rs |
| net_run/net_stack/udp_task | 3 | net.rs |
| heartbeat | 1 | main.rs |
| conn_task(:502) | 2 + rejector | mbtcp.rs |
| http_task(:80) | 2 | httpd.rs |
| ftp_task(:21) | 3 + rejector | ftpd.rs |
| rtu_task | 1 | rtu.rs |
| fw_can_task | 1 | fw_can.rs |
| di_task / ai_task | 1+1 | sampling.rs |

所有 TCP/UDP socket 缓冲都以 `#[link_section = ".ccm.bss"] static ... StaticCell`
声明在 main 里(为什么放 CCRAM 见 §6.2)。

`heartbeat`(100ms Ticker)是全系统的心脏:延迟重启轮询 → 1Hz epoch tick →
每 3s 喂 IWDG(30s 窗口)→ 每 500ms netmon(断链时 DO 全灭 + REGS.DO 清零)
→ LED 300ms 亮/2700ms 灭。

panic handler 把 location 和 message 分别打进行缓冲(heapless 160B),
最后 `udf()` 停机——IWDG 30s 后自然复位。

### 4.2 appstate.rs:全局状态与 Hooks 桥接

定义四个进程级单例:

```rust
REGS       : Mutex<CriticalSectionRawMutex, RefCell<RegMap>>     // 寄存器表
UDP_STATE  : Mutex<..., RefCell<UdpCfgState>>                    // 重启 pending
MB_SERVER  : Mutex<..., RefCell<MbServer>>                       // 共享诊断计数
REBOOT_AT  : Mutex<..., RefCell<Option<u64>>>                    // 延迟重启时刻
```

`Hooks` 实现 proto 的 `RegHooks`,把副作用接到真实外设:

- `set_do` → `io_gpio::set_do_led`
- `set_timestamp` → `systime::set_timestamp`
- `holding_save` → **CTRL_QUEUE** 发 `CfgSave`(控制车道,见 §4.4——
  已应答"保存成功"的请求绝不能被塞满的历史队列丢掉)
- `history_enable_write(false)` → 发 `CloseKeepName`(关文件保名字,
  下次开启续写同一文件)
- `reboot_cold` → `reboot::cold`

`set_reboot_status(true)` 设定 now+250ms 的延迟重启,由 heartbeat 轮询执行
(Web/WS/Shell 路径走它;UDP 路径不走——它在 udp_task 内联同步重启,
理由见 §4.3)。

### 4.3 net.rs:W5500 组网与 UDP 服务

`setup()` 流程:SPI2 @21MHz(异步 DMA1_CH3/CH4)→ shared-bus SpiDevice →
`embassy_net_wiznet::new`(INT=PD1 ExtiInput,RST=PD0)→ MAC 由
96bit UID XOR 折叠 + Wiznet OUI(00:08:DC)派生 → 静态 IP/24 取自 REGS,
网关 x.y.z.1 → `StackResources<16>`(13 个活跃 socket + 余量)。

一个重要的物理约束(注释原文):**wiznet 的 `State` 必须留在主 RAM**——
它的 SPI DMA 会直接向 State 内部的包队列搬运,CCM 对 DMA2 不可达。
这与"socket 缓冲进 CCRAM"形成对照(smoltcp 的缓冲是 CPU 访问,无 DMA 约束)。

`udp_task` 绑 :8600,RX 缓冲 **16KB in CCRAM**——v2 升级窗口是 8×1400B,
突发到达速度超过 NOR 页写消化速度,窗口必须整体装下。

主循环的分发顺序体现了优先级:

1. 同网段检查(`/24`),跨网段仅放行 GET_IP;
2. `0xFA/0xFB/0xFC` 调试命令(shell RX 计数 / fw finish 诊断 / 存储 RPC 状态);
3. `fw_udp_cmd` 升级通道(START/DATA/DATA_V2/END)**内联处理**——
   START 要整槽擦除(~1s)才回复,DATA_V2 页写毫秒级;期间网络栈继续轮询;
4. 其余交给 `udp_app_cmd`(临界区内锁 REGS + UDP_STATE);
5. 回复上线后检查 `take_reboot_pending`:命中则 **本任务内联**
   Sync → 等 100ms → `system_reset()`。注释解释了为什么不在后台定时:
   后台 deadline 会留下 ~200ms 窗口,上位机轮询"是否回来"会读到旧固件的
   uptime,随后的流量死在换区中途。

`fw_udp_cmd` 的 DATA_V2(offset+data≤1400)只在 offset == received() 时写,
重复/乱序直接丢(宿主机 go-back-N 负责),且 1400B 按页拆成 ≤256B 分片、
片间 yield——整块写入会把网络轮询饿到丢窗口。

### 4.4 storage.rs:NOR 唯一所有者(本项目最核心的文件)

1412 行,承担:littlefs 文件系统、历史记录器、配置持久化、HTTP 下载 RPC、
FTP 全部文件操作。设计核心一句话:**所有 NOR 访问收敛到一个任务,
其他任务通过 Channel 发 RPC**。因为 NOR 操作忙等可达秒级(整槽擦除),
绝不能让多个任务各自摸芯片。

**两条队列**是理解本文件的钥匙:

```rust
QUEUE      : Channel<CriticalSectionRawMutex, StorageCmd, 8>  // 历史记录,满了就丢
CTRL_QUEUE : Channel<CriticalSectionRawMutex, StorageCmd, 4>  // 控制,绝不丢
```

C 版只有一个队列,历史的洪流可能把刚确认的"保存参数"挤掉——Rust 版
拆成双车道并用 `select` 同时消费,控制命令(配置保存/恢复出厂)不再可能
被挤掉。`StorageCmd` 枚举覆盖 17 种文件/配置操作:Write(HisData)/
CloseKeepName/Sync/CfgSave/CfgEraseAll/SnapReq/Del/FileOpen/FileChunk,以及 FTP 的
Stat/Ls/OpenRead/OpenWrite/WriteChunk/CloseWrite/ReadChunk/Remove/Mkdir/Rename,
外加升级 RPC:FwBegin(整槽擦除)/FwProg(写 256B 页)/FwRead(读 256B)/
FwMarkUpdated / FwMarkBooted——embassy-boot 的 updater 对象在 storage 任务内
构造,通过零尺寸适配器(FwDfuNor/FwStateNor)访问外部 DFU/STATE 分区,
每次硬件操作各自拿 NOR 锁(与 LfsNor 同款模式)。

**RPC 应答模式**:没有回调,而是共享结果寄存器 + 代际号:

```rust
pub static RPC_SEQ: AtomicU32;                 // 每处理完一条命令 +1
pub static FTP_RES:  Mutex<..., (bool,bool,u32)>;   // (ok, is_dir, size)
// 调用方(httpd/ftpd):
let seq = RPC_SEQ.load();
QUEUE.try_send(cmd).ok();
while RPC_SEQ.load() <= seq { /* 2ms 轮询,2500ms 超时 */ }
```

注释解释了为什么不用 embassy Signal:Signal 会锁存旧值导致立即假醒,
代际号比较没有这个问题。

**littlefs 挂载**:`LfsNor` 实现 `Storage` trait(READ/WRITE_SIZE=16,
BLOCK_SIZE=4096,BLOCK_CYCLES=512 磨损均衡,CACHE_SIZE=1KB),offset
0xF0000 起(与 C 版盘上布局一致)。mount 失败自动 format 再 mount;
再失败进入"只服务配置命令"的降级循环,历史离线但参数还能存。

**持久句柄 + 稳定地址分配**是本文件最精细的部分:

- littlefs 把缓存指针嵌进打开的 `lfs_file_t`,句柄结构**移动即悬垂**;
  于是 `AllocCell` 用 `UnsafeCell<MaybeUninit<FileAllocation>>` +
  init 原子标志把每个文件的分配固定在静态地址,`alloc_get` 保证只初始化一次;
- 常开的 `OPEN_FILE`(历史文件)、`DOWNLOAD_FILE`(HTTP 分块下载)、
  `FTP_XFER[3]`(FTP 会话槽)持有这些句柄;
- 每次长 NOR 操作(close/sync/read/write)前**把 File 从互斥锁里 take 出来**,
  操作完再放回去——临界区只保护指针交换,几百 ms 的 flash 操作期间
  不屏蔽中断(否则 CAN 3 深 FIFO 必丢帧)。这是全文反复出现的模式,
  例如 `hist_write`、`file_chunk`、`ftp_read_chunk`。

**失败围栏**:`close_or_poison`——失败的 close 会让文件节点残留在
littlefs 的 mlist 里,复用它的 alloc cell 会让节点自环、下次 commit 的
mlist 遍历死旋(存储任务连同排队的 CfgSave 一起卡死)。一旦 close 失败置
`FS_BAD`,之后所有 open 拒绝直到重启:"failing beats hanging"。
同理 `slot_close_all` 在复用 FTP 槽前把 dl/wr 两类句柄都关闭,
避免"已链接节点再次 open"的自环。

**历史写入路径**(`hist_write`)的性能语义与 C 严格对齐:

- 快路径:文件已开且 <1MB → 一次 buffered write,**不逐条 sync**
  (每次 sync 都会把内联数据重新提交成一个新 tag,~8 条记录就填满目录
  触发 compaction 擦除风暴——这正是 C 版 his_file_write 的行为);
- 轮转路径:文件不存在/超限 → 保留名重试 → 找最新 `data_*.raw` →
  都不行才按当前时间建新文件,并触发保留清理(`cleanup_old_files`,
  文件名序即时间序,超出 10 个删最老);
- 升级流传输期间(`fw::active()`)暂停写历史——轮转擦除会冻中断数秒,
  W5500 MACRAW 缓冲会丢升级窗口。

**HTTP 下载**(`FileOpen/FileChunk`)与 FTP 读(`FtpReadChunk`)都是
"持久句柄顺序读":512B/2048B 块,避免每块 reopen+seek 的 O(n²)。

### 4.5 w25q.rs:NOR 驱动

SPI1 阻塞轮询 @42MHz(无 DMA 无中断——驱动被 ThreadModeRawMutex 保护,
见 §6.1)。JEDEC ID 0xEF4018 校验失败直接不初始化。

值得注意的实现细节:

- `wait_not_busy` 每 ~1ms 轮询 SR1,**顺带向 IWDG KR(0x4000_3000)写
  0xAAAA 喂狗**——整分区 format 要几分钟,不喂狗必复位;
- erase 自动选大块:64K 对齐用 D8、32K 对齐用 52、否则 4K sector(0x20),
  超时按 datasheet 上限 ×5~8(400ms→2s 等);
- write 强制 ≤256B 且不跨页(NOR 页编程约束),调用方(LfsNor::write)
  负责按页切分。

### 4.6 升级体系:embassy-boot 引导器 + fw.rs 会话 + fw_can.rs CAN 通道

**引导器(crates/boot,独立 bin)**:embassy-boot 库 + 两块 flash 的适配器。
分区几何定义在 proto::fw_upg::partitions(§3.8),固件与引导器共用:

| 分区 | 介质 | 地址 | 大小 |
|---|---|---|---|
| BOOTLOADER | 片内 | 0x08000000 | 128K(实际 ~8K) |
| ACTIVE | 片内 | 0x08020000 | 384K = 三个 128K 整扇区 |
| STATE | W25Q | 0x000000 | 4K |
| DFU | W25Q | 0x001000 | 512K |

关键约束与决策:

- **页 = 128K**:embassy-boot 页大小取 max(ACTIVE::ERASE_SIZE,
  DFU::ERASE_SIZE),而 F407 内部 flash 扇区非均匀(16K/64K/128K)、HAL 上报
  ERASE_SIZE=最大值。若页小于物理扇区,换区算法擦一页会毁掉同扇区邻居
  (它们还没备份到 DFU)——直接变砖。所以 ACTIVE 必须取整扇区倍数,
  让每个 128K 页恰好对应一个物理扇区;DFU ≥ ACTIVE+1 页 → 512K;
- **prepare_boot 缓冲只需整除页大小**(拷贝循环按缓冲步进):用 4KB 静态
  缓冲即可,不需要 128K;
- **STATE 不要求页对齐**,只要求容量/WRITE_SIZE ≥ 2+2×ACTIVE 页数(8 字节);
- **IWDG 跨复位运行**:应用已 unleash 的看门狗在软复位后继续计数,引导器
  里每次长操作都喂狗(W25Q 忙等轮询内联写 KR;内部扇区擦除前后各喂一次);
- **外部 flash 缺失不挡启动**:JEDEC 校验失败就跳过整个 swap 机构直接
  引导 ACTIVE——宁可放弃 OTA 能力也不变砖;
- 跳转前关中断 + 清 NVIC ICER/ICPR(应用的启动序列依赖这个契约)。

**fw.rs 会话层**:三条通道(UDP v2、WebSocket 二进制、CAN)共用一个会话,
flash 操作全部走 storage 任务 RPC(FwBegin/FwProg/FwRead/FwMarkUpdated):

```rust
static FW:   Mutex<ThreadModeRawMutex, RefCell<Sess>>;   // active/total/received/页缓冲
static RING: Mutex<CriticalSectionRawMutex, RefCell<Ring>>; // 待编程页环 ×12
```

- `start(total, keyhash)`(**async**):忙(-3)/尺寸非法(-1)/keyhash
  不匹配(-2)/擦除失败(-1)。FwBegin 让 updater `prepare_update()` 整槽
  擦除(~1-2s,与 MCUboot 时代语义一致);
- `write(data)`(**同步!**):只做 256B 页缓冲 + 入待编程环,不碰 flash——
  WS 解析回调是同步的,二进制帧在这里无阻塞入队;环满 = 传输失败;
- `flush()`(**async**):把环逐页经 FwProg RPC 写入 DFU(updater 的懒擦除
  此时不会触发,槽已在 START 擦净);UDP/CAN 每包后调用,WS 在读事件臂调用;
- `finish(crc)`(**async**):先 flush → received==total 校验 → **256B 一块
  从 DFU 读回算 CRC16**(校验的是编程结果而非发送侧)→ 有期望 CRC 则比较
  → FwMarkUpdated 置 SWAP_MAGIC,下次重启引导器交换。诊断全程写 `FW_DBG`
  (UDP 0xFB 可读:阶段码/写入量/计算与期望 CRC)。

**试用启动与回滚**: embassy-boot 无 test/permanent 之分——新固件总是试用
启动,应用在心跳任务里开机 ~10s 后发 FwMarkBooted 确认(存储任务 handler
仅在 state==Swap 时执行 mark_booted);10s 内没跑到确认(panic 循环、
砖化驱动等)则下次复位自动回滚旧版。UDP END 的 permanent 字节和 CAN
CONFIRM 的 arg 保留在线格式里但不再影响语义。

`fw_can.rs`(bxCAN 通道)的非显而易见决策:

- **TX 不用 BufferedCanTx 而用裸 `CanTx::write`**:embassy 0.6 的 buffered TX
  在邮箱里有同 ID 帧挂起时会拿到 WouldBlock 并**丢弃 ring 里已出队的帧**
  ——第 2 个 0x105 版本帧就这样被静默吞掉。裸 write 会停在 WouldBlock
  直到邮箱空中断,与 C 的 mailbox 等待语义相同;
- **RX 用 ISR 供数的 128 帧 RxBuf**:硬件 FIFO 只有 3 深,NOR 页写的
  几毫秒里宿主机突发的 512B 窗口必然溢出;
- 过滤器组 0/FIFO0 两个半槽:A=精确业务 ID(mask 0x7FF),
  B=0x100-0x1FF 升级段(mask 0x700);
- 波特率快照自寄存器,50k-1000k 支持,**800k 不可实现**(PCLK1 42MHz
  算不出位时序),回落 250k;
- 协议:0x101 命令(START 无条件重开,keyhash 仅当 0x104 五片凑齐才校验;
  CONFIRM 不带 CRC——读回校验仍在,CRC 门禁交给试用回滚;REBOOT 不回帧,
  排水 100ms → Sync → 500ms → 复位)/ 0x102 回复 / 0x103 数据 /
  每 64B 一个 OFFSET 流控(Zephyr 权威语义)。

### 4.7 httpd.rs / ftpd.rs / mbtcp.rs:TCP 服务三件套

**httpd.rs**(1122 行,pool_size=2):

- 连接循环:accept → serve(keep-alive 循环)→ abort。socket 超时 75s
  (>60s keep-alive idle);serve 内部用 select 实现"半请求 5s/空闲 60s"
  双超时;
- 手写 HTTP 解析:请求行按 `%7s %95s` 语义截断(超长 path 得 404 而非 400,
  与 C sscanf 行为一致);pipelined 请求在同一缓冲里循环消费;
  POST body 上限 128B;
- `/ws` 升级:`WS_ACTIVE.compare_exchange` 在**任何 await 之前**抢占单会话名额
  (101 写出会 yield,晚加锁会被第二个 upgrade 竞争穿过),抢不到回 503
  并等 50ms 让 503 被 ACK 再 abort(RST 吃掉未 ACK 数据客户端就看不到错误);
- WS 会话:`select3(sock.read, 1s push ticker, 10s info ticker)`;
  帧解析在同步回调里入队(heapless Vec<768>),async 循环统一 flush;
  文本帧是 JSON 命令(do/reg/time/cfg/save 同步执行;fw_start/fw_end 只记入
  `WsPlan` 由会话循环异步执行——它们要 await 存储 RPC),二进制帧同步灌
  `fw::write`(纯内存页环)并在读事件后 `fw::flush().await` 落盘——
  第三条升级通道;
- REST API 表:`GET /api/info|io|regs|history|history/download?name=`,
  `POST /api/do|reg|time|save|reboot|cfg|history/delete`;未知路径按
  "已知路径错方法→405,否则 404";
- 存储交互全部走 §4.4 的 RPC(try_send + rpc_wait 代际号等待),
  `/api/history/download` 逐块拉 FILE_DL.chunk 直到 eof;
- JSON 构建用 heapless String + write!,容量 704 的 info 缓冲是 C httpd
  的 body cap,刻意保留以得到字节级相同的 JSON 截断行为。

**ftpd.rs**(831 行,pool_size=3 + rejector):

- 会话槽计数 `FTP_BUSY` 门控 421 拒绝者:只在满载时 arm,**负载下降 50ms
  内 disarm**——常驻监听会偷走会话结束后的合法重连(mbtcp rejector
  踩过同一个坑,注释互相引用);
- PASV/EPSV:端口从 40000+ 轮转;**227 回复发出的同一时刻就用 poll_once
  预挂 accept**——CPython ftplib 收到回复立刻连,不等传输命令;
- PORT/EPRT 解析委托 proto(§3.9);路径归一 `norm_path` 用栈式 parts
  处理 ".."(不允许逃出根);
- 权限:admin/admin 可写,anonymous/ftp 只读;**鉴权在 RPC 入队之前**
  (存储任务会执行每条入队命令,事后检查只能改变回复内容,拦不住实际删除);
- RETR:150 → open_data → 循环 FtpReadChunk RPC → TYPE A 时 \n→\r\n →
  结束后 **data.close()(优雅 FIN)而不是 abort**——客户端最后一次 read
  必须看到 EOF 而不是 RST;读失败置 err,回 426 而不是假 226;
- STOR/APPE:512B staging 经 FtpWriteChunk RPC 落盘,werr 传播为
  451(不给截断文件回 226);TYPE A 时剥离 \r 并用 pending_cr 处理
  跨块的 \r\n 对。

**mbtcp.rs**(142 行):2 个 conn_task(pool_size=2)+ rejector(accept 后
立即 abort——客户端 connect 成功但请求死于 RST,与 C 版第 3 连接行为相同)。
帧组装按 MBAP 长度字段精确取齐(钳位 256),半帧 500ms 超时;
一次 read 到多条 pipelined ADU 时只拷贝当前帧所需字节,剩余留给下一轮。
诊断计数与 RTU 共享同一个全局 MbServer。

### 4.8 shell.rs / uart_raw.rs / log.rs:控制台三件套

**uart_raw.rs** 为什么存在(模块头注释):logger 需要在临界区里可用的
同步 TX(TXE 寄存器轮询);shell RX 必须在 NOR 操作的毫秒级 PRIMASK 冻结下
存活——寄存器级 RXNE 中断只有 1 字节 DR,冻结期间必丢,而 DMA2 circular
ring 在硬件层面持续搬 DR。所以:

- TX:`write()` 纯寄存器轮询,临界区内安全;
- RX:DMA2 stream2/ch4 循环写 `RX_BUF[256]`,`rx_available(tail)` 用
  NDTR 硬件寄存器算 head,shell 任务 2ms 轮询 `getchar`——冻结只是增加
  延迟,字节早已被 DMA 收走。

**log.rs**:行格式 `[HH:MM:SS.mmm][I/W/E] msg`,时间取本地时区
(systime::now_epoch_local = UTC+8);`line/raw` 无时间戳,给 shell 用,
保证 shell 输出永不与日志行交错(同一出口)。

**shell.rs**(1208 行,与 C 版 sh.c 1:1 对齐):

- 行编辑:echo、退格(0x08/0x7F)、光标左右(\x1b[C/D)、行中插入/删除
  (copy_within 平移 + redraw)、ESC 序列小状态机、CRLF 收敛(prev_cr);
- 历史 8 条环形,↑↓ 在草稿(draft)与历史间往返,连续重复不入库;
- Tab 补全:静态命令树(ROOT_CMDS → io 子树 → rs485/can 叶子),
  `complete_level` 走树定位尾词所在层级;唯一候选直接补全加空格,
  多候选补最长公共前缀并列出;参数区(off-tree)不补全;
- 命令:`help` / `tasks`(=`ps`,见 §4.10)/ `reboot`(Sync + 延迟重启)/
  `io` 子命令(info/di/do set/ai/rs485 baud|sid/can id|bps/ip/reg/save/factory,
  写寄存器一律走 `io_write_holding` 带副作用,提示"reboot to apply");
- 全部输出经 heapless String 格式化,`parse_u32` 支持 0x 前缀。

### 4.9 sampling.rs / io_gpio.rs / systime.rs:IO 与时间

**sampling.rs**:两个独立任务。

- `di_task`:16 个 Input(Pull::Down,高有效)按通道序扫一遍组成位图 →
  `update_input(INPUT_DI_IDX)`;任一通道使能就投递 DI 历史记录;
- `ai_task`:ADC1 IN10-13(PC0-3),144 周期采样,`ai_convert` 转工程值
  写 input 1-4;任一位使能投递 AI 记录;
- 间隔取寄存器 0x03/0x04,clamp [10,5000]ms(0-9 视作 10)。

历史记录入 QUEUE(`try_send`,满即弃——与 C history 队列语义一致)。

**io_gpio.rs**:DO8 + LED8 两组 Output,`drive` 按位展开;`set_do_led(val)`
在临界区内同时驱动两组(Hooks.set_do 的落地)。

**systime.rs**:epoch 的"RTC + RAM 缓存"混合方案——boot 时从 RTC(VBAT
维持)读一次转 epoch 存原子量,之后靠 heartbeat 的 1Hz tick 递增;
`set_timestamp` 同时写 RTC 和缓存。为什么不每次问 RTC:RTC 读要走备份域
访问序列,而时间戳寄存器是 FC03/JSON 的热路径。显示用 UTC+8
(`LOCAL_OFFSET_SECS`),RTC 本体保持 UTC。

### 4.10 reboot.rs / stackmark.rs:辅助设施

**reboot.rs**:延迟重启的极简实现——`DEADLINE_MS: AtomicU32`(0=无),
`due()` 用 `wrapping_sub` 距离判断到期且**一次性**(到期即清零),
对 32bit 毫秒回绕安全。`cold()`=100ms(web/UDP 回复已在路上);
heartbeat 每 100ms 轮询一次。`graceful()/cancel()` 目前无调用方
(保留 API,#[allow(dead_code)])。

**stackmark.rs** 回答一个问题:embassy 没有 per-task 栈,也没有运行时任务
注册表(池子里是匿名 future),"tasks/ps" 打什么?

- 物理上只有**一条共享栈**([_stack_end, _stack_start)),
  每个 task 的 poll 深度不同;
- `probe(name)`:任务在自己循环里调用(几十条指令),首次以 'static str
  指针相等注册(MAX_TASKS=24),记录该任务见过的最低 MSP 与迭代次数;
- `usage()`:0xA5A5A5A5 图案扫描(init 时填充)得全栈水印——**权威值**,
  因为它包含 IRQ 帧和 probe 点以下的 C-FFI 深度;
- `tasks` 命令打印注册表 + 图案扫描 + RAM 台账(statics + stack = 128K,
  ccm used/total)。probe 还有一个妙用:`nor_with` 里也调了一次,
  把 NOR 忙等的深度记到 storage 任务头上。

---

## 5. crates/littlefs2-sys:vendored littlefs

upstream `littlefs2` crate 依赖 `littlefs2-sys`(bindgen 生成)。本项目
`[patch.crates-io]` 用仓库内的版本替换它:

- `littlefs/`:lfs.c/lfs.h/lfs_util.* 原样 vendored(v2.11,与 C 仓库
  deps/littlefs 同版——**盘上兼容的前提**);
- `src/bindings.rs`:**手写**(非 bindgen)FFI,符号命名遵循 bindgen
  惯例(`lfs_error_LFS_ERR_IO` 等),让 littlefs2 crate 里的 `ll::` 引用
  原样解析,不用 fork littlefs2 本身;
- `src/lib.rs`:lfs.c 链接所需的 minimal libc 字符串函数
  (`strlen/strchr/strspn/strcspn`,C99 语义),`#[cfg(target_os="none")]`
  只在裸机目标编译(host 测试/lfstest 用系统 libc,避免 LNK2005 重复符号);
  `strlen` 返回类型必须是 `usize`(rustc 对标准库保留符号做签名检查,
  写 `c_ulong` 会告警)。
- `build.rs`:`cc` 编译 lfs.c,按 feature 传宏(assertions/trace 等)。

---

## 6. 横切设计:并发模型与内存布局

### 6.1 并发模型:三档互斥

单核单 executor,所有任务协作调度,ISR 只做 embassy 胶水。由此推出
三档保护策略,**选型的依据是"持锁时长是否会饿死中断"**:

| 档位 | 类型 | 适用 | 典型用户 |
|---|---|---|---|
| 短临界区 | `CriticalSectionRawMutex`(PRIMASK) | 微秒级指针交换/寄存器表读写 | REGS、OPEN_FILE、各结果寄存器 |
| 线程模式锁 | `ThreadModeRawMutex`(不上锁,靠协作调度) | 毫秒~秒级的忙等操作 | `storage::NOR`、`fw::FW` |
| 异步互斥 | `embassy_sync::mutex::AsyncMutex` | 需要 await 的共享资源 | W5500 SPI 总线 |

ThreadModeRawMutex 用于 NOR 的论证(代码注释原文概括):所有调用方都是
线程模式的 embassy 任务,闭包内无 await(不会被调度切换打断),无 ISR
碰 NOR——所以"不加锁"是可靠的;反之若用 PRIMASK 保护一次 2s 的整槽擦除,
bxCAN 3 深 FIFO 和 W5500 MACRAW 都会溢出。

第二层模式:**长操作前把句柄 take 出锁外**(§4.4 反复出现),
让临界区只覆盖指针交换。

第三层约定:**能放进 StorageCmd 的就不共享文件系统**。文件系统只有
storage_task 一个使用者,天然免锁。

### 6.2 内存布局

```
主 RAM 128K:  statics(.data/.bss) ↑ 向下生长
              共享栈(stackmark 管理) ↓ 从顶部向下
              两者之间的图案区间就是"剩余"

CCRAM 64K:    .ccm.bss(NOLOAD,main 手工清零)
              ├─ 全部 smoltcp socket 缓冲(HTTP/FTP/Modbus/UDP 16K RX)
              ├─ StackResources<16>
              └─ StaticCell 元数据
```

分工原则:**DMA 要碰的一律主 RAM**(wiznet State 的包队列),
CPU 独享的缓冲尽量挤进 CCRAM 给主 RAM 的栈留余量。
`.ccm.bss` 的 NOLOAD 特性要求使用方在使用前自行初始化
(StaticCell::init 正好满足),main() 的清零只是给 StaticCell 的
double-init 检查提供零值前提。

### 6.3 错误处理的统一哲学

- **fail loud**:W25Q JEDEC 不匹配 → 不初始化;w5500 init 失败 → 日志 +
  死循环;spawn 失败 → expect panic(IWDG 兜底复位);
- **fail safe 而非 hang**:littlefs close 失败 → FS_BAD 围栏拒绝后续 open
  (宁可历史离线,不可存储任务卡死拖垮配置保存);
- **降级服务**:littlefs 彻底挂掉 → 只服务配置命令的循环;
- **静默丢弃要有计数**:Modbus 各种静默路径全部计入 NoResp/CrcErr,
  FC08 可查。

---

## 7. 从 C 移植时刻意保留的行为怪癖清单

这些"看起来像 bug"的行为都有 e2e 或单测锁定,改动会破坏与 C 版/上位机的
互操作(详见各模块单测):

| 怪癖 | 位置 | 锁定方式 |
|---|---|---|
| FACTORY_RESET 开机 5s 内单命令即生效(wrapping_sub 比较) | proto/udp_cfg.rs | `factory_reset_single_command_quirk_within_boot_5s` |
| FC16 长度校验用整数除法 `num_bytes/reg_qty != 2` | proto/mb_server.rs | `fc16_write_regs_with_quirks` |
| FACTORY_RESET 开机 5s 内单命令即生效(wrapping_sub 比较) | proto/udp_cfg.rs | `factory_reset_single_command_quirk_within_boot_5s` |
| FC16 长度校验用整数除法 `num_bytes/reg_qty != 2` | proto/mb_server.rs | `fc16_write_regs_with_quirks` |
| FP 区(≥5000)读回 ILLEGAL_FC 而非 DATA_ADDR | proto/mb_server.rs | `fc03_qty_violation_fp_area_exc1` |
| proto != 0 的 server failure 早于广播检查处理 | proto/mbtcp_adu.rs | `proto_nonzero_server_failure` |
| 历史文件名用手动 +8h 而非真时区 | proto/history.rs | `name_matches_c_format` |
| AI 换算的两步截断除法(满量程 2445/1221 金标) | proto/adc_math.rs | `full_scale` |
| 超长 URL 截断到 95 字节后 404(sscanf %95s 语义) | firmware/httpd.rs | parse_request_line 注释 |
| info JSON 的 704B 缓冲上限(截断行为一致) | firmware/httpd.rs | build_info_json |
| 历史记录 buffered write 不逐条 sync | firmware/storage.rs | hist_write 注释(erase storm) |
| UDP END 的 permanent 字节保留但不改变语义(一律试用启动) | firmware/net.rs | fw_udp_cmd 注释 |
