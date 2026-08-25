# io-edge-hub-rust 的 embassy 架构文档(按子模块)

本文档描述本固件对 [embassy](https://embassy.dev) 异步嵌入式框架的使用方式,按源码子模块逐一展开:
每个模块用到了哪些 embassy API、为什么这样选、和 C(FreeRTOS)版的对应关系,以及实测踩过的坑。
所有结论都来自本项目的真实代码与硬件验证(LCKFB STM32F407VET6 + W5500 + W25Q128 + PCAN)。

## 目录

1. [依赖版本与总体架构](#1-依赖版本与总体架构)
2. [执行模型:executor、task 与中断](#2-执行模型executortask-与中断)
3. [时间系统(embassy-time)](#3-时间系统embassy-time)
4. [同步原语(embassy-sync)与 critical_section —— 本项目最重要的一节](#4-同步原语与-critical_section)
5. [main.rs:时钟树、启动、任务布局、心跳](#5-mainrs)
6. [net.rs:W5500 + embassy-net 网络栈](#6-netrs-w5500--embassy-net)
7. [httpd.rs / ftpd.rs / mbtcp.rs:TCP 服务三件套](#7-httpd--ftpd--mbtcp-tcp-服务三件套)
8. [storage.rs + w25q.rs:NOR、littlefs 与存储任务](#8-storagers--w25qrs)
9. [fw.rs:固件升级会话](#9-fwrs固件升级会话)
10. [fw_can.rs:bxCAN 驱动与升级协议](#10-fw_canrsbxcan)
11. [rtu.rs:Modbus RTU(UART + DMA)](#11-rtursmodbus-rtu)
12. [uart_raw.rs + log.rs + shell.rs:控制台](#12-uart_raw--log--shell)
13. [sampling.rs + io_gpio.rs:DI/DO/AI 采样](#13-sampling--io_gpio)
14. [systime.rs:RTC 系统时间](#14-systimersrtc)
15. [reboot.rs + appstate.rs:延迟重启与全局状态](#15-reboot--appstate)
16. [内存布局:memory.x 与 CCRAM](#16-内存布局)
17. [踩坑记录(实测,带根因与修复)](#17-踩坑记录)

---

## 1. 依赖版本与总体架构

```toml
embassy-stm32   = "0.6.0"   # 特性: stm32f407ve, time-driver-any, exti
embassy-executor= "0.10.0"  # platform-cortex-m + executor-thread + executor-interrupt
embassy-time    = "0.5.0"
embassy-sync    = "0.8.0"
embassy-futures = "0.1.1"   # select / poll_once / yield_now
embassy-net     = "0.9.0"   # proto-ipv4, medium-ethernet, tcp, udp
embassy-net-wiznet = "0.3.0"# W5500 MACRAW 设备
embassy-embedded-hal = "0.6.0"  # 共享 SPI 总线的异步 SpiDevice
static_cell     = "2"       # 'static 单例化
```

架构一句话:**单核单 executor、全部任务线程模式协作调度;硬件中断只跑 embassy 的 ISR 胶水
(waker / 环形缓冲搬运),不做业务**。C 版是"ISR 收帧入队 + FreeRTOS 任务消费",Rust 版对应
"embassy ISR 唤醒/搬运 + 异步任务消费"。

任务清单(全部由 `main` spawn,详见 [§5](#5-mainrs)):

| 任务 | 数量 | 职责 | 关键 embassy 设施 |
|---|---|---|---|
| `shell_task` | 1 | 控制台行编辑 | 裸寄存器 UART(见 §12) |
| `storage_task` | 1 | littlefs/配置/NOR 唯一所有者 | `Channel` 命令队列 |
| `heartbeat` | 1 | IWDG/LED/netmon/延迟重启 | `Ticker` 100ms |
| `net` runner(runner_task) | 1 | W5500↔smoltcp 报文泵 | `embassy-net-wiznet` |
| `udp_task`(net.rs) | 1 | UDP :8600 配置/升级服务 | `UdpSocket` |
| `http_task` | 2(pool_size=2) | HTTP :80 | `TcpSocket` |
| `ftp_task` + `ftp_reject_task` | 3+1 | FTP :21 | `TcpSocket`×2/任务 |
| `conn_task` + `reject_task` | 2+1 | Modbus TCP :502 | `TcpSocket` |
| `rtu_task` | 1 | RS485 | `Uart` + DMA + `read_until_idle` |
| `fw_can_task` | 1 | CAN 升级通道 | bxCAN buffered RX + 直写 TX |
| `di_task` / `ai_task` | 1+1 | DI16 / AI4 采样 | `Input` / `Adc` + `Timer` |

---

## 2. 执行模型:executor、task 与中断

### 2.1 任务声明与 spawn

```rust
#[embassy_executor::task]
pub async fn rtu_task(p: RtuPins) { ... }

// 多实例:pool_size 让同一任务体有 N 份独立的静态存储
#[embassy_executor::task(pool_size = 2)]
pub async fn conn_task(stack: Stack<'static>, rx_buf: &'static mut [u8; 512], ...) { ... }
```

- 任务函数被宏改写为状态机,`spawn` 时任务体被放进 executor 的静态存储;
  **任务参数按值移动进任务**,大缓冲区因此以 `&'static mut [u8; N]` 传入
  (见 §5 的 `StaticCell` 用法)。
- 同一任务多个实例**必须** `pool_size ≥ 实例数`;不写则只允许 spawn 一次。
  `pool_size` 的每一份存储独立,因此 `mbtcp::conn_task` 两个实例的缓冲互不干扰。
- spawn 失败(池满)返回 `Err`,本项目统一 `.expect("spawn xxx")`——启动期失败即 bug。

### 2.2 `#[embassy_executor::main]`

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) { ... }
```

cortex-m 上该宏展开为:定义 `VectorTable`、以 cortex-m-rt 启动、创建线程模式 executor、
把 `main` 本身作为第一个任务 spawn。`main` 里的顺序即任务启动顺序;

`embassy_stm32::init(board_config())` 必须最先调用(使能时钟、绑定外设单例),
之后才能 `dp.PA5` 这类外设引用。

### 2.3 中断绑定:`bind_interrupts!`

embassy 不改写向量表内容,而是把向量表里的 IRQ handler 指到 embassy 的 typelevel handler:

```rust
bind_interrupts! {
    struct Irqs {
        DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
        USART2       => usart::InterruptHandler<peripherals::USART2>;
    }
}
// 使用处把 Irqs 传给驱动构造:
let uart = Uart::new(p.usart2, p.rx, p.tx, p.tx_dma, p.rx_dma, Irqs, cfg);
```

规则与经验:

- **一个 IRQn 只能绑定一个 handler**;同一流(如 DMA1_STREAM5)被多个驱动用时编译期报错,
  需要自己写组合 handler(本项目未遇到,W5500 的 DMA 流与 RTU 的流不冲突)。
- 每个 `bind_interrupts!` 生成一个独立的 struct,不同外设各建各的(本仓库有 `Irqs`/`CanIrqs` 多组)。
- ISR 内部只做两件事:搬数据到环形缓冲(`Channel::try_send`)或登记 waker。
  **业务逻辑绝不进 ISR** —— embassy 驱动的 ISR 也遵守这一点。

### 2.4 合作式调度的含义(必须内化)

单 executor 无抢占:任务只在 `.await` 挂起点让出。推论:

1. **同步代码段(无 await)再长也不会被别的任务打断**,所以"任务内 + 无 await 的闭包"
   只需防中断,不需防任务 —— 这是 §4 ThreadModeRawMutex 安全性论证的一半。
2. **`critical_section::with` 在本平台 = PRIMASK 置位,屏蔽所有可屏蔽中断**。
   长操作( NOR 擦写毫秒~秒级)放进临界区 = CAN/W5500/定时器中断全部饿死 ——
   本项目真实事故见 §17-坑 1。原则:**临界区只包几十个周期的寄存器操作**。
3. ISR 可以打断任务,但 embassy 驱动的 ISR 都极短,不构成调度问题;
   反过来,任务里的长临界区才是杀手。

---

## 3. 时间系统(embassy-time)

- 特性 `time-driver-any`:embassy-stm32 自动挑一个硬件定时器做全局 tick 驱动,
  `Instant::now()` / `Timer` / `Ticker` 全部可用;`embassy_time::TICK_HZ` 是编译期常数。
- `Timer::after_millis(n).await` —— 一次性延时,常用作退避/超时臂:
  ```rust
  if sock.accept(PORT).await.is_err() { Timer::after_millis(100).await; continue; }
  ```
- `Ticker::every(Duration::from_millis(100))` —— 周期节拍,`ticker.next().await` 消费。
  心跳任务用一个 100ms Ticker 派生出全部慢节拍(`ticks % 5` netmon、`% 10` 1Hz、`% 30` 喂狗),
  与 C 版单循环多计时语义一致。
- `Instant::now()` —— 单调钟,用于空闲判定(httpd 的 rx_idle、ftpd 会话计时)与
  `reboot.rs` 的 wraparound 安全 deadline 比较:
  ```rust
  // wraparound 安全的到期判定
  if now_ms().wrapping_sub(d) < 0x8000_0000 { /* due */ }
  ```
- **陷阱**:embassy 的周期任务里别用 `Timer::after` 循环拼周期(会累积漂移),要 `Ticker`。

---

## 4. 同步原语与 critical_section

本项目同时用了三种 raw mutex 与两种消息原语,选择逻辑是全文档最重要的工程结论。

### 4.1 三种 RawMutex 的选择表

| 原语 | 保护对象 | 保护内容最长耗时 | 为什么安全 |
|---|---|---|---|
| `CriticalSectionRawMutex` | `REGS`、`UDP_STATE`、`MB_SERVER`、`CFG`、`OPEN_FILE.take/put`、`FTP_XFER`、`LINK_UP`… | 寄存器数学/Option 搬移,微秒级 | PRIMASK 临界区短,中断延迟可忽略 |
| `ThreadModeRawMutex` | `storage::NOR`(W25Q 驱动)、`fw::FW`(升级会话) | NOR 页编程 0.4-3ms、整槽擦除 ~2s、整镜像回读 ~百 ms | 见下节论证 |
| `NoopRawMutex` | W5500 共享 SPI 总线的异步 Mutex | —— | 总线由 async Mutex 排队,Noop 只当类型占位 |

### 4.2 ThreadModeRawMutex 的安全性论证(NOR / 升级会话)

```rust
// storage.rs —— 为什么不是 CriticalSection:
/// NOR access is serialized with ThreadModeRawMutex, NOT a critical section:
/// SPI operations run 10-100+ ms (erase) and masking interrupts that long
/// drops CAN frames (3-deep FIFO) and W5500 traffic. Sound because all
/// callers are embassy tasks in thread mode on the single core — the closure
/// contains no await — and no ISR touches the NOR.
pub static NOR: Mutex<ThreadModeRawMutex, RefCell<Option<W25q>>> = ...;

pub fn nor_with<R>(f: impl FnOnce(&mut W25q) -> R) -> Option<R> {
    NOR.lock(|r| r.borrow_mut().as_mut().map(f))   // 无 critical_section 包裹
}
```

三个成立条件(缺一不可,新代码照抄前必须逐条核对):

1. **所有调用方都在线程模式(任务上下文)**。`ThreadModeRawMutex::lock` 在中断上下文调用会
   panic(assert thread mode)。本项目的 NOR/升级会话调用方:storage 任务、udp 任务、
   http 任务、fw_can 任务 —— 全部是任务。
2. **闭包内无 `await`**。无 await = 闭包原子执行 = 不存在"任务 A 持锁挂起、任务 B 重入"。
   embassy 协作调度下这是编译器 + 人工审查共同保证(审查点:闭包里只能调同步函数)。
3. **没有 ISR 触碰该状态**。CAN/DMA/EXTI 中断都不碰 NOR 与 FW 会话。

附带的类型要求:驱动句柄要进 `static`,必须 `unsafe impl Send/Sync`
(理由同上,见 `w25q.rs` 的 SAFETY 注释)。

`fw.rs` 是同一论证的第二个实例:升级会话的页编程/擦除/回读都在锁内,
曾经用 `CriticalSectionRawMutex` 包裹导致 CAN 硬件 FIFO 溢出丢帧(§17-坑 1),
改 `ThreadModeRawMutex` 并拆除全部 `critical_section::with` 后全速零丢帧。

### 4.3 Channel(任务间命令队列)

```rust
pub static QUEUE: Channel<CriticalSectionRawMutex, StorageCmd, 8> = Channel::new();

// 生产端(任意任务/临界区内):满则丢,不阻塞
QUEUE.try_send(StorageCmd::Sync).ok();

// 消费端(storage 任务,唯一所有者):
match QUEUE.receive().await { StorageCmd::Write(d) => ..., }
```

- 这就是 C 版"消息队列 + 单一存储任务"的直译:所有 littlefs/NOR 操作收敛到一个任务,
  天然串行、无锁。FTP/HTTP/WS 的文件操作全部走 `StorageCmd` RPC(§8.3)。
- `try_send` 满时静默丢弃是**有意**对齐 C 的 `send_history_data`(队满丢记录);
  需要背压的场景应改用 `send().await`(本项目未用)。

### 4.4 异步 Mutex(W5500 共享 SPI 总线)

```rust
static SPI_BUS: StaticCell<AsyncMutex<NoopRawMutex, SpiBus>> = StaticCell::new();
let bus = SPI_BUS.init(AsyncMutex::new(spi));
let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(bus, cs);
```

embassy-net-wiznet 的 runner 任务会长期持有式地访问 SPI,`SpiDevice` 内部用
async `Mutex`(等待而非关中断)排队,`NoopRawMutex` 只满足类型参数(真正的互斥由 async Mutex 做)。

### 4.5 原子变量优先

链路状态、WS 会话占用、FTP/Modbus 槽位占用、RPC 序号全部用 `core::sync::atomic`
(`AtomicBool/AtomicU8/AtomicU32`)。 embassy 的 `Signal<T>` 有"旧值滞留、立即唤醒"的语义
陷阱(本项目早期用过,后换原子 + 轮询/序号),**跨任务的单值状态优先原子,事件用 Channel**。

---

## 5. main.rs

### 5.1 时钟树(RCC)

```rust
cfg.rcc.hse = Some(Hse { freq: Hertz(13_000_000), mode: HseMode::Oscillator });
cfg.rcc.pll = Some(Pll { prediv: PllPreDiv::DIV13, mul: PllMul::MUL336,
                         divp: Some(PllPDiv::DIV2), divq: Some(PllQDiv::DIV7), ... });
cfg.rcc.sys = Sysclk::PLL1_P;              // 168 MHz
cfg.rcc.apb1_pre = rcc::APBPrescaler::DIV4; // 42 MHz —— CAN 位时序的时钟基准
cfg.rcc.apb2_pre = rcc::APBPrescaler::DIV2; // 84 MHz —— USART1 BRR=729
cfg.rcc.ls = rcc::LsConfig::default_lse();  // RTC 走 LSE(VBAT 保持)
```

与 C 版逐项相同。CAN 波特率表(§10)按 PCLK1=42MHz 计算,改时钟树必须同步改表。

### 5.2 从 bootloader 跳转来的环境清理(纯寄存器,但必须有)

bootloader 跳转前只做了 `__disable_irq` + 清 NVIC ICPR[0]。应用必须自己:

```rust
unsafe {
    core::ptr::write_volatile(0xE000_ED08 as *mut u32, 0x0801_0200); // SCB.VTOR = 应用向量表
    // ICER×3 全关、ICPR×3 清 pending、EXTI RTSR/FTSR/IMR/PR 清残留
    cortex_m::interrupt::enable();   // 最后开 PRIMASK
}
```

不清 EXTI 残留的话,一开中断就会因 pending 的 DI 脚活动陷入 DefaultHandler 死循环
(向量表里未绑定的 IRQ 都指向 default handler)。

### 5.3 CCM 清零与任务缓冲的 `StaticCell` 模式

`.ccm.bss` 段(§16)不进 cortex-m-rt 的 .bss 清零循环,而 `StaticCell::init` 依赖
"内存初值为 0"做二次初始化检测 —— 所以 main 里手动把 `__sccm..__eccm` 清零,再 spawn 任务。

每个任务的套接字缓冲都从 main 的 static 来,再 `init()` 成 `'static mut` 传进任务:

```rust
#[link_section = ".ccm.bss"]
static MB_RX1: StaticCell<[u8; 512]> = StaticCell::new();
spawner.spawn(mbtcp::conn_task(*stack, MB_RX1.init([0u8; 512]), ...)).unwrap();
```

要点:
- **多实例任务的缓冲必须来自不同 static**(`MB_RX1`/`MB_RX2`),函数体内写 static 会在
  第二次实例化时被 `StaticCell` 的 double-init 检测 panic。
- `#[link_section = ".ccm.bss"]` 把 CPU 专属缓冲挪进 CCRAM,给 DMA 相关状态腾主 RAM(§6.3)。

### 5.4 heartbeat 任务

一个 100ms `Ticker` 复用出全部慢节拍:`appstate::reboot_due()`(250ms 粒度延迟重启)、
`reboot::due()`(100ms 死线复位)、1Hz `systime::tick_1hz()`、3s `wdt.pet()`(IWDG 30s)、
500ms netmon(`stack.is_link_up()`,断链 DO 全灭,C 版 w5500 net_mon 同语义)、
心跳 LED 300ms 亮/2.7s 灭。`IndependentWatchdog::new(dp.IWDG, 30_000_000)` + `unleash()` 启动。

### 5.5 panic handler

panic 先 `log::err` 位置与 payload(走 §12 的阻塞 TX,panic 上下文也能出 log),
再 `udf()` 停机 —— 停机后 IWDG 30s 内复位,与 C 版看门狗兜底一致。

---

## 6. net.rs:W5500 + embassy-net

### 6.1 链路构成

```
SPI2(21MHz, DMA1 CH3/CH4) ── SpiDevice(共享总线) ── embassy-net-wiznet::new
                                                    │  (MACRAW, socket0)
EXTI1 ←W5500 INT ── ExtiInput(中断驱动收包)         ↓
PD0  ──RST 推挽输出                          Device + Runner
                                                    ↓
                        embassy_net::Stack(smoltcp: TCP/UDP/ICMP/ARP)
```

### 6.2 关键代码

```rust
let mut spi_cfg = spi::Config::default();
spi_cfg.frequency = Hertz(21_000_000);
let spi = Spi::new(p.spi2, ..., p.tx_dma, p.rx_dma, Irqs, spi_cfg);   // 异步 DMA SPI
let int = ExtiInput::new(p.int, unsafe { peripherals::EXTI1::steal() }, Pull::Up, Irqs);
let (device, runner) = embassy_net_wiznet::new::<_, _, W5500, _, _, _>(mac, state, spi_dev, int, rst).await?;

let config = embassy_net::Config::new();   // 静态 IP,无 DHCP(C 版同)
let stack = STACK.init(Stack::new(device, config, DP, seed));
spawner.spawn(runner(runner)).ok();        // 报文泵任务
```

`Stack` 的静态化模式:`static STACK: StaticCell<Stack<'static>>`,所有任务持 `Stack<'static>`
拷贝(内部是 `&'static` 引用,`Copy`)。

### 6.3 内存约束(实测踩坑)

> embassy-net-wiznet 的 State 里的包队列由 **SPI DMA 直接写入** —— `State` 与包缓冲
> 必须留在 DMA 可达的主 RAM,**不能**放进 CCRAM(`#[link_section = ".ccm.bss"]` 会静默丢包)。
> 反过来,smoltcp 的 TCP 套接字缓冲是纯 CPU 存取,放 CCRAM 正好。本项目因此把
> `embassy_net_wiznet::State<4,4>` 留主 RAM、各 TCP 任务的 rx/tx 缓冲放 CCRAM。

### 6.4 UDP :8600 服务

`UdpSocket::new(stack, rx_meta, rx_buf)` → `bind(8600)` → `recv_from(&mut rx).await`;
应答 `send_to(&rep, endpoint)`,跨网段时定向广播到 8601(C 版 `udp_task.c` 语义)。
升级命令(0x01/02/03/06)在此任务内同步处理——NOR 页编程会短暂阻塞本任务,
但中断照常响应(§4.2),W5500 报文由 MACRAW 缓冲兜住,不丢窗口。

---

## 7. httpd / ftpd / mbtcp:TCP 服务三件套

三个模块共享同一套模式,差异只在业务:

### 7.1 通用模式

```rust
let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);      // 缓冲来自任务参数(§5.3)
sock.set_timeout(Some(Duration::from_secs(120)));          // smoltcp 级超时
loop {
    if sock.accept(PORT).await.is_err() { Timer::after_millis(100).await; continue; }
    serve(&mut sock).await;
    sock.abort();                                          // 见下:abort 而非 close
    Timer::after_millis(10).await;                         // 立刻可接下一个连接
}
```

三个实测要点:

1. **`accept(PORT)` 传端口不传地址**。`Some(0.0.0.0)` 会真的去匹配 0.0.0.0 而绑定失败,
   `None`(只给端口)= 任意地址。
2. **会话结束用 `abort()` 不用优雅关闭**。优雅关闭会走 FIN/TIME_WAIT,端口长时间没有
   LISTEN,下一个客户连不上;`abort()` 发 RST 立刻回到可 accept 状态(对齐 C 版 select 模型
   关闭语义)。客户侧表现为 connect 成功后请求被复位 —— Modbus 第 3 主站正是要这个效果。
3. **读写超时用 `select`**,不依赖 `set_timeout`:
   ```rust
   match select(sock.read(&mut buf), Timer::after(limit)).await {
       Either::First(Ok(n)) => ...,
       _ => /* 半请求 5s / keep-alive 空闲 60s 超时 */,
   }
   ```

### 7.2 拒绝器(rejector)模式 —— 第 N+1 个连接

FTP 限 3 会话、Modbus 限 2 主站,超出者由专职 `reject_task` accept 后立刻 `abort()`
(客户端收到 421 / 连接复位,C 版同)。**拒绝器必须只在满载时武装**:

```rust
// 满载 → 武装 accept;50ms 内空载 → abort 掉挂起的 accept 再 disarm
match select(sock.accept(PORT), Timer::after_millis(50)).await {
    Either::First(_) => { sock.abort(); }
    Either::Second(_) => {
        if BUSY.load(...) < MAX { continue; } else { sock.abort(); }
    }
}
```

如果不做 50ms 撤防,`accept()` future 会保持武装,把**下一个合法重连**抢走拒掉 —— 
本项目在满量 e2e 里真实出现过(§17-坑 4)。

### 7.3 各服务要点

- **httpd**(pool 2):gzip SPA 由 build.rs 生成 `INDEX_GZ` 常量;keep-alive + pipelining
  (半请求 5s、空闲 60s);WS 升级在同一 80 端口内完成握手,`WS_ACTIVE: AtomicBool`
  实现单会话(第二个握手 503 `ws busy`,ws.c 同)。WS 推送用 1s io/regs + 10s info 两级节拍。
- **ftpd**(pool 3):每任务两把 socket(控制 21 + 数据 PASV/PORT);`FTP_BUSY: AtomicU8`
  位图占槽;TYPE A 的 CR/LF 转换、REST 双向断点、APPE;文件操作全部 RPC 到存储任务(§8.3)。
- **mbtcp**(pool 2):ADU 拼装/拆包在 proto 库,服务循环 120s 超时;与 RTU 共享
  `MB_SERVER` 诊断计数(mb_server.c 同)。

---

## 8. storage.rs + w25q.rs

### 8.1 单所有者架构

```rust
// littlefs Filesystem 由 storage_task 独占;任务不退出,把局部 fs 'static 化长期持有
let fs: &'static mut Filesystem<'static, LfsNor> = unsafe { core::mem::transmute(&mut fs) };
loop { match QUEUE.receive().await { ... } }
```

littlefs2 crate 的 `Filesystem::allocate()` 需要 `&mut` 且 `'static`;任务永不退出,
transmute 把栈上的 fs 延寿成 `'static` 是该 crate 官方推荐的裸机用法。
所有文件 API 只在 storage 任务内调用 —— **从别的任务碰 fs 是未定义行为**,
外部一律走 QUEUE 命令。

### 8.2 littlefs 接入(Storage trait)

```rust
pub struct LfsNor;
impl Storage for LfsNor {
    fn read(&self, off: u32, buf: &mut [u8]) -> ... { nor_with(|w| w.read(LFS_OFFSET + off, buf)) }
    // write 按页拆分、erase 4K 对齐,全部走 nor_with(§4.2)
}
```

盘上格式与 C 版完全兼容(block 4096/lookahead 32B):C 写的历史文件本版直接挂载续写。

### 8.3 RPC 模式(跨任务文件操作)

```
ftpd/httpd 任务                    storage 任务
    QUEUE.try_send(FtpLs(path))  →  执行 fs 操作,结果写 FTP_RES/FILE_DL 等 static
    循环读 RPC_SEQ 原子变化       ←  RPC_SEQ.fetch_add(1)
```

`RPC_SEQ: AtomicU32` 是"完成序号":请求方记下发送前序号,自旋等 `RPC_SEQ` 变化即结果就绪。
比 `Signal` 可靠(Signal 旧值滞留会假就绪),比 Channel 简单(结果本体在专属 static 里)。

### 8.4 与升级互斥

`StorageCmd::Write` 处理时检查 `crate::fw::active()`:升级推流期间暂停历史落盘 ——
littlefs 目录轮转的整块擦除会秒级占用 NOR,与升级流竞争(§4.2 的锁保证正确性,
这里进一步主动避让保吞吐,C 版靠 flash 锁阻塞实现同样效果)。

### 8.5 w25q.rs(阻塞 SPI 驱动)

`W25q` 持有**阻塞模式**的 `Spi<'static, Blocking, Master>` + CS `Output`。为什么不用异步 SPI:
所有访问已被 `NOR`(ThreadModeRawMutex)串行化、调用方都在任务里同步调用
(`nor_with` 闭包不能有 await,§4.2),异步化没有收益;`wait_not_busy` 的 1ms 轮询间隔
是 STOR 吞吐(C 42KB/s vs 本版 14KB/s)的已知差距来源,优化点在加粗轮询/读状态位,
不影响正确性。驱动本体:JEDEC ID 探测、4K 擦、页编程(自动跨页拆分)、连续读。

---

## 9. fw.rs:固件升级会话

> **2026-08(feat/embassy-boot 分支)重写**:MCUboot 已换为 embassy-boot(见
> crates/bootloader 与 README 分区表)。载荷 = 裸 app + 64B ed25519 签名
> (SHA-512 摘要),签名在应用侧验(salty),swap 状态写在 NOR state 分区。

移植 `fw_upg.c` 的状态机与**返回码契约**(0 ok / -2 keyhash 不符 / -3 busy / -1 其他):

| 函数 | 职责 | C 对应 |
|---|---|---|
| `start(total, keyhash)` | 校验 keyhash → DFU 分区整擦(512K)→ 置 active | `fw_upg_start` |
| `write(data)` | 256B 页缓冲,页满编程 NOR | `fw_upg_write` |
| `finish(crc)` | 尾页冲刷 → 整载荷回读 CRC → salty ed25519 验签 | `fw_upg_finish_ex` |
| `boot_set_pending(_permanent)` | state 分区写 SWAP 魔数(embassy-boot 语义) | `boot_set_pending` |
| `boot_confirm()` | main 末尾确认本次换机(否则下次复位回滚) | — |
| `received()/total()/active()` | 进度查询(流控应答用) | `fw_upg_received` 等 |

三通道差异在 `finish` 的 CRC 参数:UDP/WS 带主机算的 CRC16(`Some`),CAN CONFIRM 不带
(`None` —— C 版 `fw_upg_finish_ex(0, false)` 同语义,完整性由 ed25519 验签兜底,
本版早期曾误将 None 当 0 比较导致 CONFIRM 恒失败,§17-坑 6)。

锁:整个会话状态 `FW: Mutex<ThreadModeRawMutex, RefCell<FwSession>>`,论证与坑见 §4.2、§17-坑 1。

---

## 10. fw_can.rs:bxCAN

### 10.1 初始化

```rust
let mut can = Can::new(can1, rx, tx, CanIrqs);   // 绑定 TX/RX0/RX1/SCE 四个中断
can.set_bitrate(kbps * 1000);                     // 查表 {50,100,125,250,500,1000}k,非法回落 250k
can.modify_filters().enable_bank(0, Fifo::Fifo0, [
    Mask16::frames_with_std_id(id_a, mask_full),   // 半槽A: 业务 ID 精确(holding[0x06])
    Mask16::frames_with_std_id(id_b, mask_top),    // 半槽B: 0x100-0x1FF 升级协议段
]);
can.enable().await;
let (mut tx, rx) = can.split();                   // ★ 收发半分离
let mut rx = rx.buffered(RXB.init(RxBuf::new())); // RX: 128 帧中断环形缓冲
```

- `set_bitrate` 内部按 PCLK1=42MHz 生成时序(采样点 85-90%),与 C 版位时序表一致;
  800k 无整数分频,回落 250k。
- 16 位标识符掩码过滤器的**两组半槽**布局是 bxCAN 特色:一个 bank 当两个独立过滤器用,
  (id<<5, mask<<5) 的半字布局务必与 ST 手册对齐(本项目以 C 版 net/can.c 为准逐位对齐)。
- MCR 默认:自动重传开(NART=0)、优先级调度(TXFP=0)—— 均与 C 版一致,勿改。

### 10.2 RX:为什么必须 buffered

bxCAN 硬件 RX FIFO 只有 **3 深**。250kbps 全速时帧间隔 ~0.55ms,任务一旦阻塞超过 1.65ms
(升级流的 NOR 页编程 0.4-3ms)FIFO 即溢出丢帧。`rx.buffered::<N>(RxBuf)` 把 RX0 中断变成
"硬件 FIFO → `Channel` 环形缓冲(N=128)"的搬运器,任务随时来 `rx.read().await` 消费 ——
相当于 C 版 "ISR 收帧入 32 深队列" 的加强版。

### 10.3 TX:为什么必须**不**用 buffered(踩坑换来的)

embassy 0.6 的 `BufferedCanTx` 发送中断在优先级调度下有缺陷:`Registers::transmit` 遇到
**同 ID 帧仍在邮箱待发**时返回 `WouldBlock`,而 buffered ISR 已把帧从环形队列取出、
直接丢弃该返回值 —— 帧无声丢失。CAN VERSION 应答是 `0x102 + 0x105 + 0x105` 三连发,
第二帧 0x105(git 哈希)必丢,上位机只见 `v0.3.0_`。

修复:TX 用 `split()` 出来的非缓冲 `CanTx`:

```rust
async fn send(tx: &mut CanTx<'static>, id: u16, data: &[u8]) {
    if let Ok(f) = Frame::new_standard(id, data) {
        let _ = tx.write(&f).await;   // WouldBlock 时挂起,邮箱空中断唤醒重试,不丢
    }
}
```

`CanTx::write` 的 poll 逻辑 = "登记 waker → 试写邮箱 → 失败则 Pending 等待 TX 中断唤醒重试",
语义等同 C 版 `mod_can_send` 的等邮箱忙等,但异步不烧 CPU。

### 10.4 协议与流控(以 Zephyr 为权威源)

帧:`0x101` 命令 `[cmd LE32][arg LE32]` / `0x102` 回复 / `0x103` 数据 ≤8B /
`0x104` keyhash 5×[seq][7B] / `0x105` 版本分帧。命令:START=0、CONFIRM=1、VERSION=2、REBOOT=3;
回复码 OFFSET=0/UPDATE_SUCCESS=1/VERSION=2/CONFIRM=3(+magic 0x55AA55AA)/FLASH_ERROR=4/
TRANSFER_ERROR=5/KEYHASH_ERROR=6。

**流控 = 每 64B 回一次 OFFSET**(Zephyr `can_fw_upgrade.c` 的 `fw_written % 64 == 0`)。
上位机工具(io-edge-hub can_manager.c)每发 8 帧读一次应答、读不到等 2s 超时容错 ——
如果固件 512B 才应答,每 512B 白等 7×2s,206KB 要 90 分钟以上(§17-坑 7:FreeRTOS 移植时
改成了 512B,本版最初照抄)。REBOOT 无应答,100ms 排空 + history 刷盘 + 内联复位
(`vTaskDelay(100)+log_flush(500)+history_sync()+NVIC_SystemReset()` 逐项对齐)。

---

## 11. rtu.rs:Modbus RTU

```rust
let uart = Uart::new(p.usart2, p.rx, p.tx, p.tx_dma, p.rx_dma, Irqs, cfg);  // DMA 收发
let (mut tx, mut rx) = uart.split();
let mut de = Output::new(p.de, Level::Low, Speed::Low);   // F4(usart_v1) 无驱动托管 DE,手动拉

loop {
    let n = rx.read_until_idle(&mut chunk).await;   // ★ t3.5 判帧
    // 帧完整(地址+CRC 过) → DE 高 → tx.write(响应) → DE 低
}
```

- **`read_until_idle` = 空闲线断帧**:DMA 收到字节流后,线路静默(IDLE 标志)即返回。
  9600bps 的 t3.5≈3.6ms,IDLE 检测粒度足够,再叠加 proto 层的静默校验兜底 —— 
  替代了 C 版"读多少算多少 + 4ms 定时器"组合,且天然处理背靠背帧。
- 波特率/从站号是**启动快照**(改配置重启生效,C 同),任务开头从 REGS 一次读出。
- DE 手动驱动:发送前拉高、写完等 TX 完成(`flush`)再拉低,窗口覆盖整个帧。

---

## 12. uart_raw / log / shell:控制台

唯一**不用** embassy 驱动的外设,原因两条(都是实测):

1. **logger 需要临界区内可用的同步 TX**:`log::err` 可能从 `critical_section::with` 里被调用,
   只能 TXE 位轮询直写寄存器(每字节 ~87µs@115200,可接受)。
2. **shell RX 必须在 PRIMASK 冻结期存活**:寄存器级 RXNE 中断在毫秒级关中断下丢字节
   (1 字节 DR + 溢出),而 **DMA2 循环通道由硬件搬 DR**,中断开关不影响;
   embassy 的 `RingBufferedUart` 走中断路径,不满足。

因此 `uart_raw.rs` 直接写 RCC/GPIOA/USART1/DMA2 寄存器:TXE 轮询发送 +
DMA2-Stream2 循环写到 256B 环(`rx_available`/`rx_peek` 按 NDTR 推算读写指针)。
`shell_task` 轮询这个环(10ms Timer 节拍),行编辑/历史/Tab 补全逻辑是纯代码,
与 embassy 无耦合。**这是"知道框架边界在哪"的示范:框架不合适时,裸寄存器 + 明确理由。**

---

## 13. sampling / io_gpio

- `di_task`:16×`Input::new(pin, Pull::Down)`(`AnyPin` 数组入任务),周期 = holding 寄存器
  [10,5000]ms 钳位;每轮在临界区里快照 REGS 配置 → 读引脚 → 回写 input 寄存器 →
  `QUEUE.try_send(StorageCmd::Write(HisData))` 落历史。
- `ai_task`:`Adc::new(adc1)` + `SampleTime`,4 通道轮测,`adc_math::ai_convert`
  换算工程值,同样走 REGS + QUEUE。
- `io_gpio`:DO8(PD7-14)+ LED8(PE8-15)`Output` 阵列,`set_do_led(u8)` 一次刷 16 脚;
  netmon 断链时清零(§5.4)。

DI 引脚是 EXTI-capable 的,但**故意用轮询**:DI 采样本来就是周期语义,EXTI 中断
只会给"变化即采样"的错误节奏(C 版也是轮询)。

---

## 14. systime.rs:RTC

```rust
let (rtc, tp) = Rtc::new(dp.RTC, RtcConfig::default());  // LSE,VBAT 保持
systime::init(rtc, &tp);
// epoch 只在启动时从 RTC 读一次,之后 AtomicU32 缓存 + 心跳 1Hz tick 递增;
// set_timestamp 同时写缓存和 RTC。
```

理由:RTC 寄存器读跨多个 BCD 寄存器需要小心一致性,而业务只需要秒级;
缓存原子 + 1Hz 递增的精度对日志时间戳/历史记录足够(2000-2100 合法窗口,缺省 2020)。

---

## 15. reboot / appstate

三条重启路径,语义各不相同(对齐 C):

| 路径 | 时序 | 实现 |
|---|---|---|
| UDP(END/REBOOT/FACTORY_RESET 应答后) | 应答上线 → 刷历史 → 100ms → **本任务内联复位** | net.rs:`Timer(100ms)+system_reset()` |
| CAN REBOOT | 100ms 排空 → 刷历史 → 500ms → 内联复位 | fw_can.rs |
| Web/shell | 应答后置 `set_reboot_status`,心跳任务 ~250ms 后走 cold | appstate + heartbeat |

**为什么 UDP/CAN 必须内联**:后台轮询式复位会留下 ~200ms 窗口,期间设备还在应答
GET_VERSION,上位机 `wait_online` 会抢答到"重启前的旧镜像"(stale uptime),
随后复位又砸在后续测试中间 —— e2e 曾整片级联失败(§17-坑 5)。C 版 `io_reboot_cold`
就是"应答后本任务阻塞 100ms 再复位",任务即刻静默;移植时把"内联阻塞"改成"全局死线"
看似等价,实则改变了可观察行为。**协议时序语义不能靠实现细节碰巧成立。**

`appstate.rs` 同时是 proto 库与固件的桥:`Hooks`(RegHooks)把寄存器写入副作用
(DO、时间戳、history 开关、保存)落到真实外设/队列 —— proto 纯逻辑、固件提供实现,
host 单测因此能全量跑协议层。

---

## 16. 内存布局

`memory.x`:

```
MEMORY {
  FLASH : ORIGIN = 0x08020000, LENGTH = 0x60000    /* embassy-boot active 分区(3×128K) */
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
```

- 应用链接在 0x08020000(embassy-boot active 分区,裸镜像无头,向量表在分区首;
  0x08000000-0x0801FFFF 留给 bootloader,实测 ~9K)。
  旧布局(MCUboot 0x08010200/448K)见 git main 分支。
- `.ccm.bss` 段:CCRAM 只 CPU 可达(DMA 不可!)—— 放 smoltcp 套接字缓冲、任务栈缓冲;
  **绝不**放 W5500 State(§6.3)。
- 段由 linker script 定义为 NOLOAD,main 里手动清零(§5.3)。
- release 配置:`opt-level="s"` + fat LTO + codegen-units=1,固件 ~202K < 448K slot。

---

## 17. 踩坑记录

每条都经过根因定位与复现验证,新模块评审时逐条对照:

| # | 现象 | 根因 | 修复/规则 |
|---|---|---|---|
| 1 | CAN 全速突发每 256B 丢 ~3 帧,上位机升级 30-90 分钟 | `fw.rs` 的页编程/整槽擦除包在 `critical_section::with`(PRIMASK)里,3 深 HW FIFO 在 0.55ms 帧距下溢出 | 长操作一律 ThreadModeRawMutex(§4.2 三条件);临界区只包微秒级 |
| 2 | CAN VERSION 只显示 `v0.3.0_` | embassy 0.6 BufferedCanTx 的 ISR 丢弃 `WouldBlock`(同 ID 帧还在邮箱时),第 2 帧 0x105 无声丢失 | TX 用非缓冲 `CanTx::write`(挂起重试);同 ID 连发场景禁用 buffered TX |
| 3 | littlefs 历史写入触发擦除风暴,FTP STOR 卡死看门狗复位 | 每条记录 `sync()` 重写 ~512B INLINESTRUCT tag → 目录 ~8 条一压实 | 有续写语义的文件保持打开、缓冲追加、显式时机才 sync |
| 4 | 满载后下一个合法 FTP/Modbus 连接被 421/复位 | rejector 的 `accept()` future 常驻武装,空载后仍抢连接 | select(accept, 50ms) + 空载 disarm(§7.2) |
| 5 | UDP 升级测试 uptime 断言失败、后续 42 项级联 | 重启命令走后台轮询死线,~200ms 内设备还在应答,wait_online 抢答旧镜像;复位落在下一测试中间 | UDP/CAN 重启必须任务内联复位(§15) |
| 6 | CAN CONFIRM 恒回 code 5 | `finish(None)` 误把"无 CRC"当"期望 CRC=0"比较,永远不等 | `None` = 跳过比较,MCUboot 验签兜底(§9) |
| 7 | 上位机工具 CAN 升级极慢 | 流控间隔 512B(FreeRTOS 移植改动),工具按 Zephyr 权威语义每 64B 等应答,7/8 等满 2s 超时 | ACK_INTERVAL=64 对齐 Zephyr 权威源;协议参数以权威源+上位机为准 |
| 8 | W5500 随机丢包/不收 | `embassy_net_wiznet::State` 里的包队列被 SPI DMA 直写,放进 CCRAM(DMA 不可达)后静默丢 | DMA 关联内存留主 RAM,CCRAM 只放 CPU 缓冲(§6.3) |

附加小坑(一句话):
- `NoopRawMutex` 含裸指针,不 `Sync` —— 需要进 static 时换 `ThreadModeRawMutex` + `unsafe Send/Sync`。
- 多实例任务体内不能定义 `StaticCell`(二次实例化 panic),缓冲从 main 传入(§5.3)。
- `Signal` 旧值滞留会假唤醒,跨任务状态用原子、事件用 Channel(§4.5)。
- `accept()` 绑定地址必须 `None` 而非 `Some(0.0.0.0)`(§7.1)。
- bootloader 跳转进应用前要自清 VTOR/NVIC/EXTI/PRIMASK,否则一开中断进 DefaultHandler(§5.2)。
