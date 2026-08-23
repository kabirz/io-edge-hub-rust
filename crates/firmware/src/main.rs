#![no_std]
#![no_main]

mod appstate;
mod ftpd;
mod httpd;
mod io_gpio;
mod log;
mod mbtcp;
mod net;
mod reboot;
mod rtu;
mod sampling;
mod storage;
mod systime;
mod w25q;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{self, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Config as BoardConfig;
use embassy_time::{Duration, Ticker};

fn board_config() -> BoardConfig {
    let mut cfg = BoardConfig::default();
    // HSE 13MHz /13 *336 /2 = 168MHz SYSCLK, APB1 42MHz, APB2 84MHz (same tree as the C firmware)
    cfg.rcc = rcc::Config::new();
    cfg.rcc.hse = Some(Hse { freq: Hertz(13_000_000), mode: HseMode::Oscillator });
    cfg.rcc.pll_src = PllSource::HSE;
    cfg.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV13,
        mul: PllMul::MUL336,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    cfg.rcc.sys = Sysclk::PLL1_P;
    cfg.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
    cfg.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
    cfg.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    // RTC on LSE (VBAT-backed, same as the C firmware)
    cfg.rcc.ls = rcc::LsConfig::default_lse();
    cfg
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // boot_jump_vec() leaves PRIMASK set (__disable_irq before the jump) and
    // cortex-m-rt never clears it. Before unmasking we must also kill any
    // interrupts the bootloader left pending/enabled: its jump only clears
    // NVIC ICPR[0], so a pending IRQ whose vector our table binds to
    // DefaultHandler (e.g. EXTI9_5 from DI pin activity) would trap the core
    // in the default handler loop the moment interrupts open.
    unsafe {
        core::ptr::write_volatile(0xE000_ED08 as *mut u32, 0x0801_0200); // SCB.VTOR
        for i in 0..3usize {
            core::ptr::write_volatile((0xE000_E180 + 4 * i) as *mut u32, 0xFFFF_FFFF); // ICER: disable all
            core::ptr::write_volatile((0xE000_E280 + 4 * i) as *mut u32, 0xFFFF_FFFF); // ICPR: clear pending
        }
        // wipe leftover EXTI configuration from the loader: triggers + mask + pending
        core::ptr::write_volatile(0x4001_3C08 as *mut u32, 0x0); // RTSR
        core::ptr::write_volatile(0x4001_3C0C as *mut u32, 0x0); // FTSR
        core::ptr::write_volatile(0x4001_3C00 as *mut u32, 0x0); // IMR
        core::ptr::write_volatile(0x4001_3C14 as *mut u32, 0xFFFF_FFFF); // PR: clear all
        for _ in 0..100 {
            cortex_m::asm::nop();
        }
        cortex_m::interrupt::enable();
    }
    let dp = embassy_stm32::init(board_config());

    // USART1 console: PA9 TX / PA10 RX @115200 (same as C firmware)
    let uart = Uart::new_blocking(dp.USART1, dp.PA10, dp.PA9, UartConfig::default())
        .ok()
        .expect("usart1 config");
    let (tx, rx) = uart.split();
    log::init(tx);
    let _shell_rx = rx; // shell lands in M7
    use core::fmt::Write as _;
    let mut banner = heapless::String::<64>::new();
    banner
        .write_fmt(format_args!("io-edge-hub rust {} boot", appstate::version::FW_VERSION))
        .ok();
    log::inf(&banner);

    // DO8 on PD7-PD14, LED8 mirror on PE8-PE15 (idle low)
    io_gpio::init(
        [
            Some(Output::new(dp.PD7, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD8, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD9, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD10, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD11, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD12, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD13, Level::Low, Speed::Low)),
            Some(Output::new(dp.PD14, Level::Low, Speed::Low)),
        ],
        [
            Some(Output::new(dp.PE8, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE9, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE10, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE11, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE12, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE13, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE14, Level::Low, Speed::Low)),
            Some(Output::new(dp.PE15, Level::Low, Speed::Low)),
        ],
    );


    // RTC-backed system time (VBAT persistent)
    let (rtc, tp) = Rtc::new(dp.RTC, RtcConfig::default());
    systime::init(rtc, &tp);

    // W25Q128 NOR on SPI1 (PA5/6/7, CS PA4): config store + littlefs.
    // Config loads synchronously before net bring-up (IP comes from cfg);
    // the chip is then owned by the storage task in an interrupt executor
    // so NOR busy-polls never stall network/Modbus for more than one
    // flash operation.
    let nor = w25q::W25q::new(w25q::W25qPins {
        spi1: dp.SPI1,
        sck: dp.PA5,
        miso: dp.PA6,
        mosi: dp.PA7,
        cs: dp.PA4,
    });
    critical_section::with(|_cs| {
        crate::storage::NOR.lock(|r| *r.borrow_mut() = nor);
    });
    crate::storage::boot_config_load();

    spawner.spawn(crate::storage::storage_task().expect("spawn storage"));

    // heartbeat: LED PE7, IWDG 30s fed every 3s, delayed reboot
    // (spawned after net setup: netmon needs the stack handle)
    let led = Output::new(dp.PE7, Level::High, Speed::Low);
    let wdt = IndependentWatchdog::new(dp.IWDG, 30_000_000);

    // W5500 MACRAW + embassy-net + UDP :8600
    let stack = net::setup(
        &spawner,
        net::NetPins {
            spi2: dp.SPI2,
            sck: dp.PB13,
            miso: dp.PB14,
            mosi: dp.PB15,
            cs: dp.PB12,
            int: dp.PD1,
            rst: dp.PD0,
            tx_dma: dp.DMA1_CH4,
            rx_dma: dp.DMA1_CH3,
        },
    )
    .await;
    log::inf("net: W5500 up, udp 8600");
    spawner.spawn(heartbeat(wdt, led, *stack).expect("spawn hb"));

    // Modbus TCP :502: 2 serving sockets (3rd client finds no listener -> RST,
    // same cap as the C firmware); per-instance buffers from distinct statics
    static MB_RX1: static_cell::StaticCell<[u8; 512]> = static_cell::StaticCell::new();
    static MB_TX1: static_cell::StaticCell<[u8; 512]> = static_cell::StaticCell::new();
    static MB_RX2: static_cell::StaticCell<[u8; 512]> = static_cell::StaticCell::new();
    static MB_TX2: static_cell::StaticCell<[u8; 512]> = static_cell::StaticCell::new();
    spawner
        .spawn(mbtcp::conn_task(
            *stack,
            MB_RX1.init([0u8; 512]),
            MB_TX1.init([0u8; 512]),
        )
        .expect("spawn mbtcp1"));
    spawner
        .spawn(mbtcp::conn_task(
            *stack,
            MB_RX2.init([0u8; 512]),
            MB_TX2.init([0u8; 512]),
        )
        .expect("spawn mbtcp2"));
    // 3rd listener accepts-then-aborts the excess master (tcp.c behavior)
    static RJ_RX: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static RJ_TX: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    spawner
        .spawn(mbtcp::reject_task(
            *stack,
            RJ_RX.init([0u8; 64]),
            RJ_TX.init([0u8; 64]),
        )
        .expect("spawn mbreject"));
    log::inf("mbtcp: port 502 listening");

    // HTTP :80 (2 connections, httpd.c cap); per-instance socket buffers
    static HTTP_RX1: static_cell::StaticCell<[u8; 640]> = static_cell::StaticCell::new();
    static HTTP_TX1: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static HTTP_RX2: static_cell::StaticCell<[u8; 640]> = static_cell::StaticCell::new();
    static HTTP_TX2: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    spawner
        .spawn(httpd::http_task(
            *stack,
            HTTP_RX1.init([0u8; 640]),
            HTTP_TX1.init([0u8; 2048]),
        )
        .expect("spawn http1"));
    spawner
        .spawn(httpd::http_task(
            *stack,
            HTTP_RX2.init([0u8; 640]),
            HTTP_TX2.init([0u8; 2048]),
        )
        .expect("spawn http2"));
    log::inf("httpd: port 80 listening");

    // FTP :21 (3 sessions + 421 rejector, ftpd.c cap)
    static FR1: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FT1: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FDR1: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FDT1: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FR2: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FT2: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FDR2: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FDT2: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FR3: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FT3: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();
    static FDR3: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FDT3: static_cell::StaticCell<[u8; 2048]> = static_cell::StaticCell::new();
    static FRJ: static_cell::StaticCell<[u8; 128]> = static_cell::StaticCell::new();
    static FTJ: static_cell::StaticCell<[u8; 128]> = static_cell::StaticCell::new();
    spawner
        .spawn(ftpd::ftp_task(
            *stack,
            FR1.init([0u8; 1024]),
            FT1.init([0u8; 1024]),
            FDR1.init([0u8; 2048]),
            FDT1.init([0u8; 2048]),
        )
        .expect("spawn ftp1"));
    spawner
        .spawn(ftpd::ftp_task(
            *stack,
            FR2.init([0u8; 1024]),
            FT2.init([0u8; 1024]),
            FDR2.init([0u8; 2048]),
            FDT2.init([0u8; 2048]),
        )
        .expect("spawn ftp2"));
    spawner
        .spawn(ftpd::ftp_task(
            *stack,
            FR3.init([0u8; 1024]),
            FT3.init([0u8; 1024]),
            FDR3.init([0u8; 2048]),
            FDT3.init([0u8; 2048]),
        )
        .expect("spawn ftp3"));
    spawner
        .spawn(ftpd::ftp_reject_task(
            *stack,
            FRJ.init([0u8; 128]),
            FTJ.init([0u8; 128]),
        ).expect("spawn ftprej"));
    log::inf("ftp: port 21 listening");

    // Modbus RTU on USART2 + DE PA1 (baud/slave snapshot from cfg)
    spawner
        .spawn(rtu::rtu_task(rtu::RtuPins {
            usart2: dp.USART2,
            rx: dp.PA3,
            tx: dp.PA2,
            de: dp.PA1,
            tx_dma: dp.DMA1_CH6,
            rx_dma: dp.DMA1_CH5,
        })
        .expect("spawn rtu"));

    // DI16 sampling (channel order = dio.c di_pins, pull-down active-high)
    spawner
        .spawn(sampling::di_task(sampling::DiPins([
            dp.PD3.into(),
            dp.PD4.into(),
            dp.PD5.into(),
            dp.PD6.into(),
            dp.PB5.into(),
            dp.PB6.into(),
            dp.PB7.into(),
            dp.PB8.into(),
            dp.PB9.into(),
            dp.PB10.into(),
            dp.PB11.into(),
            dp.PD2.into(),
            dp.PB0.into(),
            dp.PB1.into(),
            dp.PB3.into(),
            dp.PB4.into(),
        ]))
        .expect("spawn di"));

    // AI4 sampling on ADC1 IN10-13 = PC0-PC3
    spawner
        .spawn(sampling::ai_task(sampling::AdcPins {
            adc1: dp.ADC1,
            ch0: dp.PC0,
            ch1: dp.PC1,
            ch2: dp.PC2,
            ch3: dp.PC3,
        })
        .expect("spawn ai"));
    log::inf("io: DI16/AI4 sampling, rtu up");
}

#[embassy_executor::task]
async fn heartbeat(
    mut wdt: IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>,
    mut led: Output<'static>,
    stack: embassy_net::Stack<'static>,
) {
    wdt.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut ticks: u32 = 0;
    loop {
        ticker.next().await;
        ticks = ticks.wrapping_add(1);
        if ticks == 1 {
            log::inf("hb: ticking");
        }

        // delayed reboot poll (100ms granularity): web/UDP-triggered
        appstate::reboot_due();
        if reboot::due() {
            log::wrn("reboot: system reset");
            reboot::system_reset();
        }

        // 1 Hz epoch tick
        if ticks % 10 == 0 {
            systime::tick_1hz();
        }
        // IWDG feed every 3s (30s window)
        if ticks % 30 == 0 {
            wdt.pet();
        }
        // netmon: 500ms link poll; link down -> DO all off (w5500.c net_mon)
        if ticks % 5 == 0 {
            let up = stack.is_link_up();
            critical_section::with(|_cs| crate::net::LINK_UP.lock(|b| *b.borrow_mut() = up));
            if !up {
                critical_section::with(|_cs| {
                    crate::appstate::REGS.lock(|r| {
                        r.borrow_mut().holding[io_edge_hub_proto::regmap::HOLDING_DO_IDX] = 0
                    });
                });
                io_gpio::set_do_led(0);
            }
        }
        // heartbeat LED: 300ms on / 2700ms off (same as the C firmware)
        led.set_level(if ticks % 30 < 3 { Level::High } else { Level::Low });
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // location and payload logged separately: each fits the line buffer
    match info.location() {
        Some(l) => {
            let mut msg = heapless::String::<160>::new();
            use core::fmt::Write as _;
            let _ = msg.write_fmt(format_args!("PANIC at {}", l));
            log::err(&msg);
        }
        None => {
            log::err("PANIC (no location)");
        }
    }
    if let Some(p) = info.payload().downcast_ref::<&str>() {
        log::err(p);
    }
    cortex_m::asm::udf()
}
