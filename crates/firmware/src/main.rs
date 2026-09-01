#![no_std]
#![no_main]

mod appstate;
mod fw;
mod fw_can;
mod io_gpio;
mod log;
mod mbtcp;
mod net;
mod reboot;
mod rtu;
mod sampling;
mod shell;
mod stackmark;
mod storage;
mod systime;
mod uart_raw;
mod w25q;
mod w5500;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{
    self, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk,
};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Config as BoardConfig;
use embassy_time::{Duration, Ticker};

fn board_config() -> BoardConfig {
    let mut cfg = BoardConfig::default();
    // HSE 13MHz /13 *336 /2 = 168MHz SYSCLK, APB1 42MHz, APB2 84MHz
    cfg.rcc = rcc::Config::new();
    cfg.rcc.hse = Some(Hse {
        freq: Hertz(13_000_000),
        mode: HseMode::Oscillator,
    });
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
    // RTC on LSE (VBAT-backed)
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
            core::ptr::write_volatile((0xE000_E280 + 4 * i) as *mut u32, 0xFFFF_FFFF);
            // ICPR: clear pending
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
    // .ccm.bss is NOLOAD and outside the runtime's .bss zero loop, but the
    // statics placed there (StaticCell) require zeroed memory for their
    // double-init check — zero the region before any task runs.
    unsafe {
        extern "C" {
            static mut __sccm: u32;
            static mut __eccm: u32;
        }
        let mut p = core::ptr::addr_of_mut!(__sccm);
        let end = core::ptr::addr_of_mut!(__eccm);
        while p < end {
            p.write_volatile(0);
            p = p.add(1);
        }
    }
    // stack watermark pattern for `tasks`/`ps` (must precede embassy init:
    // from here on every deeper frame belongs to the runtime, not the boot)
    stackmark::init();
    let dp = embassy_stm32::init(board_config());

    // console + shell UART: raw USART1 (uart_raw) — sync TX for the logger,
    // DMA circular RX that survives NOR flash freezes; then spawn the shell
    uart_raw::init();
    spawner.spawn(shell::shell_task().expect("spawn sh"));
    use core::fmt::Write as _;
    let mut banner = heapless::String::<96>::new();
    banner
        .write_fmt(format_args!(
            "io-edge-hub rust {} boot ({})",
            appstate::version::FW_VERSION,
            appstate::version::FW_BUILD,
        ))
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

    // W5500 hardware TCP/IP stack (this branch): UDP :8600 config/upgrade +
    // Modbus TCP :502 on chip sockets; HTTP/FTP dropped with the smoltcp
    // stack. PD1 (INT) unused — the net task polls at 2ms.
    spawner.spawn(
        net::net_task(w5500::W5500Pins {
            spi2: dp.SPI2,
            sck: dp.PB13,
            miso: dp.PB14,
            mosi: dp.PB15,
            cs: dp.PB12,
            rst: dp.PD0,
        })
        .expect("spawn net"),
    );

    // heartbeat: LED PE7, IWDG 30s fed every 3s, delayed reboot
    let led = Output::new(dp.PE7, Level::High, Speed::Low);
    let wdt = IndependentWatchdog::new(dp.IWDG, 30_000_000);
    spawner.spawn(heartbeat(wdt, led).expect("spawn hb"));

    // Modbus RTU on USART2 + DE PA1 (baud/slave snapshot from cfg)
    spawner.spawn(
        rtu::rtu_task(rtu::RtuPins {
            usart2: dp.USART2,
            rx: dp.PA3,
            tx: dp.PA2,
            de: dp.PA1,
            tx_dma: dp.DMA1_CH6,
            rx_dma: dp.DMA1_CH5,
        })
        .expect("spawn rtu"),
    );

    // CAN1 fw-upgrade channel (PA11/PA12, baud/id snapshot from cfg)
    spawner.spawn(fw_can::fw_can_task(dp.CAN1, dp.PA11, dp.PA12).expect("spawn fwcan"));

    // DI16 sampling (channel order = dio.c di_pins, pull-down active-high)
    spawner.spawn(
        sampling::di_task(sampling::DiPins([
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
        .expect("spawn di"),
    );

    // AI4 sampling on ADC1 IN10-13 = PC0-PC3
    spawner.spawn(
        sampling::ai_task(sampling::AdcPins {
            adc1: dp.ADC1,
            ch0: dp.PC0,
            ch1: dp.PC1,
            ch2: dp.PC2,
            ch3: dp.PC3,
        })
        .expect("spawn ai"),
    );
    log::inf("io: DI16/AI4 sampling, rtu up");
    stackmark::probe("embassy-main");
}

#[embassy_executor::task]
async fn heartbeat(
    mut wdt: IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>,
    mut led: Output<'static>,
) {
    wdt.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut ticks: u32 = 0;
    loop {
        stackmark::probe("hb");
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
        // netmon: 500ms; LINK_UP refreshed by the net task's PHY poll;
        // link down -> DO all off
        if ticks % 5 == 0 {
            let up = crate::net::net_link_up();
            if !up {
                critical_section::with(|_cs| {
                    crate::appstate::REGS.lock(|r| {
                        r.borrow_mut().holding[io_edge_hub_proto::regmap::HOLDING_DO_IDX] = 0
                    });
                });
                io_gpio::set_do_led(0);
            }
        }
        // heartbeat LED: 300ms on / 2700ms off
        led.set_level(if ticks % 30 < 3 {
            Level::High
        } else {
            Level::Low
        });
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

    #[allow(deprecated)]
    if let Some(p) = info.payload().downcast_ref::<&str>() {
        log::err(p);
    }
    cortex_m::asm::udf()
}
