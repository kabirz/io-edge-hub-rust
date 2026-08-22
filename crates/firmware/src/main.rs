#![no_std]
#![no_main]

mod appstate;
mod io_gpio;
mod log;
mod net;
mod reboot;
mod systime;

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
    // cortex-m-rt never clears it; without this every IRQ stays masked and
    // all embassy timers/SPI-DMA/EXTI wait forever.
    unsafe { cortex_m::interrupt::enable(); }
    // belt-and-braces: make sure VTOR points at our table (the loader already
    // sets it; kept for independence from loader behavior)
    unsafe {
        core::ptr::write_volatile(0xE000_ED08 as *mut u32, 0x0801_0200); // SCB.VTOR
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

    // heartbeat: LED PE7, IWDG 30s fed every 3s, 1Hz epoch, delayed reboot
    let led = Output::new(dp.PE7, Level::High, Speed::Low);
    let wdt = IndependentWatchdog::new(dp.IWDG, 30_000_000);
    spawner.spawn(heartbeat(wdt, led).expect("spawn heartbeat"));    // W5500 MACRAW + embassy-net + UDP :8600
    net::setup(
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
}

#[embassy_executor::task]
async fn heartbeat(mut wdt: IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>, mut led: Output<'static>) {
    wdt.unleash();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut ticks: u32 = 0;
    loop {
        ticker.next().await;
        ticks = ticks.wrapping_add(1);
        if ticks == 1 {
            log::inf("hb: ticking");
        }

        // delayed reboot poll (100ms granularity)
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
        // heartbeat LED: 300ms on / 2700ms off (same as the C firmware)
        led.set_level(if ticks % 30 < 3 { Level::High } else { Level::Low });
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut msg = heapless::String::<96>::new();
    use core::fmt::Write as _;
    let _ = msg.write_fmt(format_args!("PANIC {}", info));
    log::err(&msg);
    cortex_m::asm::udf()
}
