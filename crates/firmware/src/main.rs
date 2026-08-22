#![no_std]
#![no_main]

mod fw_version {
    include!(concat!(env!("OUT_DIR"), "/fw_version.rs"));
}

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{self, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Config as BoardConfig;
use embassy_time::Timer;

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
    cfg
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let dp = embassy_stm32::init(board_config());

    // heartbeat LED PE7 (same pin/pattern as the C firmware)
    let mut led = Output::new(dp.PE7, Level::High, Speed::Low);

    // USART1 console: PA9 TX / PA10 RX @115200 (same as C firmware)
    let mut uart = Uart::new_blocking(dp.USART1, dp.PA10, dp.PA9, UartConfig::default())
        .ok()
        .expect("usart1 config");

    let _ = uart.blocking_write(b"[I] io-edge-hub rust ");
    let _ = uart.blocking_write(fw_version::FW_VERSION.as_bytes());
    let _ = uart.blocking_write(b" boot\r\n");

    loop {
        Timer::after_millis(300).await;
        led.set_low();
        Timer::after_millis(2700).await;
        led.set_high();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf()
}
