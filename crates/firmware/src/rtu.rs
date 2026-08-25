//! Modbus RTU slave on USART2 (PA2 TX / PA3 RX) with DE pin PA1: baud and
//! slave-id are startup snapshots (change requires reboot).

use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Peri;
use embassy_time::Timer;
use static_cell::StaticCell;

use io_edge_hub_proto::mb_server::MB_SERVER_PDU_MAX;
use io_edge_hub_proto::regmap::{HOLDING_RS485_BAUDRATE_IDX, HOLDING_SLAVE_ID_IDX};
use io_edge_hub_proto::rtu_frame::{rtu_t35_ms, RtuFrame};

use crate::appstate::{Hooks, MB_SERVER, REGS};

bind_interrupts! {
    struct Irqs {
        DMA1_STREAM5 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
        DMA1_STREAM6 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH6>;
        USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    }
}

pub struct RtuPins {
    pub usart2: Peri<'static, embassy_stm32::peripherals::USART2>,
    pub rx: Peri<'static, embassy_stm32::peripherals::PA3>,
    pub tx: Peri<'static, embassy_stm32::peripherals::PA2>,
    pub de: Peri<'static, embassy_stm32::peripherals::PA1>,
    pub tx_dma: Peri<'static, embassy_stm32::peripherals::DMA1_CH6>,
    pub rx_dma: Peri<'static, embassy_stm32::peripherals::DMA1_CH5>,
}

#[embassy_executor::task]
pub async fn rtu_task(p: RtuPins) {
    // startup snapshot (baud/slave changes require reboot)
    let (baud, unit) = critical_section::with(|_cs| {
        REGS.lock(|r| {
            let r = r.borrow();
            (
                r.get_holding(HOLDING_RS485_BAUDRATE_IDX as u16),
                r.get_holding(HOLDING_SLAVE_ID_IDX as u16) as u8,
            )
        })
    });
    let baud: u32 = if baud == 0 { 9600 } else { baud as u32 };

    let mut cfg = UartConfig::default();
    cfg.baudrate = baud;
    let uart = Uart::new(p.usart2, p.rx, p.tx, p.tx_dma, p.rx_dma, Irqs, cfg)
        .ok()
        .expect("usart2 cfg");
    let (mut tx, mut rx) = uart.split();
    // F4 (usart_v1) has no driver-managed DE: drive PA1 manually
    let mut de = embassy_stm32::gpio::Output::new(
        p.de,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    static RTU: StaticCell<RtuFrame> = StaticCell::new();
    let rtu = RTU.init(RtuFrame::new());
    rtu.bind(unit);
    let t35 = rtu_t35_ms(baud) as u64;
    crate::log::inf("rtu: up");

    let mut chunk = [0u8; 256];
    let mut tx_frame = [0u8; 1 + MB_SERVER_PDU_MAX + 2];
    loop {
        crate::stackmark::probe("rtu");
        // idle-line detection ends the read; remaining silence >= t3.5 spec
        let n = match rx.read_until_idle(&mut chunk).await {
            Ok(n) if n > 0 => n,
            _ => continue,
        };
        rtu.rx_feed(&chunk[..n]);
        Timer::after_millis(t35).await;
        critical_section::with(|_cs| {
            let reply = REGS.lock(|r| {
                MB_SERVER.lock(|s| {
                    let mut h = Hooks;
                    rtu.t35_expired(
                        &mut s.borrow_mut(),
                        &mut r.borrow_mut(),
                        &mut h,
                        &mut tx_frame,
                    )
                })
            });
            if let Some(len) = reply {
                de.set_high();
                // DE setup time (~30us)
                cortex_m::asm::delay(168 * 30);
                let _ = tx.blocking_write(&tx_frame[..len]);
                let _ = tx.blocking_flush();
                de.set_low();
            }
        });
    }
}
