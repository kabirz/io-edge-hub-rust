//! DI16 + AI4 sampling tasks:
//! interval = reg 0x03/0x04 clamped [10,5000]ms, only enabled channels read,
//! DI bitmap -> input_reg[5]; history records land on the storage queue.

use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{AnyPin, Input, Pull};
use embassy_stm32::Peri;
use embassy_time::{Duration, Timer};

use io_edge_hub_proto::adc_math::ai_convert;
use io_edge_hub_proto::history::HisData;
use io_edge_hub_proto::regmap::{
    HOLDING_AI_ENABLE_IDX, HOLDING_AI_SAMPLE_MS_IDX, HOLDING_DI_ENABLE_IDX,
    HOLDING_DI_SAMPLE_MS_IDX, INPUT_AI0_IDX, INPUT_DI_IDX,
};

use crate::appstate::REGS;
use crate::storage::{StorageCmd, QUEUE};

/// Queue a history record (send_history_data: full queue drops silently).
fn send_history_data(d: HisData) {
    QUEUE.try_send(StorageCmd::Write(d)).ok();
}

const SAMPLE_INTERVAL_MAX: u64 = 5000;

fn clamp_interval(v: u16) -> u64 {
    match v {
        0..=9 => 10,
        v if v as u64 > SAMPLE_INTERVAL_MAX => SAMPLE_INTERVAL_MAX,
        v => v as u64,
    }
}

/// DI pins in board channel order.
pub struct DiPins(pub [Peri<'static, AnyPin>; 16]);

#[embassy_executor::task]
pub async fn di_task(pins: DiPins) {
    let inputs: [Input<'static>; 16] = pins.0.map(|p| Input::new(p, Pull::Down));
    loop {
        crate::stackmark::probe(crate::stackmark::slot::DI);
        let (si, en) = critical_section::with(|_cs| {
            REGS.lock(|r| {
                let r = r.borrow();
                (
                    r.get_holding(HOLDING_DI_SAMPLE_MS_IDX as u16),
                    r.get_holding(HOLDING_DI_ENABLE_IDX as u16),
                )
            })
        });
        let mut val = 0u16;
        for (i, pin) in inputs.iter().enumerate() {
            if en & (1 << i) != 0 && pin.is_high() {
                val |= 1 << i;
            }
        }
        critical_section::with(|_cs| {
            REGS.lock(|r| {
                r.borrow_mut().update_input(INPUT_DI_IDX as u16, val).ok();
            });
        });
        // history record: queued when any DI channel is enabled
        if en != 0 {
            send_history_data(HisData::di(crate::systime::now_epoch(), en, val));
        }
        Timer::after(Duration::from_millis(clamp_interval(si))).await;
    }
}

pub struct AdcPins {
    pub adc1: Peri<'static, embassy_stm32::peripherals::ADC1>,
    pub ch0: Peri<'static, embassy_stm32::peripherals::PC0>,
    pub ch1: Peri<'static, embassy_stm32::peripherals::PC1>,
    pub ch2: Peri<'static, embassy_stm32::peripherals::PC2>,
    pub ch3: Peri<'static, embassy_stm32::peripherals::PC3>,
}

#[embassy_executor::task]
pub async fn ai_task(p: AdcPins) {
    let mut adc: Adc<'static, embassy_stm32::peripherals::ADC1> = Adc::new(p.adc1);
    let (mut c0, mut c1, mut c2, mut c3) = (p.ch0, p.ch1, p.ch2, p.ch3);
    loop {
        crate::stackmark::probe(crate::stackmark::slot::AI);
        let (si, en) = critical_section::with(|_cs| {
            REGS.lock(|r| {
                let r = r.borrow();
                (
                    r.get_holding(HOLDING_AI_SAMPLE_MS_IDX as u16),
                    r.get_holding(HOLDING_AI_ENABLE_IDX as u16),
                )
            })
        });
        macro_rules! sample {
            ($ch:expr, $i:expr) => {
                if en & (1 << $i) != 0 {
                    let raw = adc.blocking_read(&mut $ch, SampleTime::CYCLES144);
                    let val = ai_convert($i, raw as i32);
                    critical_section::with(|_cs| {
                        REGS.lock(|r| {
                            r.borrow_mut()
                                .update_input((INPUT_AI0_IDX + $i) as u16, val)
                                .ok();
                        });
                    });
                }
            };
        }
        sample!(c0, 0);
        sample!(c1, 1);
        sample!(c2, 2);
        sample!(c3, 3);
        // history record: queued when any AI channel is enabled
        if en & 0x000F != 0 {
            let values = critical_section::with(|_cs| {
                REGS.lock(|r| {
                    let rb = r.borrow();
                    [
                        rb.get_input(INPUT_AI0_IDX as u16),
                        rb.get_input((INPUT_AI0_IDX + 1) as u16),
                        rb.get_input((INPUT_AI0_IDX + 2) as u16),
                        rb.get_input((INPUT_AI0_IDX + 3) as u16),
                    ]
                })
            });
            send_history_data(HisData::ai(
                crate::systime::now_epoch(),
                en & 0x000F,
                values,
            ));
        }
        Timer::after(Duration::from_millis(clamp_interval(si))).await;
    }
}
