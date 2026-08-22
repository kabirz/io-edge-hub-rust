//! USART1 console logging with the C firmware's line format
//! `[HH:MM:SS.mmm][L] message`.

use core::cell::RefCell;
use core::fmt::Write as _;

use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::systime;

static TX: Mutex<CriticalSectionRawMutex, RefCell<Option<UartTx<'static, embassy_stm32::mode::Blocking>>>> =
    Mutex::new(RefCell::new(None));

pub fn init(tx: UartTx<'static, embassy_stm32::mode::Blocking>) {
    critical_section::with(|_cs| {
        TX.lock(|t| *t.borrow_mut() = Some(tx));
    });
}

pub fn log(level: char, msg: &str) {
    let epoch = systime::now_epoch();
    let secs = epoch % 86_400;
    let mut line = heapless::String::<160>::new();
    let _ = write!(
        line,
        "[{:02}:{:02}:{:02}.000][{}] {}\r\n",
        secs / 3600,
        (secs % 3600) / 60,
        secs % 60,
        level,
        msg
    );
    critical_section::with(|_cs| {
        TX.lock(|t| {
            if let Some(tx) = t.borrow_mut().as_mut() {
                tx.blocking_write(line.as_bytes()).ok();
            }
        });
    });
}

pub fn inf(msg: &str) {
    log('I', msg);
}

pub fn wrn(msg: &str) {
    log('W', msg);
}

pub fn err(msg: &str) {
    log('E', msg);
}
