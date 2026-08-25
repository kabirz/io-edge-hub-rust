//! USART1 console logging, line format `[HH:MM:SS.mmm][L] message`.
//!
//! TX is the raw-register poll in uart_raw (safe under critical sections);
//! the shell's RX lives there too (DMA circular, freeze-proof).

use core::fmt::Write as _;

use crate::uart_raw;

pub fn log(level: char, msg: &str) {
    let secs = crate::systime::now_epoch_local() % 86_400;
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
    uart_raw::write(line.as_bytes());
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

/// Untagged shell line: message + CRLF, no timestamp.
pub fn line(msg: &str) {
    raw(msg.as_bytes());
    raw(b"\r\n");
}

/// Raw bytes straight to the wire: shell prompt/echo/redraw.
pub fn raw(bytes: &[u8]) {
    uart_raw::write(bytes);
}
