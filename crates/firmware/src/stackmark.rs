//! Main-stack high-water marks — backs the `tasks`/`ps` stack column.
//!
//! The firmware has no heap and a single executor, so the free RAM above the
//! statics ([_stack_end, _stack_start)) is one physical stack shared by every
//! embassy task poll and every IRQ handler (Cortex-M exceptions run on MSP).
//! There are no per-task stacks to report, but each task's polls reach a
//! different depth: [probe] records the lowest MSP a task has ever observed
//! at its loop entries (and in its deep helpers), which is that task's
//! contribution to the shared stack. The pattern-scan [usage] stays the
//! authoritative whole-stack watermark — it also catches IRQ frames and
//! C-FFI depth below any probe point.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

const PATTERN: u32 = 0xA5A5_A5A5;

extern "C" {
    static _stack_end: u32; // bottom: first byte past .bss (start of free RAM)
    static _stack_start: u32; // top: initial SP (end of RAM)
}

/// One row per spawned task, in `tasks`/`ps` display order. The [slot]
/// constants index into the min-SP table — keep them in sync with this order.
pub const TASK_NAMES: [&str; 20] = [
    "embassy-main",
    "hb",
    "net-run",
    "net-stack",
    "udp-cfg",
    "storage",
    "mbtcp1",
    "mbtcp2",
    "mb-reject",
    "http1",
    "http2",
    "ftp1",
    "ftp2",
    "ftp3",
    "ftp-reject",
    "rtu",
    "fwcan",
    "di",
    "ai",
    "sh",
];

pub mod slot {
    pub const MAIN: usize = 0;
    pub const HB: usize = 1;
    pub const NET_RUN: usize = 2;
    pub const NET_STACK: usize = 3;
    pub const UDP: usize = 4;
    pub const STORAGE: usize = 5;
    pub const MBTCP1: usize = 6;
    pub const MBTCP2: usize = 7;
    pub const MB_REJECT: usize = 8;
    pub const HTTP1: usize = 9;
    pub const HTTP2: usize = 10;
    pub const FTP_BASE: usize = 11; // + ftp_task's 0/1/2 slot param
    pub const FTP_REJECT: usize = 14;
    pub const RTU: usize = 15;
    pub const FWCAN: usize = 16;
    pub const DI: usize = 17;
    pub const AI: usize = 18;
    pub const SH: usize = 19;
}

/// Lowest MSP seen per task slot; 0 = task has not reached a probe yet.
/// Only thread-mode tasks call [probe] (no preemption between them), so the
/// ThreadModeRawMutex is sound and cheap.
static MIN_SP: Mutex<ThreadModeRawMutex, RefCell<[usize; TASK_NAMES.len()]>> =
    Mutex::new(RefCell::new([0; TASK_NAMES.len()]));

/// Record the current MSP as this task's watermark. Call from a task's main
/// loop (and optionally its deep helpers); a handful of instructions.
pub fn probe(slot: usize) {
    let sp = cortex_m::register::msp::read() as usize;
    MIN_SP.lock(|c| {
        let mut c = c.borrow_mut();
        if c[slot] == 0 || sp < c[slot] {
            c[slot] = sp;
        }
    });
}

/// (min free bytes of the whole shared stack, total bytes) — pattern-scan
/// watermark since boot; authoritative (includes IRQ and C-FFI depth).
pub fn usage() -> (u32, u32) {
    unsafe {
        let lo = core::ptr::addr_of!(_stack_end) as *const u32;
        let hi = core::ptr::addr_of!(_stack_start) as *const u32;
        let total = ((hi as usize).saturating_sub(lo as usize)) as u32;
        let mut p = lo;
        let mut free = 0u32;
        while p < hi && p.read_volatile() == PATTERN {
            free += 4;
            p = p.add(1);
        }
        (free.min(total), total)
    }
}

/// This task's min free bytes (deepest probed SP - stack bottom); None if the
/// task has not run a probe yet. Smaller = this task's polls dig deeper into
/// the shared stack; the values are not additive across tasks.
pub fn task_free(slot: usize) -> Option<u32> {
    let lo = unsafe { core::ptr::addr_of!(_stack_end) as usize };
    let m = MIN_SP.lock(|c| c.borrow()[slot]);
    if m == 0 {
        None
    } else {
        Some((m.saturating_sub(lo)) as u32)
    }
}

/// Pattern-fill the not-yet-used stack below the current SP. Call once from
/// main() before spawning tasks. IRQs may already be enabled: at this point no
/// handler can have run, so nothing deeper than main's live frame exists and
/// the fill boundary is safe. Frames already used above the SP (Reset_Handler,
/// executor trampoline) stay untouched; the watermark therefore measures
/// usage below main's boot frame, which is where all task/IRQ depth happens.
pub fn init() {
    unsafe {
        let sp = cortex_m::register::msp::read() as usize & !3;
        let mut p = core::ptr::addr_of!(_stack_end) as *mut u32;
        while (p as usize) < sp {
            p.write_volatile(PATTERN);
            p = p.add(1);
        }
    }
}
