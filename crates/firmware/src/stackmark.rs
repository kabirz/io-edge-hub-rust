//! Main-stack high-water marks — backs the `tasks`/`ps` stack column.
//!
//! The firmware has no heap and a single executor, so the free RAM above the
//! statics ([_stack_end, _stack_start)) is one physical stack shared by every
//! embassy task poll and every IRQ handler. There are no per-task stacks to
//! report, but each task's polls reach a different depth: [probe] records
//! the lowest MSP a task has ever observed, which is that task's
//! contribution to the shared stack. The pattern-scan [usage] stays the
//! authoritative whole-stack watermark — it also catches IRQ frames and
//! C-FFI depth below any probe point.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::Mutex;

const PATTERN: u32 = 0xA5A5_A5A5;

extern "C" {
    static _stack_end: u32; // bottom: first byte past .bss (start of free RAM)
    static _stack_start: u32; // top: initial SP (end of RAM)
    static __sccm: u32; // CCM region start (.ccm.bss)
    static __eccm: u32; // CCM region end
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

/// Lowest MSP seen per task slot plus a loop-iteration count; min_sp == 0
/// means the task has not reached a probe yet. Only thread-mode tasks call
/// [probe] (no preemption between them), so the ThreadModeRawMutex is sound
/// and cheap.
#[derive(Clone, Copy)]
struct TaskStat {
    min_sp: usize,
    loops: u32,
}

static STATS: Mutex<ThreadModeRawMutex, RefCell<[TaskStat; TASK_NAMES.len()]>> =
    Mutex::new(RefCell::new(
        [TaskStat {
            min_sp: 0,
            loops: 0,
        }; TASK_NAMES.len()],
    ));

/// Record the current MSP and one loop iteration for this task. Call from a
/// task's main loop (and optionally its deep helpers); a handful of
/// instructions.
pub fn probe(slot: usize) {
    let sp = cortex_m::register::msp::read() as usize;
    STATS.lock(|c| {
        let s = &mut c.borrow_mut()[slot];
        s.loops = s.loops.wrapping_add(1);
        if s.min_sp == 0 || sp < s.min_sp {
            s.min_sp = sp;
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

/// (this task's min free bytes, its loop-iteration count). free = deepest
/// probed SP - stack bottom; None if the task has not run a probe yet.
/// Smaller free = this task's polls dig deeper into the shared stack; the
/// values are not additive across tasks.
pub fn task_stat(slot: usize) -> (Option<u32>, u32) {
    let lo = unsafe { core::ptr::addr_of!(_stack_end) as usize };
    let s = STATS.lock(|c| c.borrow()[slot]);
    if s.min_sp == 0 {
        (None, s.loops)
    } else {
        (Some((s.min_sp.saturating_sub(lo)) as u32), s.loops)
    }
}

/// Statics footprint of the main SRAM (everything below the stack region):
/// total SRAM minus the shared stack.
pub fn statics_bytes() -> u32 {
    128 * 1024 - usage().1
}

/// (used, total) of the CCM region (.ccm.bss: socket buffers etc.).
pub fn ccm_usage() -> (u32, u32) {
    unsafe {
        let s = core::ptr::addr_of!(__sccm) as usize;
        let e = core::ptr::addr_of!(__eccm) as usize;
        ((e.saturating_sub(s)) as u32, 64 * 1024)
    }
}

/// Pattern-fill the not-yet-used stack below the current SP. Call once from
/// main() before spawning tasks: at that point no handler has run yet, so
/// the fill boundary below main's live frame is safe. Frames already used
/// above the SP stay untouched; the watermark measures usage below main's
/// boot frame, which is where all task/IRQ depth happens.
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
