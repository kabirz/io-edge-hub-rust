//! Main-stack high-water marks — backs the `tasks`/`ps` stack column.
//!
//! The firmware has no heap and a single executor, so the free RAM above the
//! statics ([_stack_end, _stack_start)) is one physical stack shared by every
//! embassy task poll and every IRQ handler. There are no per-task stacks to
//! report, but each task's polls reach a different depth: [probe] records
//! the lowest MSP a task has ever observed, which is that task's
//! contribution to the shared stack.
//!
//! embassy-executor keeps no runtime task registry (its pool holds anonymous
//! futures — nothing like uxTaskGetSystemState exists to enumerate), so the
//! task list cannot be read from the executor. Tasks instead self-register
//! here BY NAME on their first [probe]; `tasks` prints whatever registered.
//! The pattern-scan [usage] stays the authoritative whole-stack watermark —
//! it also catches IRQ frames and C-FFI depth below any probe point.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::Mutex;

const PATTERN: u32 = 0xA5A5_A5A5;

/// Registration slots. A no_std register needs a fixed bound; 24 leaves
/// headroom above the current 20 tasks (a task beyond it simply does not
/// show up in `tasks`).
const MAX_TASKS: usize = 24;

/// min_sp == 0 means "registered but never past the first probe yet"
/// (registration itself probes, so in practice min_sp is set immediately).
#[derive(Clone, Copy)]
struct TaskStat {
    name: Option<&'static str>,
    min_sp: usize,
    loops: u32,
}

/// Only thread-mode tasks call [probe] (no preemption between them), so the
/// ThreadModeRawMutex is sound and cheap. Registered slots are contiguous
/// from 0 in first-probe (= spawn) order.
static STATS: Mutex<ThreadModeRawMutex, RefCell<[TaskStat; MAX_TASKS]>> = Mutex::new(RefCell::new(
    [TaskStat {
        name: None,
        min_sp: 0,
        loops: 0,
    }; MAX_TASKS],
));

/// Record the current MSP and one loop iteration for this task. Call from a
/// task's main loop (and optionally its deep helpers); a handful of
/// instructions. The first call registers the task.
pub fn probe(name: &'static str) {
    let sp = cortex_m::register::msp::read() as usize;
    STATS.lock(|c| {
        let mut stats = c.borrow_mut();
        let mut idx = stats.len();
        let mut free_slot = stats.len();
        for (i, s) in stats.iter().enumerate() {
            match s.name {
                Some(n) if core::ptr::eq(n, name) => {
                    idx = i;
                    break;
                }
                None if free_slot == stats.len() => free_slot = i,
                _ => {}
            }
        }
        if idx == stats.len() {
            if free_slot == stats.len() {
                return; // register full: not listed, not counted
            }
            idx = free_slot;
            stats[idx].name = Some(name);
        }
        let s = &mut stats[idx];
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

/// Number of registered tasks (rows in `tasks`).
pub fn task_count() -> usize {
    STATS.lock(|c| c.borrow().iter().filter(|s| s.name.is_some()).count())
}

/// Row `i` of the register: (name, min free bytes, loop-iteration count).
/// free = deepest probed SP - stack bottom; smaller free = this task's polls
/// dig deeper into the shared stack; the values are not additive across
/// tasks. None past [task_count].
pub fn task_stat(i: usize) -> Option<(&'static str, Option<u32>, u32)> {
    let lo = core::ptr::addr_of!(_stack_end) as usize;
    STATS.lock(|c| {
        let s = &c.borrow()[i];
        let free = if s.min_sp == 0 {
            None
        } else {
            Some((s.min_sp.saturating_sub(lo)) as u32)
        };
        s.name.map(|n| (n, free, s.loops))
    })
}

extern "C" {
    static _stack_end: u32; // bottom: first byte past .bss (start of free RAM)
    static _stack_start: u32; // top: initial SP (end of RAM)
    static __sccm: u32; // CCM region start (.ccm.bss)
    static __eccm: u32; // CCM region end
}

/// Statics footprint of the main SRAM (everything below the stack region):
/// total SRAM minus the shared stack.
pub fn statics_bytes() -> u32 {
    128 * 1024 - usage().1
}

/// (used, total) of the CCM region (.ccm.bss: socket buffers etc.).
pub fn ccm_usage() -> (u32, u32) {
    let s = core::ptr::addr_of!(__sccm) as usize;
    let e = core::ptr::addr_of!(__eccm) as usize;
    ((e.saturating_sub(s)) as u32, 64 * 1024)
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
