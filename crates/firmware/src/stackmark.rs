//! Main-stack high-water mark — backs the `tasks`/`ps` stack column.
//!
//! The firmware has no heap and a single executor, so the free RAM above the
//! statics ([_stack_end, _stack_start)) is one physical stack shared by every
//! embassy task poll and every IRQ handler (Cortex-M exceptions run on MSP).
//! At boot, before any task is spawned, the untouched region below the current
//! SP is pattern-filled; [usage] then scans for the first non-pattern word,
//! giving the minimum free watermark since boot.

const PATTERN: u32 = 0xA5A5_A5A5;

extern "C" {
    static _stack_end: u32; // bottom: first byte past .bss (start of free RAM)
    static _stack_start: u32; // top: initial SP (end of RAM)
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

/// (min free bytes since boot, total bytes) of the shared main stack.
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
