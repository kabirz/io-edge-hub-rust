//! Delayed reboot, ported from src/sys/reboot.c / io_reboot_cold semantics.
//!
//! The pending deadline is polled by the heartbeat task; when due it does the
//! cold reset. Cold path = short delay (100ms) so the UDP reply can leave the
//! device first; the web/shell path uses ~3s graceful delay.

use core::sync::atomic::{AtomicU32, Ordering};

/// absolute deadline in ms-since-boot (0 = none); wraps safely via wrapping_sub
static DEADLINE_MS: AtomicU32 = AtomicU32::new(0);

fn now_ms() -> u32 {
    embassy_time::Instant::now().as_ticks() as u32 / (embassy_time::TICK_HZ as u32 / 1000)
}

pub fn schedule_delayed_ms(ms: u32) {
    DEADLINE_MS.store(now_ms().wrapping_add(ms), Ordering::Relaxed);
}

/// io_reboot_cold(): reboot after ~100ms (reply must be on the wire already).
pub fn cold() {
    schedule_delayed_ms(100);
}

/// Web/shell graceful reboot (~3s like the C heartbeat path).
pub fn graceful() {
    schedule_delayed_ms(3000);
}

pub fn cancel() {
    DEADLINE_MS.store(0, Ordering::Relaxed);
}

/// True exactly once when the deadline passes (heartbeat then resets).
/// Signed distance now-deadline >= 0 means due (wraparound-safe).
pub fn due() -> bool {
    let d = DEADLINE_MS.load(Ordering::Relaxed);
    if d == 0 {
        return false;
    }
    if now_ms().wrapping_sub(d) < 0x8000_0000 {
        DEADLINE_MS.store(0, Ordering::Relaxed);
        return true;
    }
    false
}

pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}
