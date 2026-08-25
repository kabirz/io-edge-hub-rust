//! System time: RTC-backed epoch cache.
//!
//! The epoch is read once at boot (RTC keeps time via VBAT), cached in an
//! atomic, and incremented by a 1 Hz tick; `set_timestamp` writes both the RTC
//! and the cache. Validity window 2000..2100, fallback 2020-01-01.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::rtc::{DateTime, DayOfWeek, Rtc, RtcTimeProvider};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use io_edge_hub_proto::time_math::{Civil, TS_DEFAULT, civil_to_unix, ts_valid, unix_to_civil};

static EPOCH: AtomicU32 = AtomicU32::new(TS_DEFAULT);
static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    Mutex::new(RefCell::new(None));

pub fn init(rtc: Rtc, provider: &RtcTimeProvider) {
    let epoch = match provider.now() {
        Ok(dt) if (2000..=2099).contains(&dt.year()) => civil_to_unix(&Civil {
            year: dt.year(),
            month: dt.month(),
            day: dt.day(),
            hour: dt.hour(),
            min: dt.minute(),
            sec: dt.second(),
        }),
        _ => TS_DEFAULT,
    };
    EPOCH.store(epoch, Ordering::Relaxed);
    critical_section::with(|_cs| {
        RTC.lock(|c| *c.borrow_mut() = Some(rtc));
    });
}

pub fn now_epoch() -> u32 {
    EPOCH.load(Ordering::Relaxed)
}

/// 1 Hz tick from the heartbeat task.
pub fn tick_1hz() {
    EPOCH.fetch_add(1, Ordering::Relaxed);
}

/// Set RTC + cache. Range gate: 2000-01-01..2100-01-01.
pub fn set_timestamp(ts: u32) -> bool {
    if !ts_valid(ts) {
        return false;
    }
    let c = unix_to_civil(ts);
    let dt = DateTime::from(c.year, c.month, c.day, DayOfWeek::Thursday, c.hour, c.min, c.sec, 0)
        .expect("civil date in valid window");
    critical_section::with(|_cs| {
        RTC.lock(|c| {
            if let Some(rtc) = c.borrow_mut().as_mut() {
                rtc.set_datetime(dt).ok();
            }
        });
    });
    EPOCH.store(ts, Ordering::Relaxed);
    true
}
