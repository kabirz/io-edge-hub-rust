//! Unix timestamp <-> civil date conversion (Howard Hinnant's algorithm),
//! mirroring src/sys/time.c semantics: validity window 2000-01-01..2100-01-01,
//! default time 2020-01-01 00:00:00 UTC.

/// 2000-01-01T00:00:00Z (minimum accepted timestamp)
pub const TS_MIN: u32 = 946_684_800;
/// 2100-01-01T00:00:00Z (maximum accepted timestamp)
pub const TS_MAX: u32 = 4_102_444_800;
/// Power-on fallback time 2020-01-01T00:00:00Z
pub const TS_DEFAULT: u32 = 1_577_836_800;

pub fn ts_valid(ts: u32) -> bool {
    (TS_MIN..=TS_MAX).contains(&ts)
}

/// Civil date/time (UTC). Year in 2000..=2099 for our validity window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Civil {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
}

/// days from civil date (Hinnant days_from_civil)
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as i64; // [0, 399]
    let mp = ((m + 9) % 12) as i64; // Mar=0
    let doy = (153 * mp + 2) / 5 + d as i64 - 1; // [0, 365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    era * 146_097 + doe - 719_468
}

/// civil date from days (Hinnant civil_from_days)
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    (if m <= 2 { y + 1 } else { y }, m, d)
}

pub fn civil_to_unix(c: &Civil) -> u32 {
    let days = days_from_civil(c.year as i64, c.month as u32, c.day as u32);
    (days * 86_400 + c.hour as i64 * 3600 + c.min as i64 * 60 + c.sec as i64) as u32
}

pub fn unix_to_civil(ts: u32) -> Civil {
    let days = (ts / 86_400) as i64;
    let rem = ts % 86_400;
    let (y, m, d) = civil_from_days(days);
    Civil {
        year: y as u16,
        month: m as u8,
        day: d as u8,
        hour: (rem / 3600) as u8,
        min: ((rem % 3600) / 60) as u8,
        sec: (rem % 60) as u8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_reference_values() {
        let c = unix_to_civil(0);
        assert_eq!((c.year, c.month, c.day), (1970, 1, 1));
        let c = unix_to_civil(TS_MIN);
        assert_eq!((c.year, c.month, c.day, c.hour), (2000, 1, 1, 0));
        let c = unix_to_civil(TS_MAX - 1);
        assert_eq!((c.year, c.month, c.day), (2099, 12, 31));
        let c = unix_to_civil(1_787_184_000); // 2026-08-20 00:00 UTC (C test comment said 08-19; UTC is 20th)
        assert_eq!((c.year, c.month, c.day), (2026, 8, 20));
    }

    #[test]
    fn roundtrip() {
        for &ts in &[
            TS_MIN,
            1_577_836_800,
            1_787_184_000,
            2_145_916_800,
            TS_MAX - 1,
        ] {
            assert_eq!(civil_to_unix(&unix_to_civil(ts)), ts);
        }
    }

    #[test]
    fn leap_day() {
        // 2024-02-29 12:00:00 UTC = 1709208000
        let c = unix_to_civil(1_709_208_000);
        assert_eq!((c.year, c.month, c.day, c.hour), (2024, 2, 29, 12));
    }

    #[test]
    fn validity_window() {
        assert!(ts_valid(TS_MIN));
        assert!(ts_valid(TS_MAX));
        assert!(!ts_valid(TS_MIN - 1));
        assert!(!ts_valid(TS_MAX + 1));
    }
}
