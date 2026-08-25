//! History record codec + file naming (history_file.c / init.h his_data).
//!
//! DI record: 10 B — type=1 u16, timestamps u32, di_en_status u16, di_value u16.
//! AI record: 16 B — type=2 u16, timestamps u32, ai_en_status u16, ai_value[4] u16.
//! Filenames: `data_MMDD_HHMMSS.raw` from unix time + 8 h (UTC+8, same
//! manual offset as make_hist_name; values clamped to valid ranges).

pub const DI_TYPE: u16 = 1;
pub const AI_TYPE: u16 = 2;
pub const AI_NUM: usize = 4;

pub const DI_REC_LEN: usize = 10;
pub const AI_REC_LEN: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HisData {
    pub ty: u16,
    pub timestamps: u32,
    pub en_status: u16,
    /// DI: single u16 value bitmap. AI: 4 converted values.
    pub values: [u16; AI_NUM],
}

impl HisData {
    pub fn di(timestamps: u32, di_en: u16, di_value: u16) -> Self {
        Self {
            ty: DI_TYPE,
            timestamps,
            en_status: di_en,
            values: [di_value, 0, 0, 0],
        }
    }

    pub fn ai(timestamps: u32, ai_en: u16, ai_value: [u16; AI_NUM]) -> Self {
        Self {
            ty: AI_TYPE,
            timestamps,
            en_status: ai_en,
            values: ai_value,
        }
    }

    pub fn rec_len(&self) -> usize {
        if self.ty == DI_TYPE {
            DI_REC_LEN
        } else {
            AI_REC_LEN
        }
    }

    /// Wire/on-disk encoding (packed little-endian, PC tools compatible).
    pub fn to_bytes(&self) -> [u8; AI_REC_LEN] {
        let mut b = [0u8; AI_REC_LEN];
        b[0..2].copy_from_slice(&self.ty.to_le_bytes());
        b[2..6].copy_from_slice(&self.timestamps.to_le_bytes());
        b[6..8].copy_from_slice(&self.en_status.to_le_bytes());
        if self.ty == DI_TYPE {
            b[8..10].copy_from_slice(&self.values[0].to_le_bytes());
        } else {
            for i in 0..AI_NUM {
                b[8 + 2 * i..10 + 2 * i].copy_from_slice(&self.values[i].to_le_bytes());
            }
        }
        b
    }

    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < DI_REC_LEN {
            return None;
        }
        let ty = u16::from_le_bytes([b[0], b[1]]);
        if ty == DI_TYPE {
            Some(Self {
                ty,
                timestamps: u32::from_le_bytes(b[2..6].try_into().unwrap()),
                en_status: u16::from_le_bytes([b[6], b[7]]),
                values: [u16::from_le_bytes([b[8], b[9]]), 0, 0, 0],
            })
        } else if ty == AI_TYPE && b.len() >= AI_REC_LEN {
            let mut values = [0u16; AI_NUM];
            for i in 0..AI_NUM {
                values[i] = u16::from_le_bytes([b[8 + 2 * i], b[9 + 2 * i]]);
            }
            Some(Self {
                ty,
                timestamps: u32::from_le_bytes(b[2..6].try_into().unwrap()),
                en_status: u16::from_le_bytes([b[6], b[7]]),
                values,
            })
        } else {
            None
        }
    }
}

fn clamp(v: i64, lo: i64, hi: i64) -> i64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// Civil date/time from unix days (Howard Hinnant's civil_from_days).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// `data_MMDD_HHMMSS.raw` for a unix timestamp (+8 h like the C code).
pub fn make_hist_name(unix: u32) -> [u8; 20] {
    let t = unix as i64 + 8 * 3600;
    let days = t.div_euclid(86_400);
    let secs = t.rem_euclid(86_400);
    let (_y, mon, mday) = civil_from_days(days);
    let hour = clamp(secs / 3600, 0, 23);
    let min = clamp(secs / 60 % 60, 0, 59);
    let sec = clamp(secs % 60, 0, 59);
    let mut name = *b"data_0000_000000.raw";
    name[5] = b'0' + (clamp(mon as i64, 1, 12) / 10) as u8;
    name[6] = b'0' + (clamp(mon as i64, 1, 12) % 10) as u8;
    name[7] = b'0' + (clamp(mday as i64, 1, 31) / 10) as u8;
    name[8] = b'0' + (clamp(mday as i64, 1, 31) % 10) as u8;
    name[10] = b'0' + (hour / 10) as u8;
    name[11] = b'0' + (hour % 10) as u8;
    name[12] = b'0' + (min / 10) as u8;
    name[13] = b'0' + (min % 10) as u8;
    name[14] = b'0' + (sec / 10) as u8;
    name[15] = b'0' + (sec % 10) as u8;
    name
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn di_record_roundtrip() {
        let d = HisData::di(0x5F00_0000, 0xFFFF, 0x1234);
        let b = d.to_bytes();
        // DI fills only the first 10 bytes of the buffer
        assert!(b[DI_REC_LEN..].iter().all(|&x| x == 0));
        assert_eq!(b[0..2], DI_TYPE.to_le_bytes());
        let back = HisData::from_bytes(&b).unwrap();
        assert_eq!(back, d);
    }

    #[test]
    fn ai_record_roundtrip() {
        let d = HisData::ai(42, 0x000F, [100, 200, 300, 400]);
        let b = d.to_bytes();
        assert_eq!(b.len(), AI_REC_LEN);
        let back = HisData::from_bytes(&b).unwrap();
        assert_eq!(back, d);
        assert_eq!(back.values, [100, 200, 300, 400]);
    }

    #[test]
    fn di_record_exact_layout() {
        // golden bytes: type=1, ts=0x11223344, en=0x55AA, value=0x0F0F
        let d = HisData::di(0x1122_3344, 0x55AA, 0x0F0F);
        assert_eq!(
            &d.to_bytes()[..10],
            &[1, 0, 0x44, 0x33, 0x22, 0x11, 0xAA, 0x55, 0x0F, 0x0F]
        );
    }

    #[test]
    fn name_matches_c_format() {
        // 2026-08-23 00:00:00 UTC = 1787443200; +8h -> 08:00 same day
        let n = make_hist_name(1_787_443_200);
        let s = core::str::from_utf8(&n).unwrap();
        assert_eq!(s, "data_0823_080000.raw");
    }

    #[test]
    fn name_clamps_epoch_zero() {
        let n = make_hist_name(0);
        let s = core::str::from_utf8(&n).unwrap();
        assert_eq!(s, "data_0101_080000.raw");
    }
}
