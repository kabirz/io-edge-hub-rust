//! CRCs, ported 1:1 from src/util/io_crc.c.

pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    crc
}

pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    crc ^ 0xFFFF_FFFF
}

/// CRC16-CCITT as used by the fw-upgrade protocol (Zephyr crc16() semantics:
/// reflected 0x1021, init 0, no final xor) - from src/fw/fw_upg.c.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    crc16_ccitt_seed(data, 0)
}

pub fn crc16_ccitt_seed(data: &[u8], seed: u16) -> u16 {
    let mut crc = seed;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_modbus_check() {
        // standard check value for "123456789"
        assert_eq!(crc16_modbus(b"123456789"), 0x4B37);
    }

    #[test]
    fn crc32_ieee_check() {
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn crc16_ccitt_empty_is_zero() {
        assert_eq!(crc16_ccitt(&[]), 0);
    }

    #[test]
    fn crc16_ccitt_incremental() {
        let whole = crc16_ccitt(b"hello world");
        assert_eq!(crc16_ccitt_seed(b"world", crc16_ccitt(b"hello ")), whole);
    }
}
