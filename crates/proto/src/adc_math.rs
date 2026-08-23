//! AI engineering-value conversion, ported from src/io/adc.c ai_convert.

/// coeffs scaled by 1e4: AI0/1 = 4-20mA (0.01mA), AI2/3 = 0-10V (0.01V)
pub const AI_COEFF: [u32; 4] = [7414, 7414, 3704, 3704];

/// 12-bit raw -> engineering value; two-step truncating division like the C
/// code (voltage first, then u64 product / 10000).
pub const fn ai_convert(ch: usize, raw: i32) -> u16 {
    let voltage_mv = raw * 3300 / 4096; // VREF = VDDA = 3.3V
    (AI_COEFF[ch] as u64 * voltage_mv as u64 / 10000) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_scale() {
        // raw 4095 -> ~3300mV -> 4-20mA ch: 7414*3299/10000 = 2446 (0.01mA)
        assert_eq!(ai_convert(0, 4095), 2446);
        // 0-10V ch: 3704*3299/10000 = 1222 (0.01V)
        assert_eq!(ai_convert(2, 4095), 1222);
    }

    #[test]
    fn zero_and_mid() {
        assert_eq!(ai_convert(0, 0), 0);
        assert_eq!(ai_convert(3, 0), 0);
        // raw 2048 -> 2048*3300/4096 = 1650mV
        assert_eq!(ai_convert(0, 2048), 7414 * 1650 / 10000);
    }
}
