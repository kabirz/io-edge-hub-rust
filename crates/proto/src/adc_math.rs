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
        // C goldens (test_adc_math.c): 4095*3300/4096 = 3299 mV first step
        assert_eq!(ai_convert(0, 4095), 2445);
        assert_eq!(ai_convert(1, 4095), 2445);
        assert_eq!(ai_convert(2, 4095), 1221);
        assert_eq!(ai_convert(3, 4095), 1221);
    }

    #[test]
    fn zero_and_mid() {
        assert_eq!(ai_convert(0, 0), 0);
        assert_eq!(ai_convert(3, 0), 0);
        // raw 2048 -> 1650 mV -> 7414*1650/10000 = 1223 (0.01mA)
        assert_eq!(ai_convert(0, 2048), 1223);
        assert_eq!(ai_convert(2, 2048), 611);
        assert_eq!(ai_convert(0, 1), 0);
        assert_eq!(ai_convert(3, 1), 0);
    }
}
