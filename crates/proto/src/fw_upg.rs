//! Firmware-upgrade protocol helpers.
//!
//! Bootloader: embassy-boot (replaced MCUboot). The swap algorithm and its
//! partition constraints are documented in `partitions` below; the app only
//! talks to the DFU/STATE partitions through `embassy_boot`'s
//! BlockingFirmwareUpdater, and the bootloader binary owns ACTIVE.
//!
//! - CRC16-CCITT: Zephyr sys/crc.h crc16_ccitt (reflected, poly 0x1021,
//!   init 0) — identical to the host-side helpers; the final check hashes
//!   the bytes read back from DFU (validating the programming result, not
//!   the source).
//! - FW_KEYHASH: SHA-256 of the signing public key. It is no longer an
//!   MCUboot TLV — it survives as a lightweight "right device?" gate on the
//!   START command of every transport.

/// SHA-256(RSA-2048 public key, PKCS#1 DER) — kept from the MCUboot era as
/// a START-command device gate. Fixed for the lifetime of the signing key.
pub const FW_KEYHASH: [u8; 32] = [
    0xfb, 0x73, 0x8f, 0x7d, 0xcf, 0x8c, 0x1a, 0x3a, 0x89, 0x88, 0xae, 0x60, 0x85, 0xd5, 0x2f, 0x06,
    0xaf, 0x2a, 0x25, 0x61, 0x33, 0xee, 0x1c, 0x06, 0x84, 0xf7, 0x14, 0x25, 0xaa, 0x15, 0x7f, 0xac,
];

pub const FW_KEYHASH_LEN: usize = 32;

/// embassy-boot partition layout for this board.
///
/// embassy-boot's swap unit ("page") is `max(ACTIVE::ERASE_SIZE,
/// DFU::ERASE_SIZE)`. The STM32F407 internal flash has non-uniform sectors
/// (4×16K + 64K + 3×128K) and its HAL reports ERASE_SIZE = max = 128K, so:
///
/// - PAGE_SIZE is fixed at 128K;
/// - ACTIVE must be a multiple of 128K AND page boundaries must coincide
///   with physical sector boundaries (each erase(page) erases exactly one
///   real sector) → ACTIVE = the three 128K sectors at the top of flash;
/// - DFU must be ≥ ACTIVE + one page → 512K, on the external W25Q (4K
///   uniform sectors, 128K % 4K == 0);
/// - STATE has no page-alignment requirement, only
///   `capacity/WRITE_SIZE >= 2 + 2 * active_pages` (= 8 words here); it
///   lives in the first external sector.
pub mod partitions {
    /// Internal flash: bootloader reservation (the binary itself is ~24K;
    /// the rest is slack for debug builds and future growth).
    pub const BOOT_BASE: u32 = 0x0800_0000;
    pub const BOOT_LEN: u32 = 128 * 1024;

    /// Internal flash ACTIVE slot = the three 128K sectors. The app links
    /// here directly (no header offset — plain bin, no imgtool).
    pub const ACTIVE_BASE: u32 = 0x0802_0000;
    pub const ACTIVE_LEN: u32 = 384 * 1024;

    /// External W25Q STATE partition (swap/revert progress), first sector.
    pub const STATE_BASE: u32 = 0x0000_0000;
    pub const STATE_LEN: u32 = 4096;

    /// External W25Q DFU slot (upgrade staging). Config A/B (0xE0000+) and
    /// littlefs (0xF0000+) sit above and are untouched.
    pub const DFU_BASE: u32 = 0x0000_1000;
    pub const DFU_LEN: u32 = 512 * 1024;

    /// embassy-boot page size for this layout: max(128K int, 4K ext).
    pub const PAGE_SIZE: u32 = 128 * 1024;
}

// ==================== CRC16-CCITT (reflected, init 0) ====================

pub fn crc16_ccitt(mut seed: u16, src: &[u8]) -> u16 {
    for &b in src {
        let e = (seed ^ b as u16) as u8;
        let f = e ^ (e << 4);
        seed = (seed >> 8) ^ ((f as u16) << 8) ^ ((f as u16) << 3) ^ ((f >> 4) as u16);
    }
    seed
}

// ==================== base64 (standard alphabet, decode only) ====================

/// Standard-alphabet base64 decode with padding; returns the decoded length
/// or None on any invalid character / length. Used for the WS fw_start
/// keyhash field (44 chars -> 32 bytes).
pub fn b64_decode(inp: &[u8], out: &mut [u8]) -> Option<usize> {
    fn val(c: u8) -> Option<u8> {
        match c {
            b'A'..=b'Z' => Some(c - b'A'),
            b'a'..=b'z' => Some(c - b'a' + 26),
            b'0'..=b'9' => Some(c - b'0' + 52),
            b'+' => Some(62),
            b'/' => Some(63),
            _ => None,
        }
    }
    let data_len = inp.iter().position(|&c| c == b'=').unwrap_or(inp.len());
    let pad = inp.len().saturating_sub(data_len);
    if pad > 2 || inp.len() % 4 != 0 {
        return None;
    }
    let mut acc: u32 = 0;
    let mut nbits = 0u32;
    let mut o = 0usize;
    for &c in &inp[..data_len] {
        acc = (acc << 6) | val(c)? as u32;
        nbits += 6;
        if nbits >= 8 {
            nbits -= 8;
            if o >= out.len() {
                return None;
            }
            out[o] = (acc >> nbits) as u8;
            o += 1;
        }
    }
    Some(o)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// crc16_ccitt(0, "123456789") for the reflected poly-0x1021/init-0
    /// variant (CRC-16/KERMIT check value).
    #[test]
    fn crc16_check_value() {
        assert_eq!(crc16_ccitt(0, b"123456789"), 0x2189);
    }

    #[test]
    fn crc16_incremental_matches_oneshot() {
        let data: Vec<u8> = (0..=255u8).cycle().take(1000).collect();
        let a = crc16_ccitt(0, &data);
        let b = crc16_ccitt(crc16_ccitt(0, &data[..500]), &data[500..]);
        assert_eq!(a, b);
    }

    #[test]
    fn partitions_satisfy_embassy_boot_invariants() {
        use partitions::*;
        // page = max(int erase 128K, ext erase 4K)
        assert_eq!(PAGE_SIZE, 128 * 1024);
        // ACTIVE covers whole pages and starts at a 128K sector boundary:
        // every erase(page) maps to exactly one physical sector.
        assert_eq!(ACTIVE_BASE % PAGE_SIZE, 0);
        assert_eq!(ACTIVE_LEN % PAGE_SIZE, 0);
        assert_eq!(ACTIVE_BASE + ACTIVE_LEN, 0x0808_0000); // end of flash
        // DFU >= ACTIVE + 1 page, whole pages, inside the chip.
        assert!(DFU_LEN >= ACTIVE_LEN + PAGE_SIZE);
        assert_eq!(DFU_LEN % PAGE_SIZE, 0);
        assert_eq!((DFU_BASE + DFU_LEN) <= 0x000E_0000, true); // below config A
        // STATE holds 2 + 2*active_pages words of WRITE_SIZE=1.
        let words_needed = 2 + 2 * (ACTIVE_LEN / PAGE_SIZE);
        assert!(STATE_LEN >= words_needed);
        // STATE + DFU stay clear of config/littlefs partitions.
        assert!(DFU_BASE + DFU_LEN <= 0xE_0000);
    }

    #[test]
    fn b64_roundtrip_known() {
        // 32 zero bytes == 43 'A' chars + one '=' pad
        let mut out = [0u8; 32];
        let n = b64_decode(b"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=", &mut out).unwrap();
        assert_eq!(n, 32);
        assert_eq!(out, [0u8; 32]);

        // base64.b64encode(bytes(range(32)))
        let mut out2 = [0u8; 32];
        let n2 = b64_decode(b"AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8=", &mut out2).unwrap();
        assert_eq!(n2, 32);
        assert_eq!(out2[0], 0);
        assert_eq!(out2[31], 31);

        assert!(b64_decode(b"!!!", &mut out).is_none());
    }
}
