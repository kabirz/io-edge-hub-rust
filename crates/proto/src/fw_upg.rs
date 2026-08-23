//! Firmware-upgrade protocol helpers, port of src/fw/fw_upg.c +
//! deps/mcuboot bootutil trailer layout (SWAP_USING_SCRATCH, MAX_ALIGN 8).
//!
//! - CRC16-CCITT: Zephyr sys/crc.h crc16_ccitt (reflected, poly 0x1021,
//!   init 0) — identical to the host-side helpers/fwupd.py and
//!   tools/firmware_upgrade.py; the final check hashes the bytes read back
//!   from slot1 (validating the programming result, not the source).
//! - TLV keyhash: parsed from the MCUboot image written to slot1 (magic
//!   0x96F3B83D, TLV block magic 0x6907, KEYHASH tag 0x01).
//! - Trailer offsets: bootutil_misc.h formulas for BOOT_MAX_ALIGN == 8.

pub const FW_KEYHASH_LEN: usize = 32;

/// SHA-256(RSA-2048 public key, PKCS#1 DER) of keys/root-rsa2048.pem —
/// the same fingerprint imgtool bakes into the image KEYHASH TLV and
/// tools/gen_keyhash.py compiles into the C firmware (fw_keyhash.h).
/// Fixed for the lifetime of the signing key.
pub const FW_KEYHASH: [u8; FW_KEYHASH_LEN] = [
    0xfb, 0x73, 0x8f, 0x7d, 0xcf, 0x8c, 0x1a, 0x3a, 0x89, 0x88, 0xae, 0x60, 0x85, 0xd5, 0x2f,
    0x06, 0xaf, 0x2a, 0x25, 0x61, 0x33, 0xee, 0x1c, 0x06, 0x84, 0xf7, 0x14, 0x25, 0xaa, 0x15,
    0x7f, 0xac,
];

/// Slot1 (upgrade staging) on the W25Q NOR: 0x0..0x70000.
pub const SLOT1_SIZE: u32 = 0x70000;

/// Parsed MCUboot image header fields needed to locate the TLV block.
pub struct ImageTlvPos {
    /// absolute offset of the TLV block within the image
    pub tlv_off: u32,
    /// size of the TLV block in bytes
    pub tlv_size: u16,
}

/// Parse [magic 4][.. hdr_size @8 LE16][.. img_size @12 LE32] and derive the
/// TLV block position. `hdr32` is the first 32 bytes of the image.
pub fn image_tlv_pos(hdr32: &[u8], hdr_size_expect: u32) -> Option<ImageTlvPos> {
    if hdr32.len() < 16 {
        return None;
    }
    let magic = u32::from_le_bytes([hdr32[0], hdr32[1], hdr32[2], hdr32[3]]);
    if magic != 0x96F3_B83D {
        return None;
    }
    let hdr_size = u16::from_le_bytes([hdr32[8], hdr32[9]]) as u32;
    let img_size = u32::from_le_bytes([hdr32[12], hdr32[13], hdr32[14], hdr32[15]]);
    if hdr_size != hdr_size_expect {
        return None;
    }
    let tlv_off = hdr_size + img_size;
    if tlv_off % 4 != 0 {
        return None;
    }
    Some(ImageTlvPos { tlv_off, tlv_size: 0 })
}

/// Walk a `tlv` slice that begins with the TLV block header
/// [magic LE16 == 0x6907][size LE16] followed by (tag LE16, len LE16, data)
/// entries; return the KEYHASH (tag 0x01, len 32) payload slice.
pub fn keyhash_in_tlv(tlv: &[u8]) -> Option<&[u8]> {
    if tlv.len() < 4 {
        return None;
    }
    let magic = u16::from_le_bytes([tlv[0], tlv[1]]);
    let size = u16::from_le_bytes([tlv[2], tlv[3]]) as usize;
    if magic != 0x6907 || size == 0 || size > tlv.len() {
        return None;
    }
    let end = size.min(tlv.len());
    let mut off = 4usize;
    while off + 4 <= end {
        let tag = u16::from_le_bytes([tlv[off], tlv[off + 1]]);
        let len = u16::from_le_bytes([tlv[off + 2], tlv[off + 3]]) as usize;
        if tag == 0x0001 && len == FW_KEYHASH_LEN {
            let s = off + 4;
            if s + len <= end {
                return Some(&tlv[s..s + len]);
            }
            return None;
        }
        off += 4 + len.div_ceil(4) * 4;
    }
    None
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

// ==================== MCUboot trailer (BOOT_MAX_ALIGN = 8) ====================

/// bootutil_public.c boot_img_magic (BOOT_MAX_ALIGN == 8 variant).
pub const BOOT_MAGIC: [u8; 16] = [
    0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79,
    0x80,
];

pub const BOOT_FLAG_SET: u8 = 1;
pub const BOOT_SWAP_TYPE_TEST: u8 = 2;
pub const BOOT_SWAP_TYPE_PERM: u8 = 3;

/// boot_magic_off: fa_size - BOOT_MAGIC_SZ
pub const fn magic_off() -> u32 {
    SLOT1_SIZE - 16
}

/// boot_image_ok_off: ALIGN_DOWN(magic_off - 8, 8)
pub const fn image_ok_off() -> u32 {
    (SLOT1_SIZE - 16 - 8) & !7
}

/// boot_copy_done_off: image_ok_off - 8
pub const fn copy_done_off() -> u32 {
    image_ok_off() - 8
}

/// boot_swap_info_off: copy_done_off - 8
pub const fn swap_info_off() -> u32 {
    copy_done_off() - 8
}

/// boot_swap_size_off: swap_info_off - 8
pub const fn swap_size_off() -> u32 {
    swap_info_off() - 8
}

/// boot_write_trailer_flag buffer: [flag][0xff x 7]
pub fn trailer_flag(flag: u8) -> [u8; 8] {
    let mut b = [0xFFu8; 8];
    b[0] = flag;
    b
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
    fn tlv_walk_finds_keyhash() {
        // TLV block: magic 0x6907, size; SHA256 entry (tag 0x10), SIG (0x20),
        // KEYHASH (0x01, 32B) — lengths padded to 4 like imgtool writes.
        let mut tlv = Vec::new();
        let mut entry = |tag: u16, data: &[u8], v: &mut Vec<u8>| {
            v.extend_from_slice(&tag.to_le_bytes());
            v.extend_from_slice(&(data.len() as u16).to_le_bytes());
            v.extend_from_slice(data);
            let pad = data.len().div_ceil(4) * 4 - data.len();
            v.extend(std::iter::repeat(0xFF).take(pad));
        };
        entry(0x10, &[0xAA; 32], &mut tlv);
        entry(0x20, &[0xBB; 256], &mut tlv);
        entry(0x01, &[0xCC; 32], &mut tlv);
        let mut buf = Vec::new();
        // tlv_size spans the whole block INCLUDING the 4-byte header (fwupd.py)
        buf.extend_from_slice(&0x6907u16.to_le_bytes());
        buf.extend_from_slice(&((tlv.len() + 4) as u16).to_le_bytes());
        buf.extend_from_slice(&tlv);

        let kh = keyhash_in_tlv(&buf).expect("keyhash");
        assert_eq!(kh, &[0xCC; 32]);
    }

    #[test]
    fn tlv_missing_keyhash() {
        let mut buf = vec![];
        buf.extend_from_slice(&0x6907u16.to_le_bytes());
        buf.extend_from_slice(&8u16.to_le_bytes());
        buf.extend_from_slice(&0x10u16.to_le_bytes());
        buf.extend_from_slice(&4u16.to_le_bytes());
        buf.extend_from_slice(&[1, 2, 3, 4]);
        assert!(keyhash_in_tlv(&buf).is_none());
    }

    #[test]
    fn image_header_parse() {
        let mut hdr = [0u8; 32];
        hdr[0..4].copy_from_slice(&0x96F3_B83Du32.to_le_bytes());
        hdr[8..10].copy_from_slice(&0x200u16.to_le_bytes());
        hdr[12..16].copy_from_slice(&190_000u32.to_le_bytes());
        let pos = image_tlv_pos(&hdr, 0x200).expect("hdr");
        assert_eq!(pos.tlv_off, 0x200 + 190_000);

        hdr[0] = 0; // bad magic
        assert!(image_tlv_pos(&hdr, 0x200).is_none());
        hdr[0..4].copy_from_slice(&0x96F3_B83Du32.to_le_bytes());
        hdr[8..10].copy_from_slice(&0x100u16.to_le_bytes()); // wrong hdr size
        assert!(image_tlv_pos(&hdr, 0x200).is_none());
    }

    #[test]
    fn trailer_offsets_match_bootutil() {
        // S = 0x70000: magic at S-16, image_ok ALIGN_DOWN(S-16-8,8),
        // copy_done -8, swap_info -8, swap_size -8
        assert_eq!(magic_off(), 0x6FFF0);
        assert_eq!(image_ok_off(), 0x6FFE8);
        assert_eq!(copy_done_off(), 0x6FFE0);
        assert_eq!(swap_info_off(), 0x6FFD8);
        assert_eq!(swap_size_off(), 0x6FFD0);
        assert_eq!(BOOT_MAGIC, [
            0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6,
            0x79, 0x80
        ]);
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
