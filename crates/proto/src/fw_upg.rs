//! Firmware-upgrade protocol helpers for the embassy-boot layout.
//!
//! Update payload (as transferred over UDP/WS/CAN): raw application binary
//! followed by a 64-byte ed25519 signature of SHA-512(binary) (pure Ed25519
//! over the 64-byte digest — the scheme embassy-boot/salty verify on the
//! device). The device stages the payload in the DFU partition on the NOR,
//! re-reads and CRC-checks it, then verifies the signature before marking
//! the state partition for a swap on the next boot.
//!
//! Partition map (must match the bootloader and memory.x):
//! - internal flash: bootloader @0x08000000 (sectors 0-4), active (app)
//!   @0x08020000, 3 x 128 KiB = 384 KiB
//! - W25Q128 NOR: DFU (staging) @0x0 size 512 KiB, state @0x80000 size 4 KiB,
//!   config A/B @0xE0000/0xE8000 and littlefs @0xF0000 unchanged
//!
//! CRC16-CCITT: Zephyr sys/crc.h crc16_ccitt (reflected, poly 0x1021, init 0)
//! — identical to the host-side helpers/fwupd.py; the final check hashes the
//! bytes read back from the DFU partition (validating the programming
//! result, not the source).

use core::cmp::min;

pub const FW_KEYHASH_LEN: usize = 32;

/// Raw ed25519 public key of keys/ed25519.pub (tools/gen_ed25519.py).
pub const FW_PUBKEY: [u8; 32] = [
    0x3a, 0xaf, 0x77, 0x59, 0x3b, 0x20, 0x00, 0xdb, 0x30, 0x29, 0xcb, 0x29, 0xac, 0xcb, 0x70, 0x7f,
    0xa2, 0xfd, 0x40, 0x86, 0x95, 0x27, 0x9e, 0x46, 0xec, 0xb1, 0x8f, 0xd4, 0x50, 0x0d, 0x7e, 0xc1,
];

/// SHA-256(FW_PUBKEY) — the 32-byte fingerprint every upgrade channel sends
/// in its START command; a mismatch aborts the session before any flash
/// erase (wrong-key images must not clobber the staging partition).
pub const FW_KEYHASH: [u8; FW_KEYHASH_LEN] = [
    0x7c, 0xb9, 0xc1, 0xc5, 0x52, 0x4d, 0xf6, 0xbd, 0xa9, 0x73, 0xb7, 0x51, 0xda, 0xd4, 0x20, 0x1d,
    0x2d, 0x5d, 0x89, 0x08, 0x66, 0x8e, 0xaf, 0xdf, 0x44, 0x19, 0x23, 0x99, 0x69, 0xc6, 0x85, 0x1f,
];

/// Application partition (internal flash): starts right after the 5th
/// sector boundary and spans three 128 KiB sectors.
pub const APP_FLASH_START: u32 = 0x0802_0000;
pub const APP_MAX_SIZE: u32 = 0x6_0000;

/// DFU staging partition on the W25Q (embassy-boot requires it to be at
/// least one swap page (128 KiB) larger than the active partition).
pub const DFU_SIZE: u32 = 0x8_0000;

/// embassy-boot state partition on the W25Q: swap trigger magic + copy
/// progress indices. Byte layout (WRITE_SIZE = 1):
/// [0]=magic [1]=progress-valid [2..]=progress indices.
pub const STATE_OFF: u32 = 0x8_0000;
pub const STATE_SIZE: u32 = 0x1000;

/// ed25519 signature appended to the update payload.
pub const SIG_LEN: usize = 64;

/// embassy-boot state-partition magics (embassy-boot 0.7 lib.rs).
pub const STATE_MAGIC_BOOT: u8 = 0xD0;
pub const STATE_MAGIC_SWAP: u8 = 0xF0;
pub const STATE_MAGIC_REVERT: u8 = 0xC0;
pub const STATE_MAGIC_DFU_DETACH: u8 = 0xE0;

/// Classify the first state byte (embassy-boot `State::from` semantics).
pub fn state_from_magic(b: u8) -> &'static str {
    match b {
        STATE_MAGIC_SWAP => "swap",
        STATE_MAGIC_REVERT => "revert",
        STATE_MAGIC_DFU_DETACH => "dfu-detach",
        _ => "boot",
    }
}

/// Minimum/maximum accepted payload sizes: an update must carry more than
/// the bare signature and fit the DFU partition.
pub const fn payload_ok(total: u32) -> bool {
    total > SIG_LEN as u32 && total <= DFU_SIZE
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

// ==================== SHA-256 (host tests + keyhash tooling) ====================

const SHA256_K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

/// SHA-256 of `data`, for host tests and keyhash derivation (kept dependency
/// free; the firmware itself hashes with salty's SHA-512 only).
pub fn sha256(data: &[u8]) -> [u8; 32] {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let compress = |chunk: &[u8; 64], h: &mut [u32; 8]| {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let (mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh) =
            (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(SHA256_K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    };

    // whole blocks straight from the input
    let full = data.len() / 64 * 64;
    let mut block = [0u8; 64];
    for chunk in data[..full].chunks(64) {
        block.copy_from_slice(chunk);
        compress(&block, &mut h);
    }
    // tail: 0x80 + zeros so the block ends with the 8-byte big-endian bit
    // length (one or two final blocks)
    let tail = &data[full..];
    block = [0u8; 64];
    block[..tail.len()].copy_from_slice(tail);
    block[tail.len()] = 0x80;
    let bitlen = (data.len() as u64).wrapping_mul(8);
    if tail.len() + 9 <= 64 {
        block[56..64].copy_from_slice(&bitlen.to_be_bytes());
        compress(&block, &mut h);
    } else {
        compress(&block, &mut h);
        block = [0u8; 64];
        block[56..64].copy_from_slice(&bitlen.to_be_bytes());
        compress(&block, &mut h);
    }

    let mut out = [0u8; 32];
    for (i, v) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&v.to_be_bytes());
    }
    out
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

/// Split an update payload into (image, signature) views; None when the
/// payload cannot hold a signature.
pub fn payload_split(payload: &[u8]) -> Option<(&[u8], &[u8])> {
    if payload.len() < SIG_LEN {
        return None;
    }
    let cut = payload.len() - SIG_LEN;
    Some((&payload[..cut], &payload[cut..]))
}

/// Clamp helper shared by the transfer loops.
pub fn clamp_chunk(remaining: u32, max: usize) -> usize {
    min(remaining as usize, max)
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
    fn keyhash_is_sha256_of_pubkey() {
        assert_eq!(FW_KEYHASH, sha256(&FW_PUBKEY));
    }

    #[test]
    fn sha256_known_vectors() {
        // sha256("") and sha256("abc") FIPS 180-4 check values
        let e3b0 = hex("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(sha256(b""), e3b0);
        let abc = hex("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        assert_eq!(sha256(b"abc"), abc);
        // > one block
        let long = hex("41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
        assert_eq!(sha256(&vec![b'a'; 1000]), long);
    }

    #[test]
    fn partition_invariants() {
        // active: multiple of the 128 KiB swap page, inside the 512 KiB bank
        assert_eq!(APP_FLASH_START, 0x0802_0000);
        assert_eq!(APP_MAX_SIZE % 0x2_0000, 0);
        assert_eq!(APP_FLASH_START + APP_MAX_SIZE, 0x0808_0000);
        // DFU >= active + one swap page (embassy-boot assert_partitions)
        assert!(DFU_SIZE - APP_MAX_SIZE >= 0x2_0000);
        // state: 2 + 4*(active/page) words needed, WRITE_SIZE = 1
        let need = 2 + 4 * (APP_MAX_SIZE / 0x2_0000);
        assert!(STATE_SIZE >= need);
        // DFU + state must not reach the config slots at 0xE0000
        assert!(STATE_OFF + STATE_SIZE <= 0xE_0000);
        // state magic classes
        assert_eq!(state_from_magic(STATE_MAGIC_SWAP), "swap");
        assert_eq!(state_from_magic(STATE_MAGIC_BOOT), "boot");
        assert_eq!(state_from_magic(0xFF), "boot");
    }

    #[test]
    fn payload_split_and_bounds() {
        assert!(!payload_ok(64));
        assert!(!payload_ok(0));
        assert!(payload_ok(65));
        assert!(!payload_ok(DFU_SIZE + 1));
        let mut p = vec![0xAAu8; 100];
        p[36..].fill(0xBB);
        let (img, sig) = payload_split(&p).unwrap();
        assert_eq!(img, &[0xAA; 36]);
        assert_eq!(sig, &[0xBB; 64]);
        assert!(payload_split(&[0u8; 8]).is_none());
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

    fn hex(s: &str) -> [u8; 32] {
        let b = s.as_bytes();
        let mut out = [0u8; 32];
        for i in 0..32 {
            out[i] = u8::from_str_radix(core::str::from_utf8(&b[i * 2..i * 2 + 2]).unwrap(), 16)
                .unwrap();
        }
        out
    }
}
