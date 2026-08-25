//! Config store A/B slot codec (src/storage/config_store.c).
//!
//! On-disk slot format (little-endian scalar fields, io_cfg native bytes):
//! ```text
//! [0]  magic "IOCF"          4 B
//! [4]  generation            u32 LE
//! [8]  len                   u16 LE (== 26, size of IoCfg)
//! [10] struct io_cfg         26 B, 13 packed u16 LE
//! [36] crc32_ieee([0..35])   u32 LE
//! ```
//! Writes go header -> body -> CRC (CRC last), so a torn write always leaves
//! a CRC mismatch and the peer slot stays loadable.

use crate::crc::crc32_ieee;

pub const CFG_SLOT_SIZE: u32 = 0x8000;
/// Address of slot A inside the W25Q storage partition (flash_layout.h).
pub const CFG_SLOT_A: u32 = 0x000E_0000;
pub const CFG_SLOT_B: u32 = CFG_SLOT_A + CFG_SLOT_SIZE;

pub const CFG_MAGIC: [u8; 4] = *b"IOCF";
pub const CFG_HDR_LEN: usize = 10;
pub const CFG_CRC_OFF: usize = 36;
pub const CFG_REC_LEN: usize = 40;

/// The 10 persisted configuration keys (13 packed u16, byte-identical to
/// struct io_cfg on disk).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IoCfg {
    pub di_en: u16,
    pub ai_en: u16,
    pub di_si: u16,
    pub ai_si: u16,
    pub his: u16,
    pub can_id: u16,
    pub can_bps: u16,
    pub rs485_bps: u16,
    pub slave_id: u16,
    /// 4 IP octets as 4 u16 (matches the C struct: `uint16_t ip[4]`).
    pub ip: [u16; 4],
}

impl IoCfg {
    /// Factory defaults (match the Zephyr holding_reg[] initializers).
    pub const fn defaults() -> Self {
        Self {
            di_en: 0xFFFF,
            ai_en: 0x000F,
            di_si: 200,
            ai_si: 200,
            his: 0,
            can_id: 0x0111,
            can_bps: 250,
            rs485_bps: 9600,
            slave_id: 1,
            ip: [192, 168, 12, 101],
        }
    }

    pub fn to_bytes(&self) -> [u8; 26] {
        let mut b = [0u8; 26];
        let words = [
            self.di_en,
            self.ai_en,
            self.di_si,
            self.ai_si,
            self.his,
            self.can_id,
            self.can_bps,
            self.rs485_bps,
            self.slave_id,
            self.ip[0],
            self.ip[1],
            self.ip[2],
            self.ip[3],
        ];
        for (i, w) in words.iter().enumerate() {
            b[2 * i..2 * i + 2].copy_from_slice(&w.to_le_bytes());
        }
        b
    }

    pub fn from_bytes(b: &[u8; 26]) -> Self {
        let w = |i: usize| u16::from_le_bytes([b[2 * i], b[2 * i + 1]]);
        Self {
            di_en: w(0),
            ai_en: w(1),
            di_si: w(2),
            ai_si: w(3),
            his: w(4),
            can_id: w(5),
            can_bps: w(6),
            rs485_bps: w(7),
            slave_id: w(8),
            ip: [w(9), w(10), w(11), w(12)],
        }
    }
}

/// Backend flash: erase must be sector-aligned; write must respect the
/// 256 B page / no-cross constraint of the C io_flash interface.
pub trait Flash {
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), ()>;
    fn write(&mut self, addr: u32, buf: &[u8]) -> Result<(), ()>;
    fn erase(&mut self, addr: u32, len: u32) -> Result<(), ()>;
}

enum SlotRead {
    Valid(IoCfg, u32),
    Invalid,
}

fn slot_read(f: &mut dyn Flash, addr: u32) -> Result<SlotRead, ()> {
    let mut rec = [0u8; CFG_REC_LEN];
    f.read(addr, &mut rec)?;
    if rec[..4] != CFG_MAGIC {
        return Ok(SlotRead::Invalid);
    }
    if u16::from_le_bytes([rec[8], rec[9]]) != 26 {
        return Ok(SlotRead::Invalid);
    }
    if u32::from_le_bytes(rec[36..40].try_into().unwrap()) != crc32_ieee(&rec[..36]) {
        return Ok(SlotRead::Invalid);
    }
    let cfg = IoCfg::from_bytes(rec[10..36].try_into().unwrap());
    let gen = u32::from_le_bytes(rec[4..8].try_into().unwrap());
    Ok(SlotRead::Valid(cfg, gen))
}

/// Result of [`config_store_init`]: active config plus the slot holding it.
pub struct ConfigState {
    pub cfg: IoCfg,
    pub cur_slot: Option<u32>,
}

/// Read both slots and pick the active one: higher generation wins, a valid
/// slot beats an invalid one, both-invalid falls back to defaults.
pub fn config_store_init(f: &mut dyn Flash) -> Result<ConfigState, ()> {
    let mut state = ConfigState {
        cfg: IoCfg::defaults(),
        cur_slot: None,
    };
    let (a, b) = (slot_read(f, CFG_SLOT_A)?, slot_read(f, CFG_SLOT_B)?);
    match (a, b) {
        (SlotRead::Valid(ca, ga), SlotRead::Valid(cb, gb)) => {
            if ga >= gb {
                state.cfg = ca;
                state.cur_slot = Some(CFG_SLOT_A);
            } else {
                state.cfg = cb;
                state.cur_slot = Some(CFG_SLOT_B);
            }
        }
        (SlotRead::Valid(c, _), SlotRead::Invalid) => {
            state.cfg = c;
            state.cur_slot = Some(CFG_SLOT_A);
        }
        (SlotRead::Invalid, SlotRead::Valid(c, _)) => {
            state.cfg = c;
            state.cur_slot = Some(CFG_SLOT_B);
        }
        (SlotRead::Invalid, SlotRead::Invalid) => {}
    }
    Ok(state)
}

/// Encode one slot record; exposed for tests.
pub fn encode_record(cfg: &IoCfg, gen: u32) -> [u8; CFG_REC_LEN] {
    let mut rec = [0u8; CFG_REC_LEN];
    rec[..4].copy_from_slice(&CFG_MAGIC);
    rec[4..8].copy_from_slice(&gen.to_le_bytes());
    rec[8..10].copy_from_slice(&26u16.to_le_bytes());
    rec[10..36].copy_from_slice(&cfg.to_bytes());
    let crc = crc32_ieee(&rec[..36]);
    rec[36..40].copy_from_slice(&crc.to_le_bytes());
    rec
}

/// Pure validation/decode of a raw 40-byte slot record (slot_read minus I/O).
pub fn decode_record(rec: &[u8; CFG_REC_LEN]) -> Option<(IoCfg, u32)> {
    if rec[..4] != CFG_MAGIC {
        return None;
    }
    if u16::from_le_bytes([rec[8], rec[9]]) != 26 {
        return None;
    }
    if u32::from_le_bytes(rec[36..40].try_into().unwrap()) != crc32_ieee(&rec[..36]) {
        return None;
    }
    let cfg = IoCfg::from_bytes(rec[10..36].try_into().unwrap());
    let gen = u32::from_le_bytes(rec[4..8].try_into().unwrap());
    Some((cfg, gen))
}

/// Persist cfg to the inactive slot (erase, then header/body/CRC writes),
/// generation + 1, and return the new active slot address. The caller tracks
/// the current generation between calls (mirrors config_store.c's statics).
pub fn config_store_save_gen(
    f: &mut dyn Flash,
    cfg: &IoCfg,
    cur_slot: Option<u32>,
    cur_gen: u32,
) -> Result<u32, ()> {
    let tgt = if cur_slot == Some(CFG_SLOT_A) {
        CFG_SLOT_B
    } else {
        CFG_SLOT_A
    };
    let rec = encode_record(cfg, cur_gen + 1);
    f.erase(tgt, CFG_SLOT_SIZE)?;
    f.write(tgt, &rec[..CFG_HDR_LEN])?;
    f.write(tgt + CFG_HDR_LEN as u32, &rec[CFG_HDR_LEN..36])?;
    f.write(tgt + CFG_CRC_OFF as u32, &rec[36..40])?;
    Ok(tgt)
}

/// Factory reset: erase both slots.
pub fn config_store_erase_all(f: &mut dyn Flash) -> Result<(), ()> {
    f.erase(CFG_SLOT_A, CFG_SLOT_SIZE)?;
    f.erase(CFG_SLOT_B, CFG_SLOT_SIZE)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct FakeFlash {
        mem: Vec<u8>,
        torn_writes: bool,
    }

    impl FakeFlash {
        fn new() -> Self {
            Self {
                mem: vec![0xFF; 0x100000],
                torn_writes: false,
            }
        }
    }

    impl Flash for FakeFlash {
        fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), ()> {
            buf.copy_from_slice(&self.mem[addr as usize..addr as usize + buf.len()]);
            Ok(())
        }
        fn write(&mut self, addr: u32, buf: &[u8]) -> Result<(), ()> {
            for (i, b) in buf.iter().enumerate() {
                let v = addr as usize + i;
                // NOR semantics: can only clear bits (unless "erased")
                self.mem[v] &= *b;
            }
            Ok(())
        }
        fn erase(&mut self, addr: u32, len: u32) -> Result<(), ()> {
            for v in &mut self.mem[addr as usize..addr as usize + len as usize] {
                *v = 0xFF;
            }
            Ok(())
        }
    }

    #[test]
    fn defaults_roundtrip() {
        let b = IoCfg::defaults().to_bytes();
        assert_eq!(IoCfg::from_bytes(&b), IoCfg::defaults());
    }

    #[test]
    fn fresh_flash_yields_defaults() {
        let mut f = FakeFlash::new();
        let st = config_store_init(&mut f).unwrap();
        assert_eq!(st.cfg, IoCfg::defaults());
        assert_eq!(st.cur_slot, None);
    }

    #[test]
    fn save_then_reload() {
        let mut f = FakeFlash::new();
        let mut cfg = IoCfg::defaults();
        cfg.slave_id = 7;
        cfg.rs485_bps = 19200;
        let slot = config_store_save_gen(&mut f, &cfg, None, 0).unwrap();
        assert_eq!(slot, CFG_SLOT_A);
        let st = config_store_init(&mut f).unwrap();
        assert_eq!(st.cfg, cfg);
        assert_eq!(st.cur_slot, Some(CFG_SLOT_A));

        // second save flips to B with higher generation
        let slot2 = config_store_save_gen(&mut f, &cfg, st.cur_slot, 1).unwrap();
        assert_eq!(slot2, CFG_SLOT_B);
        let st2 = config_store_init(&mut f).unwrap();
        assert_eq!(st2.cur_slot, Some(CFG_SLOT_B));
    }

    #[test]
    fn higher_generation_wins() {
        let mut f = FakeFlash::new();
        let mut a = IoCfg::defaults();
        a.slave_id = 3;
        let mut b = IoCfg::defaults();
        b.slave_id = 9;
        // A gen 5, B gen 2 -> A wins
        f.write(CFG_SLOT_A, &encode_record(&a, 5)).unwrap();
        f.write(CFG_SLOT_B, &encode_record(&b, 2)).unwrap();
        let st = config_store_init(&mut f).unwrap();
        assert_eq!(st.cfg.slave_id, 3);
        assert_eq!(st.cur_slot, Some(CFG_SLOT_A));
    }

    #[test]
    fn bad_crc_falls_back_to_peer() {
        let mut f = FakeFlash::new();
        let mut rec = encode_record(&IoCfg::defaults(), 9);
        rec[40 - 1] ^= 0x01; // corrupt CRC
        f.write(CFG_SLOT_B, &rec).unwrap();
        let mut other = IoCfg::defaults();
        other.slave_id = 4;
        f.write(CFG_SLOT_A, &encode_record(&other, 1)).unwrap();
        let st = config_store_init(&mut f).unwrap();
        assert_eq!(st.cfg.slave_id, 4);
        assert_eq!(st.cur_slot, Some(CFG_SLOT_A));
    }

    #[test]
    fn torn_write_body_only_leaves_invalid() {
        let mut f = FakeFlash::new();
        let rec = encode_record(&IoCfg::defaults(), 1);
        // header + body written, CRC missing (torn)
        f.write(CFG_SLOT_A, &rec[..36]).unwrap();
        let st = config_store_init(&mut f).unwrap();
        assert_eq!(st.cfg, IoCfg::defaults());
        assert_eq!(st.cur_slot, None);
    }
}
