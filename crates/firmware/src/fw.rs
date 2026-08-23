//! Firmware-upgrade session over the W25Q slot1, port of src/fw/fw_upg.c.
//!
//! Same return-code contract as the C code: start() gives 0 ok / -2 keyhash
//! mismatch / -3 busy / -1 other. start() erases the whole slot (the trailer
//! area at the slot tail must be clean or a later trailer write fails);
//! writes are page-buffered; finish() flushes the tail, then verifies by
//! reading slot1 back (CRC16 over the received image + TLV keyhash).
//! NOR access goes through the storage task's driver mutex, so littlefs
//! traffic and upgrade traffic serialize on the same SPI bus.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use io_edge_hub_proto::fw_upg as upg;

use crate::storage::nor_with;

const PAGE_SZ: usize = 256;
const READ_CHUNK: usize = 64;
const IMG_HDR_SIZE: u32 = 0x200;
const TLV_READ: usize = 512;

struct FwSession {
    active: bool,
    failed: bool,
    total: u32,
    written: u32,
    page: [u8; PAGE_SZ],
    page_len: usize,
}

static FW: Mutex<CriticalSectionRawMutex, RefCell<FwSession>> = Mutex::new(RefCell::new(
    FwSession {
        active: false,
        failed: false,
        total: 0,
        written: 0,
        page: [0; PAGE_SZ],
        page_len: 0,
    },
));

/// Last finish() diagnostics (UDP debug cmd 0xFB): written/computed crc/
/// expected crc/tlv ok + first and last 16 readback bytes.
pub static FW_DBG: Mutex<CriticalSectionRawMutex, RefCell<[u32; 16]>> =
    Mutex::new(RefCell::new([0; 16]));

fn dbg_store(vals: &[u32; 16]) {
    critical_section::with(|_cs| {
        FW_DBG.lock(|d| *d.borrow_mut() = *vals);
    });
}

fn page_flush(s: &mut FwSession) -> bool {
    if s.page_len > 0 {
        let ok = matches!(
            nor_with(|nor| nor.write(s.written, &s.page[..s.page_len])),
            Some(Ok(()))
        );
        s.page_len = 0;
        if !ok {
            return false;
        }
    }
    true
}

/// Whole-slot erase; caller checks active/keyhash/size first.
fn erase_slot1() -> bool {
    matches!(nor_with(|nor| nor.erase(0, upg::SLOT1_SIZE)), Some(Ok(())))
}

pub fn start(total: u32, keyhash: Option<&[u8; upg::FW_KEYHASH_LEN]>) -> i32 {
    let mut rc = 0i32;
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let mut s = f.borrow_mut();
            if s.active {
                rc = -3;
                return;
            }
            if total < 64 || total > upg::SLOT1_SIZE {
                rc = -1;
                return;
            }
            if let Some(kh) = keyhash {
                if kh != &upg::FW_KEYHASH {
                    rc = -2;
                    return;
                }
            }
            if !erase_slot1() {
                rc = -1;
                return;
            }
            s.active = true;
            s.failed = false;
            s.total = total;
            s.written = 0;
            s.page_len = 0;
        })
    });
    rc
}

pub fn write(data: &[u8]) -> bool {
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let mut s = f.borrow_mut();
            if !s.active || s.failed {
                return false;
            }
            if s.written + s.page_len as u32 + data.len() as u32 > s.total {
                s.failed = true;
                return false;
            }
            let mut data = data;
            while !data.is_empty() {
                let plen = s.page_len;
                let chunk = (PAGE_SZ - plen).min(data.len());
                s.page[plen..plen + chunk].copy_from_slice(&data[..chunk]);
                s.page_len = plen + chunk;
                data = &data[chunk..];
                if s.page_len == PAGE_SZ {
                    if !page_flush(&mut s) {
                        s.failed = true;
                        return false;
                    }
                    s.written += PAGE_SZ as u32;
                }
            }
            true
        })
    })
}

pub fn abort() {
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let mut s = f.borrow_mut();
            s.active = false;
            s.failed = false;
            s.page_len = 0;
            s.written = 0;
        })
    });
}

/// Flush + verify (readback CRC when `crc` is Some) + TLV keyhash check.
/// Resets the session; `received()`/`total()` read 0 afterwards.
pub fn finish(crc: Option<u16>) -> bool {
    let mut ok = false;
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let mut s = f.borrow_mut();
            if !s.active {
                return;
            }
            s.active = false;

            let tail = s.page_len as u32;
            if s.failed || !page_flush(&mut s) {
                dbg_store(&[1, s.written + tail, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
                return;
            }
            s.written += tail;
            if s.written != s.total {
                dbg_store(&[2, s.written, s.total, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
                return;
            }

            let total = s.total;
            let mut crc_v = 0u16;
            let mut buf = [0u8; READ_CHUNK];
            let mut verified = true;
            let mut off = 0u32;
            while off < total {
                let n = (total - off).min(READ_CHUNK as u32) as usize;
                let got = matches!(
                    nor_with(|nor| nor.read(off, &mut buf[..n])),
                    Some(Ok(()))
                );
                if !got {
                    verified = false;
                    break;
                }
                if off == 0 {
                    let mut d = [0u32; 16];
                    d[0] = 3;
                    d[1] = total;
                    for (i, w) in buf[..16].chunks(4).enumerate() {
                        d[4 + i] = u32::from_le_bytes([w[0], w[1], w[2], w[3]]);
                    }
                    dbg_store(&d);
                }
                crc_v = upg::crc16_ccitt(crc_v, &buf[..n]);
                off += n as u32;
            }
            if !verified {
                dbg_store(&[4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
                return;
            }
            let tlv_ok = tlv_keyhash_ok();
            let expect = crc.unwrap_or(0);
            if crc_v != expect || !tlv_ok {
                // full diagnostics: computed/expected crc + tail bytes
                let mut tail16 = [0u8; 16];
                let tail_off = total.saturating_sub(16);
                let _ = nor_with(|nor| nor.read(tail_off, &mut tail16));
                let mut d = [0u32; 16];
                d[0] = 5;
                d[1] = crc_v as u32;
                d[2] = expect as u32;
                d[3] = tlv_ok as u32;
                for (i, w) in tail16.chunks(4).enumerate() {
                    d[8 + i] = u32::from_le_bytes([w[0], w[1], w[2], w[3]]);
                }
                dbg_store(&d);
                return;
            }
            ok = true;
        })
    });
    ok
}

/// Read the image header from slot1, locate the TLV block, check its KEYHASH.
fn tlv_keyhash_ok() -> bool {
    let mut hdr = [0u8; 32];
    if !matches!(nor_with(|nor| nor.read(0, &mut hdr)), Some(Ok(()))) {
        return false;
    }
    let pos = match upg::image_tlv_pos(&hdr, IMG_HDR_SIZE) {
        Some(p) => p,
        None => return false,
    };
    if pos.tlv_off + 4 > upg::SLOT1_SIZE {
        return false;
    }
    let mut tlv = [0u8; TLV_READ];
    let read = (TLV_READ as u32).min(upg::SLOT1_SIZE - pos.tlv_off) as usize;
    if !matches!(nor_with(|nor| nor.read(pos.tlv_off, &mut tlv[..read])), Some(Ok(()))) {
        return false;
    }
    matches!(upg::keyhash_in_tlv(&tlv[..read]), Some(kh) if kh == &upg::FW_KEYHASH)
}

pub fn received() -> u32 {
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let s = f.borrow();
            s.written + s.page_len as u32
        })
    })
}

pub fn total() -> u32 {
    critical_section::with(|_cs| {
        FW.lock(|f| {
            let s = f.borrow();
            if s.active {
                s.total
            } else {
                0
            }
        })
    })
}

pub fn active() -> bool {
    critical_section::with(|_cs| {
        FW.lock(|f| f.borrow().active)
    })
}

// ==================== MCUboot trailer (boot_set_pending) ====================

/// bootutil boot_set_next(slot1, active=false, confirm=permanent) on a
/// freshly erased trailer: magic, then image_ok + swap_info (PERM/TEST).
/// Offsets/flags are the bootutil_priv/bootutil_misc formulas for
/// BOOT_MAX_ALIGN == 8; run after finish() so the region is still erased.
pub fn boot_set_pending(permanent: bool) -> bool {
    let ok = nor_with(|nor| {
        nor.write(upg::magic_off(), &upg::BOOT_MAGIC)
            .and_then(|_| {
                if permanent {
                    nor.write(upg::image_ok_off(), &upg::trailer_flag(upg::BOOT_FLAG_SET))
                } else {
                    Ok(())
                }
            })
            .and_then(|_| {
                let ty = if permanent {
                    upg::BOOT_SWAP_TYPE_PERM
                } else {
                    upg::BOOT_SWAP_TYPE_TEST
                };
                nor.write(upg::swap_info_off(), &upg::trailer_flag(ty))
            })
    });
    matches!(ok, Some(Ok(())))
}
