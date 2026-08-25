//! Firmware-upgrade session over the W25Q DFU partition (embassy-boot).
//!
//! Payload on every channel = raw app binary + 64-byte ed25519 signature of
//! SHA-512(binary) (the scheme embassy-boot/salty verify). start() gives
//! 0 ok / -2 keyhash mismatch / -3 busy / -1 other; it erases the whole DFU
//! partition. Writes are page-buffered. finish() flushes, CRC-checks the
//! readback and verifies the ed25519 signature (salty). boot_set_pending()
//! writes the embassy-boot SWAP magic into the state partition — the
//! bootloader swaps DFU with the active partition on the next reset, and
//! reverts automatically unless the new image reaches boot_confirm() (end
//! of main). NOR access goes through the storage task's driver mutex, so
//! littlefs traffic and upgrade traffic serialize on the same SPI bus.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use io_edge_hub_proto::fw_upg as upg;
use salty::{PublicKey, Sha512, Signature};

use crate::storage::nor_with;

const PAGE_SZ: usize = 256;
const READ_CHUNK: usize = 256;

struct FwSession {
    active: bool,
    failed: bool,
    total: u32,
    written: u32,
    page: [u8; PAGE_SZ],
    page_len: usize,
}

// ThreadModeRawMutex, NOT a critical section (same reasoning as storage::NOR):
// write()/start()/finish() run NOR page programs (0.4-3 ms), a whole-partition
// erase (~2 s) and a full-image readback inside the lock — masking interrupts
// that long overruns the 3-deep bxCAN RX FIFO at full bus speed and drops
// ~3 frames per 256 B page. Sound because every caller is an embassy task in
// thread mode, the closures hold no await (no interleaving), and no ISR
// touches this state.
static FW: Mutex<ThreadModeRawMutex, RefCell<FwSession>> = Mutex::new(RefCell::new(FwSession {
    active: false,
    failed: false,
    total: 0,
    written: 0,
    page: [0; PAGE_SZ],
    page_len: 0,
}));

/// Last finish() diagnostics (UDP debug cmd 0xFB): stage code, written/
/// computed crc/expected crc/sig-verified + first and last 16 readback bytes.
pub static FW_DBG: Mutex<ThreadModeRawMutex, RefCell<[u32; 16]>> =
    Mutex::new(RefCell::new([0; 16]));

fn dbg_store(vals: &[u32; 16]) {
    FW_DBG.lock(|d| *d.borrow_mut() = *vals);
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

fn erase_dfu() -> bool {
    matches!(nor_with(|nor| nor.erase(0, upg::DFU_SIZE)), Some(Ok(())))
}

pub fn start(total: u32, keyhash: Option<&[u8; upg::FW_KEYHASH_LEN]>) -> i32 {
    let mut rc = 0i32;
    FW.lock(|f| {
        let mut s = f.borrow_mut();
        if s.active {
            rc = -3;
            return;
        }
        if !upg::payload_ok(total) {
            rc = -1;
            return;
        }
        if let Some(kh) = keyhash {
            if kh != &upg::FW_KEYHASH {
                rc = -2;
                return;
            }
        }
        if !erase_dfu() {
            rc = -1;
            return;
        }
        s.active = true;
        s.failed = false;
        s.total = total;
        s.written = 0;
        s.page_len = 0;
    });
    rc
}

pub fn write(data: &[u8]) -> bool {
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
}

pub fn abort() {
    FW.lock(|f| {
        let mut s = f.borrow_mut();
        s.active = false;
        s.failed = false;
        s.page_len = 0;
        s.written = 0;
    });
}

/// Flush + CRC readback (when `crc` is Some) + ed25519 signature verify.
/// Resets the session; `received()`/`total()` read 0 afterwards.
pub fn finish(crc: Option<u16>) -> bool {
    let mut ok = false;
    FW.lock(|f| {
        let mut s = f.borrow_mut();
        if !s.active {
            return;
        }
        s.active = false;

        let tail = s.page_len as u32;
        if s.failed || !page_flush(&mut s) {
            dbg_store(&[
                1,
                s.written + tail,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ]);
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
            let n = upg::clamp_chunk(total - off, READ_CHUNK);
            let got = matches!(nor_with(|nor| nor.read(off, &mut buf[..n])), Some(Ok(())));
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
        // crc=None (CAN CONFIRM): skip the CRC compare — the ed25519
        // signature below is the integrity gate
        let crc_bad = match crc {
            Some(expect) => crc_v != expect,
            None => false,
        };
        if crc_bad {
            // full diagnostics: computed/expected crc + tail bytes
            let mut tail16 = [0u8; 16];
            let tail_off = total.saturating_sub(16);
            let _ = nor_with(|nor| nor.read(tail_off, &mut tail16));
            let mut d = [0u32; 16];
            d[0] = 5;
            d[1] = crc_v as u32;
            d[2] = crc.unwrap_or(0) as u32;
            for (i, w) in tail16.chunks(4).enumerate() {
                d[8 + i] = u32::from_le_bytes([w[0], w[1], w[2], w[3]]);
            }
            dbg_store(&d);
            return;
        }

        let sig_ok = verify_staged(total - upg::SIG_LEN as u32);
        if !sig_ok {
            dbg_store(&[6, total, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
            return;
        }
        dbg_store(&[
            7,
            total,
            crc_v as u32,
            crc.unwrap_or(0) as u32,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]);
        ok = true;
    });
    ok
}

/// ed25519-verify the staged payload: the last SIG_LEN bytes are the
/// signature (made by the host over SHA-512 of the image), everything
/// before is the image. Both are read straight off the NOR so the check
/// covers the programmed bytes, not the transfer buffers.
fn verify_staged(image_len: u32) -> bool {
    let mut sig = [0u8; upg::SIG_LEN];
    if !matches!(nor_with(|nor| nor.read(image_len, &mut sig)), Some(Ok(()))) {
        return false;
    }
    let mut h = Sha512::new();
    let mut buf = [0u8; READ_CHUNK];
    let mut off = 0u32;
    while off < image_len {
        let n = upg::clamp_chunk(image_len - off, READ_CHUNK);
        let got = matches!(nor_with(|nor| nor.read(off, &mut buf[..n])), Some(Ok(())));
        if !got {
            return false;
        }
        h.update(&buf[..n]);
        off += n as u32;
    }
    let digest = h.finalize();

    let Ok(pk) = PublicKey::try_from(&upg::FW_PUBKEY) else {
        return false;
    };
    let s = Signature::from(&sig);
    pk.verify(&digest, &s).is_ok()
}

// ---- embassy-boot state partition ----

fn state_magic() -> Option<u8> {
    let mut b = [0u8; 1];
    match nor_with(|nor| nor.read(upg::STATE_OFF, &mut b)) {
        Some(Ok(())) => Some(b[0]),
        _ => None,
    }
}

/// Byte-accurate port of embassy-boot BlockingFirmwareState::set_magic:
/// idempotent when the magic is already set; otherwise invalidate the
/// progress marker (byte 1) BEFORE the sector erase so a power cut between
/// the steps reads as "progress invalid" instead of resuming a stale swap.
fn state_set_magic(magic: u8) -> bool {
    if state_magic() == Some(magic) {
        return true;
    }
    let valid = nor_with(|nor| {
        let mut b = [0u8; 1];
        nor.read(upg::STATE_OFF + 1, &mut b).map(|_| b[0])
    });
    if matches!(valid, Some(Ok(0xFF))) {
        let b = [0x00u8; 1];
        if !matches!(
            nor_with(|nor| nor.write(upg::STATE_OFF + 1, &b)),
            Some(Ok(()))
        ) {
            return false;
        }
    }
    if !matches!(
        nor_with(|nor| nor.erase(upg::STATE_OFF, upg::STATE_SIZE)),
        Some(Ok(()))
    ) {
        return false;
    }
    let b = [magic];
    matches!(nor_with(|nor| nor.write(upg::STATE_OFF, &b)), Some(Ok(())))
}

/// Trigger the swap on the next reset. `permanent` is kept for channel
/// compatibility; with embassy-boot every swap is a trial boot — the new
/// image must run to the end of main (boot_confirm) or the next reset
/// reverts to the previous image.
pub fn boot_set_pending(_permanent: bool) -> bool {
    state_set_magic(upg::STATE_MAGIC_SWAP)
}

/// Confirm a freshly swapped-in image so the next reset does not revert.
/// Called at the end of main: reaching it means the box mounted its
/// storage, brought the network up and spawned every task.
pub fn boot_confirm() {
    match state_magic() {
        Some(upg::STATE_MAGIC_BOOT) | None => {}
        Some(_) => {
            if state_set_magic(upg::STATE_MAGIC_BOOT) {
                crate::log::inf("fw: boot confirmed");
            } else {
                crate::log::err("fw: boot confirm FAILED (will revert)");
            }
        }
    }
}

pub fn received() -> u32 {
    FW.lock(|f| {
        let s = f.borrow();
        s.written + s.page_len as u32
    })
}

pub fn total() -> u32 {
    FW.lock(|f| {
        let s = f.borrow();
        if s.active {
            s.total
        } else {
            0
        }
    })
}

pub fn active() -> bool {
    FW.lock(|f| f.borrow().active)
}
