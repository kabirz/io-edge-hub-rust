//! Firmware-upgrade session over the embassy-boot DFU slot (external W25Q).
//!
//! The transports (UDP / WS / CAN) drive one shared session:
//!
//! - [`start`] validates size/keyhash and erases the whole DFU slot (RPC;
//!   the storage task owns the chip);
//! - [`write`] is **synchronous** — it only buffers into 256 B pages and
//!   pushes them onto a pending-page ring — so the WS parser callback can
//!   feed binary frames without awaiting. Transports drain the ring with
//!   [`flush`]; [`finish`] drains whatever is left;
//! - [`finish`] verifies sizes, re-reads the whole image back from DFU
//!   (validating the programming result), compares the CRC16 when the
//!   transport provided one, and marks the image updated (swap on next
//!   reboot). Trial boot: the app confirms itself ~10 s after boot
//!   (heartbeat -> FwMarkBooted); if it never runs, the bootloader reverts.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use io_edge_hub_proto::fw_upg as upg;

use crate::storage::{rpc_seq, rpc_wait, StorageCmd, FW_READ, FW_RES, FW_STAGE, QUEUE};

const PAGE_SZ: usize = 256;
/// Pending-page ring depth (bytes of slack between transport and flash).
/// Steady-state programming (~1 ms per 256 B page) outruns any transport;
/// the ring only absorbs erase pauses. Overflow fails the transfer cleanly.
const RING_MAX: usize = 12;

static FW: Mutex<ThreadModeRawMutex, RefCell<Sess>> = Mutex::new(RefCell::new(Sess::new()));

struct Sess {
    active: bool,
    failed: bool,
    total: u32,
    /// Bytes accepted so far (= offset the next incoming byte lands at).
    received: u32,
    page: [u8; PAGE_SZ],
    page_len: usize,
}

impl Sess {
    const fn new() -> Self {
        Self {
            active: false,
            failed: false,
            total: 0,
            received: 0,
            page: [0; PAGE_SZ],
            page_len: 0,
        }
    }
}

/// Pages accepted but not yet programmed, drained by [flush].
static RING: Mutex<CriticalSectionRawMutex, RefCell<Ring>> =
    Mutex::new(RefCell::new(Ring::new()));

struct Ring {
    off: [u32; RING_MAX],
    len: [usize; RING_MAX],
    data: [[u8; PAGE_SZ]; RING_MAX],
    head: usize,
    tail: usize,
}

impl Ring {
    const fn new() -> Self {
        Self {
            off: [0; RING_MAX],
            len: [0; RING_MAX],
            data: [[0; PAGE_SZ]; RING_MAX],
            head: 0,
            tail: 0,
        }
    }
    fn push(&mut self, off: u32, data: &[u8]) -> bool {
        let next = (self.head + 1) % RING_MAX;
        if next == self.tail {
            return false;
        }
        self.off[self.head] = off;
        self.len[self.head] = data.len();
        self.data[self.head][..data.len()].copy_from_slice(data);
        self.head = next;
        true
    }
    /// Copy out the oldest entry and drop it from the ring.
    fn pop_into(&mut self, buf: &mut [u8; PAGE_SZ]) -> Option<(u32, usize)> {
        if self.tail == self.head {
            return None;
        }
        let i = self.tail;
        self.tail = (self.tail + 1) % RING_MAX;
        *buf = self.data[i];
        Some((self.off[i], self.len[i]))
    }
    fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
    }
}

/// Diagnostics for UDP debug cmd 0xFB:
/// [stage, received, total, computed crc, expected crc].
/// Stages: 2 size/failed, 4 readback io, 5 crc mismatch, 6 DFU erase,
/// 7 program, 8 mark-updated.
pub static FW_DBG: Mutex<ThreadModeRawMutex, RefCell<[u32; 16]>> =
    Mutex::new(RefCell::new([0; 16]));

fn dbg_store(vals: &[u32; 16]) {
    FW_DBG.lock(|d| *d.borrow_mut() = *vals);
}

fn fw_res_ok() -> bool {
    critical_section::with(|_cs| FW_RES.lock(|r| r.borrow().0))
}

/// Begin an upgrade session. Returns 0 ok / -2 keyhash mismatch / -3 busy /
/// -1 other (bad size, DFU erase failure).
pub async fn start(total: u32, keyhash: Option<&[u8; upg::FW_KEYHASH_LEN]>) -> i32 {
    let mut rc = 0i32;
    FW.lock(|f| {
        let s = f.borrow();
        if s.active {
            rc = -3;
        } else if total < 64 || total > upg::partitions::DFU_LEN {
            rc = -1;
        } else if let Some(kh) = keyhash {
            if kh != &upg::FW_KEYHASH {
                rc = -2;
            }
        }
    });
    if rc != 0 {
        return rc;
    }

    // Whole-DFU erase up front (~1-2 s): deterministic clean slate, matching
    // the MCUboot-era START semantics. The lazy per-sector erase inside
    // write_firmware never fires afterwards (sectors are pre-erased).
    let seq = rpc_seq();
    QUEUE.try_send(StorageCmd::FwBegin).ok();
    if !rpc_wait(seq).await || !fw_res_ok() {
        dbg_store(&[6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        return -1;
    }

    FW.lock(|f| {
        let mut s = f.borrow_mut();
        s.active = true;
        s.failed = false;
        s.total = total;
        s.received = 0;
        s.page_len = 0;
    });
    RING.lock(|r| r.borrow_mut().clear());
    0
}

/// Accept upgrade payload synchronously (no flash access here): buffers into
/// 256 B pages and queues them onto the pending ring. False = transfer must
/// fail (session inactive/failed, oversize, or ring overflow).
pub fn write(data: &[u8]) -> bool {
    FW.lock(|f| {
        let mut s = f.borrow_mut();
        if !s.active || s.failed {
            return false;
        }
        if s.received + s.page_len as u32 + data.len() as u32 > s.total {
            s.failed = true;
            return false;
        }
        let mut ok = true;
        let mut data = data;
        while !data.is_empty() && ok {
            let plen = s.page_len;
            let chunk = (PAGE_SZ - plen).min(data.len());
            let base_off = s.received;
            s.page[plen..plen + chunk].copy_from_slice(&data[..chunk]);
            s.page_len = plen + chunk;
            s.received += chunk as u32;
            data = &data[chunk..];
            if s.page_len == PAGE_SZ {
                ok = RING.lock(|r| r.borrow_mut().push(base_off, &s.page));
                s.page_len = 0;
                if !ok {
                    s.failed = true;
                }
            }
        }
        ok
    })
}

/// Drain the pending ring into the storage task. Returns false on any RPC /
/// programming failure (the session is marked failed; further calls are
/// harmless).
pub async fn flush() -> bool {
    loop {
        let mut buf = [0u8; PAGE_SZ];
        let item = RING.lock(|r| r.borrow_mut().pop_into(&mut buf));
        let (off, len) = match item {
            Some(v) => v,
            None => return true,
        };
        critical_section::with(|_cs| {
            FW_STAGE.lock(|s| s.borrow_mut()[..len].copy_from_slice(&buf[..len]));
        });
        let seq = rpc_seq();
        QUEUE.try_send(StorageCmd::FwProg { off, len: len as u16 }).ok();
        if !rpc_wait(seq).await || !fw_res_ok() {
            FW.lock(|f| f.borrow_mut().failed = true);
            dbg_store(&[7, off, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
            return false;
        }
    }
}

/// Flush pending pages, verify sizes, re-read the whole image from DFU and
/// (when `crc` is given) compare the CRC16-CCITT of the readback, then mark
/// the image for swap. Resets the session either way.
pub async fn finish(crc: Option<u16>) -> bool {
    if !flush().await {
        return false;
    }

    let (active, failed, total, received) = FW.lock(|f| {
        let s = f.borrow();
        (s.active, s.failed, s.total, s.received)
    });
    if !active {
        return false;
    }
    // one-shot: take the session down before verifying
    FW.lock(|f| {
        let mut s = f.borrow_mut();
        s.active = false;
        s.page_len = 0;
    });

    if failed || received != total {
        dbg_store(&[2, received, total, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        return false;
    }

    // Readback verify: hash the bytes AS STORED IN DFU, not the ones sent.
    let mut crc_v: u16 = 0;
    let mut verified = true;
    let mut off: u32 = 0;
    while off < total {
        let n = (total - off).min(PAGE_SZ as u32) as usize;
        let seq = rpc_seq();
        QUEUE.try_send(StorageCmd::FwRead { off }).ok();
        if !rpc_wait(seq).await || !fw_res_ok() {
            verified = false;
            break;
        }
        let chunk = critical_section::with(|_cs| FW_READ.lock(|r| *r.borrow()));
        crc_v = upg::crc16_ccitt(crc_v, &chunk[..n]);
        off += n as u32;
    }
    if !verified {
        dbg_store(&[4, received, total, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        return false;
    }

    if matches!(crc, Some(expect) if crc_v != expect) {
        dbg_store(&[
            5, received, total, crc_v as u32, crc.unwrap_or(0) as u32, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0,
        ]);
        return false;
    }

    let seq = rpc_seq();
    QUEUE.try_send(StorageCmd::FwMarkUpdated).ok();
    if !rpc_wait(seq).await || !fw_res_ok() {
        dbg_store(&[8, received, total, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        return false;
    }
    true
}

/// Abandon the session (START always runs this implicitly via the busy gate;
/// kept for explicit teardown paths).
pub fn abort() {
    FW.lock(|f| *f.borrow_mut() = Sess::new());
    RING.lock(|r| r.borrow_mut().clear());
}

/// Bytes accepted so far (includes buffered/ringed bytes not yet flashed) —
/// the offset the transport must send next.
pub fn received() -> u32 {
    FW.lock(|f| f.borrow().received)
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
