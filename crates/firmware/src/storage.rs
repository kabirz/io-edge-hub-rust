//! Storage task: W25Q NOR owner — config store A/B + littlefs history
//! recorder (ports of config_store.c usage, lfs_port.c and history_file.c).
//!
//! Runs in a dedicated interrupt executor: NOR busy-polls (with inline IWDG
//! feeds) preempt the async thread executor only for the duration of a single
//! flash operation, so network/Modbus keep running. Producers talk to this
//! task through [QUEUE] (history.c's queue semantics: full queue drops).

use core::cell::{RefCell, UnsafeCell};
use core::mem::MaybeUninit;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use generic_array::typenum::consts::{U1024, U4};
use littlefs2::driver::Storage;
use littlefs2::fs::{Allocation, File, FileAllocation, Filesystem, OpenOptions};
use littlefs2::io as lfs_io;

use io_edge_hub_proto::config_store::{
    self as cs, IoCfg, CFG_SLOT_A, CFG_SLOT_B, CFG_SLOT_SIZE,
};
use io_edge_hub_proto::history::{make_hist_name, HisData};
use io_edge_hub_proto::regmap::{
    HOLDING_AI_ENABLE_IDX, HOLDING_AI_SAMPLE_MS_IDX, HOLDING_CAN_BAUDRATE_IDX,
    HOLDING_CAN_ID_IDX, HOLDING_DI_ENABLE_IDX, HOLDING_DI_SAMPLE_MS_IDX,
    HOLDING_HISTORY_ENABLE_IDX, HOLDING_IP_OCTET1_IDX, HOLDING_IP_OCTET2_IDX,
    HOLDING_IP_OCTET3_IDX, HOLDING_IP_OCTET4_IDX, HOLDING_RS485_BAUDRATE_IDX,
    HOLDING_SLAVE_ID_IDX, RegMap,
};

use crate::appstate::REGS;
use crate::w25q::W25q;

/// littlefs partition geometry (lfs_port.c PORT_* values, on-disk compat).
const LFS_OFFSET: u32 = 0x000F_0000;
const LFS_BLOCK_COUNT: usize = (0x0100_0000 - LFS_OFFSET as usize) / 4096;
const BLOCK_CYCLES: isize = 512;

const HIST_MAX_FILES: usize = 10;
const HIST_FILE_MAX: u32 = 1024 * 1024;

pub type StorageQueue = Channel<CriticalSectionRawMutex, StorageCmd, 8>;
pub static QUEUE: StorageQueue = Channel::new();

pub enum StorageCmd {
    Write(HisData),
    /// Close the current file but keep its name: next write continues it
    /// (disable -> enable semantics of history_file.c).
    CloseKeepName,
    Sync,
    CfgSave,
    CfgEraseAll,
    /// Web RPC: refresh FS_SNAP (root listing + usage) from littlefs.
    SnapReq,
    /// Web RPC: delete one data_*.raw by name.
    Del([u8; 24]),
    /// Web RPC: open a file for chunked download (FILE_DL <- size).
    FileOpen([u8; 24]),
    /// Web RPC: fill FILE_DL.chunk with the next block (512 B).
    FileChunk,
}

/// Root-listing + usage snapshot refreshed on SnapReq (history_web_list_json
/// + history_web_usage). 10 entries x (20 B NUL-padded name + 4 B BE size).
pub struct FsSnap {
    pub entries: [[u8; 24]; 10],
    pub count: usize,
    pub free: u32,
    pub total: u32,
    pub gen: u32,
}

pub static FS_SNAP: Mutex<CriticalSectionRawMutex, RefCell<FsSnap>> =
    Mutex::new(RefCell::new(FsSnap {
        entries: [[0; 24]; 10],
        count: 0,
        free: 0,
        total: 0x00F1_0000, // littlefs partition size
        gen: 0,
    }));

/// Persistent open history file (C's his_fp): kept open across records so a
/// sampling append is one buffered write instead of open+write+close.
/// Sound as !Send: single-core, every access under a critical section.
struct OpenFile {
    file: Option<File<'static, 'static, LfsNor>>,
}
unsafe impl Send for OpenFile {}

static OPEN_FILE: Mutex<CriticalSectionRawMutex, RefCell<Option<OpenFile>>> =
    Mutex::new(RefCell::new(None));

/// File-cache allocation with a stable address (littlefs embeds the buffer
/// pointer into the open lfs_file_t: the holder must never move while a
/// file is open, so the allocation lives here, outside OPEN_FILE).
struct AllocCell(UnsafeCell<MaybeUninit<FileAllocation<LfsNor>>>);
unsafe impl Sync for AllocCell {}
static FILE_ALLOC: AllocCell = AllocCell(UnsafeCell::new(MaybeUninit::zeroed()));
static DL_ALLOC: AllocCell = AllocCell(UnsafeCell::new(MaybeUninit::zeroed()));

/// Init once from the storage task; returns the stable &'static mut.
unsafe fn alloc_get(cell: &AllocCell) -> &'static mut FileAllocation<LfsNor> {
    // the cell is zeroed until first use; FileAllocation::new is not const
    let p = cell.0.get();
    if (*p).assume_init_ref() as *const _ as usize == 0 {
        (*p).write(FileAllocation::new());
    }
    (*p).assume_init_mut()
}

/// Persistent download handle (httpd chunked reads continue sequentially —
/// no reopen+seek per chunk, which made big downloads O(n^2)).
struct DlFile(Option<File<'static, 'static, LfsNor>>);
unsafe impl Send for DlFile {}
static DOWNLOAD_FILE: Mutex<CriticalSectionRawMutex, RefCell<DlFile>> =
    Mutex::new(RefCell::new(DlFile(None)));

/// Chunked-download state machine shared between httpd and the storage task.
pub struct FileDl {
    pub name: [u8; 24],
    pub size: u32,
    pub sent: u32,
    pub chunk: [u8; 512],
    pub chunk_len: usize,
    pub open: bool,
    pub eof: bool,
    pub err: bool,
}

pub static FILE_DL: Mutex<CriticalSectionRawMutex, RefCell<FileDl>> = Mutex::new(RefCell::new(
    FileDl {
        name: [0; 24],
        size: 0,
        sent: 0,
        chunk: [0; 512],
        chunk_len: 0,
        open: false,
        eof: false,
        err: false,
    },
));

/// Generation counter for the web RPCs: bumped once per processed command.
/// The httpd side snapshots it before sending and polls until it advances —
/// no stale-signal races (a Signal latches old values and wakes immediately).
pub static RPC_SEQ: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub static NOR: Mutex<CriticalSectionRawMutex, RefCell<Option<W25q>>> =
    Mutex::new(RefCell::new(None));

/// Active config slot/generation (config_store.c statics).
pub static CFG: Mutex<CriticalSectionRawMutex, RefCell<(Option<u32>, u32)>> =
    Mutex::new(RefCell::new((None, 0)));

fn nor_with<R>(f: impl FnOnce(&mut W25q) -> R) -> Option<R> {
    critical_section::with(|_cs| NOR.lock(|r| r.borrow_mut().as_mut().map(f)))
}

/// littlefs Storage over the shared NOR, offset by LFS_OFFSET. Writes split
/// at 256 B pages like lf_prog.
struct LfsNor;

impl Storage for LfsNor {
    const READ_SIZE: usize = 16;
    const WRITE_SIZE: usize = 16;
    const BLOCK_SIZE: usize = 4096;
    const BLOCK_COUNT: usize = LFS_BLOCK_COUNT;
    const BLOCK_CYCLES: isize = BLOCK_CYCLES;
    type CACHE_SIZE = U1024;
    type LOOKAHEAD_SIZE = U4;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> littlefs2::io::Result<usize> {
        nor_with(|w| w.read(LFS_OFFSET + off as u32, buf))
            .unwrap_or(Err(()))
            .map(|_| buf.len())
            .map_err(|_| littlefs2::io::Error::IO)
    }

    fn write(&mut self, off: usize, data: &[u8]) -> littlefs2::io::Result<usize> {
        let mut addr = LFS_OFFSET + off as u32;
        let mut p = 0usize;
        while p < data.len() {
            let mut chunk = 256 - (addr % 256) as usize;
            chunk = chunk.min(data.len() - p);
            nor_with(|w| w.write(addr, &data[p..p + chunk]))
                .unwrap_or(Err(()))
                .map_err(|_| littlefs2::io::Error::IO)?;
            addr += chunk as u32;
            p += chunk;
        }
        Ok(data.len())
    }

    fn erase(&mut self, off: usize, len: usize) -> littlefs2::io::Result<usize> {
        nor_with(|w| w.erase(LFS_OFFSET + off as u32, len as u32))
            .unwrap_or(Err(()))
            .map(|_| len)
            .map_err(|_| littlefs2::io::Error::IO)
    }
}

/// Boot-time config load (config_store_init): read both slots, pick the
/// winner, apply it to REGS. Blocking but only 2x40 B reads.
pub fn boot_config_load() {
    let mut ra = [0u8; cs::CFG_REC_LEN];
    let mut rb = [0u8; cs::CFG_REC_LEN];
    let a = match nor_with(|w| w.read(cs::CFG_SLOT_A, &mut ra)) {
        Some(Ok(())) => cs::decode_record(&ra),
        _ => None,
    };
    let b = match nor_with(|w| w.read(cs::CFG_SLOT_B, &mut rb)) {
        Some(Ok(())) => cs::decode_record(&rb),
        _ => None,
    };
    let (cfg, slot, gen) = match (a, b) {
        (Some((ca, ga)), Some((cb, gb))) => {
            if ga >= gb {
                (ca, Some(CFG_SLOT_A), ga)
            } else {
                (cb, Some(CFG_SLOT_B), gb)
            }
        }
        (Some((c, g)), None) => (c, Some(CFG_SLOT_A), g),
        (None, Some((c, g))) => (c, Some(CFG_SLOT_B), g),
        (None, None) => (IoCfg::defaults(), None, 0),
    };
    critical_section::with(|_cs| CFG.lock(|c| *c.borrow_mut() = (slot, gen)));
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            apply_cfg_to_regs(&mut r.borrow_mut(), &cfg);
        })
    });
    crate::log::inf("cfg: loaded from NOR");
}

pub fn apply_cfg_to_regs(r: &mut RegMap, c: &IoCfg) {
    let vals = [
        (HOLDING_DI_ENABLE_IDX, c.di_en),
        (HOLDING_AI_ENABLE_IDX, c.ai_en),
        (HOLDING_DI_SAMPLE_MS_IDX, c.di_si),
        (HOLDING_AI_SAMPLE_MS_IDX, c.ai_si),
        (HOLDING_HISTORY_ENABLE_IDX, c.his),
        (HOLDING_CAN_ID_IDX, c.can_id),
        (HOLDING_CAN_BAUDRATE_IDX, c.can_bps),
        (HOLDING_RS485_BAUDRATE_IDX, c.rs485_bps),
        (HOLDING_SLAVE_ID_IDX, c.slave_id),
        (HOLDING_IP_OCTET1_IDX, c.ip[0]),
        (HOLDING_IP_OCTET2_IDX, c.ip[1]),
        (HOLDING_IP_OCTET3_IDX, c.ip[2]),
        (HOLDING_IP_OCTET4_IDX, c.ip[3]),
    ];
    for (idx, v) in vals {
        r.holding[idx] = v;
    }
}

fn regs_to_cfg(r: &RegMap) -> IoCfg {
    let g = |idx: usize| r.get_holding(idx as u16);
    IoCfg {
        di_en: g(HOLDING_DI_ENABLE_IDX),
        ai_en: g(HOLDING_AI_ENABLE_IDX),
        di_si: g(HOLDING_DI_SAMPLE_MS_IDX),
        ai_si: g(HOLDING_AI_SAMPLE_MS_IDX),
        his: g(HOLDING_HISTORY_ENABLE_IDX),
        can_id: g(HOLDING_CAN_ID_IDX),
        can_bps: g(HOLDING_CAN_BAUDRATE_IDX),
        rs485_bps: g(HOLDING_RS485_BAUDRATE_IDX),
        slave_id: g(HOLDING_SLAVE_ID_IDX),
        ip: [
            g(HOLDING_IP_OCTET1_IDX),
            g(HOLDING_IP_OCTET2_IDX),
            g(HOLDING_IP_OCTET3_IDX),
            g(HOLDING_IP_OCTET4_IDX),
        ],
    }
}

fn cfg_save() {
    let cfg = critical_section::with(|_cs| REGS.lock(|r| regs_to_cfg(&r.borrow())));
    let (slot, gen) = critical_section::with(|_cs| CFG.lock(|c| *c.borrow()));
    let rec = cs::encode_record(&cfg, gen + 1);
    let tgt = if slot == Some(CFG_SLOT_A) { CFG_SLOT_B } else { CFG_SLOT_A };
    let ok = nor_with(|w| w.erase(tgt, CFG_SLOT_SIZE).is_ok())
        .unwrap_or(false)
        && nor_with(|w| w.write(tgt, &rec[..cs::CFG_HDR_LEN])).map_or(false, |r| r.is_ok())
        && nor_with(|w| w.write(tgt + cs::CFG_HDR_LEN as u32, &rec[cs::CFG_HDR_LEN..36])).map_or(false, |r| r.is_ok())
        && nor_with(|w| w.write(tgt + cs::CFG_CRC_OFF as u32, &rec[36..40])).map_or(false, |r| r.is_ok());
    if ok {
        critical_section::with(|_cs| CFG.lock(|c| *c.borrow_mut() = (Some(tgt), gen + 1)));
        crate::log::inf("cfg: saved");
    } else {
        crate::log::err("cfg: save failed");
    }
}

// ==================== history file core (history_file.c) ====================

/// Latest history file name retained across disable/enable; empty = rescan.
struct HistState {
    name: [u8; 24],
    name_len: usize,
}

impl HistState {
    const fn new() -> Self {
        Self { name: [0; 24], name_len: 0 }
    }

    fn set(&mut self, n: &[u8]) {
        let l = n.len().min(23);
        self.name[..l].copy_from_slice(&n[..l]);
        self.name[l] = 0;
        self.name_len = l;
    }

    fn path(&self) -> Option<&littlefs2::path::Path> {
        if self.name_len == 0 {
            None
        } else {
            littlefs2::path::Path::from_bytes_with_nul(&self.name[..self.name_len + 1]).ok()
        }
    }
}

fn hist_write(fs: &mut Filesystem<'_, LfsNor>, st: &mut HistState, d: &HisData) {
    let mut rec = d.to_bytes();
    let rec = &mut rec[..d.rec_len()];

    // fast path: the open file stays open across records (C's his_fp) —
    // one buffered append + sync, no open/close metadata churn per record
    let mut rotated = false;
    critical_section::with(|_cs| {
        OPEN_FILE.lock(|o| {
            let mut g = o.borrow_mut();
            if let Some(holder) = g.as_mut() {
                if let Some(f) = holder.file.as_mut() {
                    if let Ok(len) = f.len() {
                        if (len as u32) < HIST_FILE_MAX {
                            if f.write(rec).is_ok() {
                                let _ = f.sync();
                            }
                            return;
                        }
                    }
                }
            }
            rotated = true; // no open file or full: rotate below
        });
    });
    if !rotated {
        return;
    }
    // close a full file (keeps the name for continuation semantics)
    critical_section::with(|_cs| {
        OPEN_FILE.lock(|o| {
            if let Some(holder) = o.borrow_mut().as_mut() {
                if let Some(f) = holder.file.take() {
                    unsafe {
                        let _ = f.close();
                    }
                }
            }
        })
    });

    // reopen path: try the retained name, then the newest data_*.raw
    if st.name_len == 0 {
        if let Some(latest) = find_latest(&mut *fs) {
            st.set(&latest);
        }
    }
    if st.name_len > 0 && open_append(&mut *fs, st, rec, false) {
        return;
    }

    // create a fresh file; empty file triggers retention cleanup
    let name = make_hist_name(crate::systime::now_epoch());
    st.set(&name);
    if open_append(&mut *fs, st, rec, true) {
        cleanup_old_files(&mut *fs);
    }
}

/// Open (or create) the current file, append one record, KEEP THE FILE OPEN
/// for the next record. `create` allows creating (rotate path).
fn open_append(
    fs: &mut Filesystem<'_, LfsNor>,
    st: &mut HistState,
    rec: &mut [u8],
    create: bool,
) -> bool {
    let path = match st.path() {
        Some(p) => p,
        None => return false,
    };
    if !create {
        if let Ok(md) = fs.metadata(path) {
            if md.len() as u32 >= HIST_FILE_MAX {
                return false;
            }
        } else {
            return false;
        }
    }
    // the returned File is stored in OPEN_FILE as File<'static>: extend the
    // borrow unsafely — the storage task outlives every call and all access
    // to the file happens from this task under OPEN_FILE's critical section
    let fs: &'static mut Filesystem<'static, LfsNor> = unsafe { core::mem::transmute(fs) };
    let mut opts = OpenOptions::new();
    opts.write(true).append(true);
    if create {
        opts.create(true);
    }
    let alloc = unsafe { alloc_get(&FILE_ALLOC) };
    match unsafe { opts.open(fs, alloc, path) } {
        Ok(mut file) => {
            let ok = file.write(rec).is_ok();
            if ok {
                let _ = file.sync();
            }
            critical_section::with(|_cs| {
                OPEN_FILE.lock(|o| *o.borrow_mut() = Some(OpenFile { file: Some(file) }))
            });
            ok
        }
        Err(_) => false,
    }
}

fn find_latest(fs: &mut Filesystem<'_, LfsNor>) -> Option<[u8; 24]> {
    let mut latest = [0u8; 24];
    let mut latest_len = 0usize;
    fs.read_dir_and_then(b"/\0".try_into().unwrap(), |dir| {
        for entry in dir.flatten() {
            let b = entry.file_name().as_str().as_bytes();
            let bl = b.len().min(23);
            if bl > 5 && &b[..5] == b"data_" {
                if latest_len == 0 || b[..bl] > latest[..latest_len] {
                    latest[..bl].copy_from_slice(&b[..bl]);
                    latest_len = bl;
                }
            }
        }
        Ok(())
    })
    .ok()?;
    if latest_len == 0 {
        None
    } else {
        Some(latest)
    }
}

fn cleanup_old_files(fs: &mut Filesystem<'_, LfsNor>) {
    let mut names: heapless::Vec<[u8; 24], 12> = heapless::Vec::new();
    fs.read_dir_and_then(b"/\0".try_into().unwrap(), |dir| {
        for entry in dir.flatten() {
            let b = entry.file_name().as_str().as_bytes();
            let l = b.len().min(23);
            if l > 5 && &b[..5] == b"data_" {
                let mut n = [0u8; 24];
                n[..l].copy_from_slice(&b[..l]);
                n[l] = 0;
                names.push(n).ok();
            }
        }
        Ok(())
    })
    .ok();
    while names.len() > HIST_MAX_FILES {
        // oldest = lexicographically smallest (name order = time order)
        let mut min_i = 0;
        for i in 1..names.len() {
            if names[i][..] < names[min_i][..] {
                min_i = i;
            }
        }
        let np = {
            let nlen = names[min_i].iter().position(|&b| b == 0).unwrap_or(24);
            if nlen >= 24 {
                continue;
            }
            littlefs2::path::Path::from_bytes_with_nul(&names[min_i][..nlen + 1]).ok()
        };
        if let Some(p) = np {
            fs.remove(p).ok();
            crate::log::inf("history: rotated out old file");
        }
        let last = names.len() - 1;
        names.swap(min_i, last);
        names.pop();
    }
}

// ==================== the task ====================

#[embassy_executor::task]
pub async fn storage_task() {
    static ALLOC: static_cell::StaticCell<Allocation<LfsNor>> = static_cell::StaticCell::new();
    static STORAGE: static_cell::StaticCell<LfsNor> = static_cell::StaticCell::new();
    let alloc = ALLOC.init(Allocation::new());
    let storage = STORAGE.init(LfsNor);

    let mut fs = match Filesystem::mount(alloc, storage) {
        Ok(fs) => fs,
        Err(_) => {
            crate::log::wrn("lfs: mount failed, formatting");
            if Filesystem::format(storage).is_err() {
                crate::log::err("lfs: format failed, history disabled");
            }
            match Filesystem::mount(alloc, storage) {
                Ok(fs) => {
                    crate::log::inf("lfs: formatted and mounted");
                    fs
                }
                Err(_) => {
                    // history stays offline; config ops still work
                    loop {
                        match QUEUE.receive().await {
                            StorageCmd::CfgSave => cfg_save(),
                            StorageCmd::CfgEraseAll => cfg_erase_all(),
                            _ => {}
                        }
                    }
                }
            }
        }
    };
    crate::log::inf("lfs: mounted");
    fs_snapshot(&mut fs); // populate usage for /api/info from boot
    unsafe { alloc_get(&FILE_ALLOC) }; // place the cache at its final address
    critical_section::with(|_cs| {
        OPEN_FILE.lock(|o| *o.borrow_mut() = Some(OpenFile { file: None }))
    });

    // the task never exits: lend the filesystem 'static so the persistent
    // history file can outlive individual write calls
    let fs: &'static mut Filesystem<'static, LfsNor> = unsafe { core::mem::transmute(&mut fs) };
    let mut st = HistState::new();
    loop {
        match QUEUE.receive().await {
            StorageCmd::Write(d) => {
                let enabled = critical_section::with(|_cs| {
                    REGS.lock(|r| r.borrow().get_holding(HOLDING_HISTORY_ENABLE_IDX as u16) != 0)
                });
                if enabled {
                    hist_write(&mut *fs, &mut st, &d);
                }
            }
            // per-op open/write/close already persists each record; the
            // disable/enable file-continuation semantic is carried by st's
            // retained name, so these are bookkeeping no-ops by design
            StorageCmd::CloseKeepName => {
                critical_section::with(|_cs| {
                    OPEN_FILE.lock(|o| {
                        if let Some(mut g) = o.borrow_mut().take() {
                            if let Some(f) = g.file.take() {
                                unsafe {
                                    let _ = f.close();
                                }
                            }
                        }
                        // put the (now file-less) holder back
                        *o.borrow_mut() = Some(OpenFile { file: None });
                    })
                });
            }
            StorageCmd::Sync => {}
            StorageCmd::CfgSave => cfg_save(),
            StorageCmd::CfgEraseAll => cfg_erase_all(),
            StorageCmd::SnapReq => {
                let _ = fs_snapshot(&mut *fs);
                RPC_SEQ.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            StorageCmd::Del(name) => {
                if let Some(p) = path_of(&name) {
                    fs.remove(p).ok();
                    crate::log::inf("history: deleted (web)");
                }
            }
            StorageCmd::FileOpen(name) => {
                let _ = file_open(&mut *fs, &name);
                RPC_SEQ.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            StorageCmd::FileChunk => {
                let _ = file_chunk(&mut *fs);
                RPC_SEQ.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}


/// Path bytes -> &Path: slice to the first NUL (names arrive as [u8; 24]
/// fixed buffers; from_bytes_with_nul demands NUL be the last byte).
fn path_of(name: &[u8; 24]) -> Option<&littlefs2::path::Path> {
    let len = name.iter().position(|&b| b == 0).unwrap_or(24);
    if len == 0 || len >= 24 {
        return None;
    }
    littlefs2::path::Path::from_bytes_with_nul(&name[..len + 1]).ok()
}

/// Refresh FS_SNAP from the mounted filesystem (history_web_list_json +
/// history_web_usage equivalents).
fn fs_snapshot(fs: &mut Filesystem<'_, LfsNor>) -> bool {
    let mut snap = FsSnap {
        entries: [[0; 24]; 10],
        count: 0,
        free: 0,
        total: 0x00F1_0000,
        gen: 0,
    };
    fs.read_dir_and_then(b"/\0".try_into().unwrap(), |dir| {
        for entry in dir.flatten() {
            if snap.count >= 10 {
                break;
            }
            let b = entry.file_name().as_str().as_bytes();
            let l = b.len().min(20);
            if l > 5 && &b[..5] == b"data_" {
                let size = entry.metadata().len() as u32;
                snap.entries[snap.count][..l].copy_from_slice(&b[..l]);
                snap.entries[snap.count][20] = (size >> 24) as u8;
                snap.entries[snap.count][21] = (size >> 16) as u8;
                snap.entries[snap.count][22] = (size >> 8) as u8;
                snap.entries[snap.count][23] = size as u8;
                snap.count += 1;
            }
        }
        Ok(())
    })
    .ok();
    // sort entries newest-first (name order = time order, like the C list)
    snap.entries[..snap.count].sort_unstable_by(|a, b| b[..20].cmp(&a[..20]));
    if let Ok(blocks) = fs.available_blocks() {
        snap.free = (blocks as u32) * 4096;
    }
    critical_section::with(|_cs| {
        FS_SNAP.lock(|s| {
            let mut g = s.borrow_mut();
            snap.gen = g.gen.wrapping_add(1);
            *g = snap;
        })
    });
    true
}

/// Open a data_*.raw for chunked download (history_web_open): opens the
/// file once; FileChunk reads continue sequentially from the handle.
fn file_open(fs: &mut Filesystem<'_, LfsNor>, name: &[u8; 24]) -> bool {
    // close any stale handle first
    critical_section::with(|_cs| {
        DOWNLOAD_FILE.lock(|d| {
            if let Some(f) = d.borrow_mut().0.take() {
                unsafe {
                    let _ = f.close();
                }
            }
        })
    });
    critical_section::with(|_cs| {
        FILE_DL.lock(|f| {
            let mut g = f.borrow_mut();
            g.open = false;
            g.eof = false;
            g.err = false;
            g.size = 0;
            g.sent = 0;
            g.chunk_len = 0;
        })
    });
    let path = match path_of(name) {
        Some(p) => p,
        None => return false,
    };
    let fs: &'static mut Filesystem<'static, LfsNor> = unsafe { core::mem::transmute(fs) };
    let mut opts = OpenOptions::new();
    opts.read(true);
    let alloc = unsafe { alloc_get(&DL_ALLOC) };
    match unsafe { opts.open(fs, alloc, path) } {
        Ok(file) => {
            let size = file.len().unwrap_or(0) as u32;
            critical_section::with(|_cs| {
                FILE_DL.lock(|f| {
                    let mut g = f.borrow_mut();
                    g.name = *name;
                    g.size = size;
                    g.open = true;
                })
            });
            critical_section::with(|_cs| {
                DOWNLOAD_FILE.lock(|d| d.borrow_mut().0 = Some(file))
            });
            true
        }
        Err(_) => false,
    }
}

/// Read the next 512 B block into FILE_DL.chunk (history_web_read).
fn file_chunk(_fs: &mut Filesystem<'_, LfsNor>) -> bool {
    let (sent, size, open) = critical_section::with(|_cs| {
        FILE_DL.lock(|f| {
            let g = f.borrow();
            (g.sent, g.size, g.open)
        })
    });
    if !open {
        return true;
    }
    if sent >= size {
        critical_section::with(|_cs| {
            FILE_DL.lock(|f| f.borrow_mut().eof = true)
        });
        // close the finished handle
        critical_section::with(|_cs| {
            DOWNLOAD_FILE.lock(|d| {
                if let Some(f) = d.borrow_mut().0.take() {
                    unsafe {
                        let _ = f.close();
                    }
                }
            })
        });
        return true;
    }
    let mut buf = [0u8; 512];
    let n = critical_section::with(|_cs| {
        DOWNLOAD_FILE.lock(|d| {
            let mut g = d.borrow_mut();
            match g.0.as_mut() {
                Some(file) => file.read(&mut buf).unwrap_or(0),
                None => {
                    g.0.take(); // no handle: error
                    0
                }
            }
        })
    });
    let n = if n == 0 { 0 } else { n };
    if n == 0 {
        critical_section::with(|_cs| {
            FILE_DL.lock(|f| {
                f.borrow_mut().err = true;
                f.borrow_mut().eof = true;
            })
        });
        critical_section::with(|_cs| {
            DOWNLOAD_FILE.lock(|d| {
                if let Some(f) = d.borrow_mut().0.take() {
                    unsafe {
                        let _ = f.close();
                    }
                }
            })
        });
        return false;
    }
    critical_section::with(|_cs| {
        FILE_DL.lock(|f| {
            let mut g = f.borrow_mut();
            g.chunk[..n].copy_from_slice(&buf[..n]);
            g.chunk_len = n;
            g.sent += n as u32;
            if g.sent >= g.size {
                g.eof = true;
            }
        })
    });
    // finished: close the handle now
    let done = critical_section::with(|_cs| {
        FILE_DL.lock(|f| {
            let g = f.borrow();
            g.eof
        })
    });
    if done {
        critical_section::with(|_cs| {
            DOWNLOAD_FILE.lock(|d| {
                if let Some(f) = d.borrow_mut().0.take() {
                    unsafe {
                        let _ = f.close();
                    }
                }
            })
        });
    }
    true
}

fn cfg_erase_all() {
    let ok = nor_with(|w| w.erase(CFG_SLOT_A, CFG_SLOT_SIZE)).map_or(false, |r| r.is_ok())
        && nor_with(|w| w.erase(CFG_SLOT_B, CFG_SLOT_SIZE)).map_or(false, |r| r.is_ok());
    if ok {
            critical_section::with(|_cs| CFG.lock(|c| *c.borrow_mut() = (None, 0)));
            let d = IoCfg::defaults();
            critical_section::with(|_cs| {
                REGS.lock(|r| apply_cfg_to_regs(&mut r.borrow_mut(), &d))
            });
            crate::log::inf("cfg: factory erase done");
    } else {
        crate::log::err("cfg: erase failed");
    }
}
