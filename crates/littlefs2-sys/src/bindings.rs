// Hand-written FFI bindings for littlefs v2.11 (deps/littlefs of the C repo),
// transcribed from lfs.h. Symbol naming follows bindgen conventions so the
// littlefs2 crate's `ll::` references resolve unchanged.

use core::ffi::{c_char, c_int, c_void};

pub type lfs_size_t = u32;
pub type lfs_off_t = u32;
pub type lfs_ssize_t = i32;
pub type lfs_soff_t = i32;
pub type lfs_block_t = u32;

pub const LFS_VERSION: u32 = 0x0002_000b;
pub const LFS_VERSION_MAJOR: u32 = 2;
pub const LFS_VERSION_MINOR: u32 = 11;
pub const LFS_DISK_VERSION: u32 = 0x0002_0001;
pub const LFS_NAME_MAX: u32 = 255;
pub const LFS_FILE_MAX: u32 = 2147483647;
pub const LFS_ATTR_MAX: u32 = 1022;

pub type lfs_error = i32;
pub const lfs_error_LFS_ERR_OK: lfs_error = 0;
pub const lfs_error_LFS_ERR_IO: lfs_error = -5;
pub const lfs_error_LFS_ERR_CORRUPT: lfs_error = -84;
pub const lfs_error_LFS_ERR_NOENT: lfs_error = -2;
pub const lfs_error_LFS_ERR_EXIST: lfs_error = -17;
pub const lfs_error_LFS_ERR_NOTDIR: lfs_error = -20;
pub const lfs_error_LFS_ERR_ISDIR: lfs_error = -21;
pub const lfs_error_LFS_ERR_NOTEMPTY: lfs_error = -39;
pub const lfs_error_LFS_ERR_BADF: lfs_error = -9;
pub const lfs_error_LFS_ERR_FBIG: lfs_error = -27;
pub const lfs_error_LFS_ERR_INVAL: lfs_error = -22;
pub const lfs_error_LFS_ERR_NOSPC: lfs_error = -28;
pub const lfs_error_LFS_ERR_NOMEM: lfs_error = -12;
pub const lfs_error_LFS_ERR_NOATTR: lfs_error = -61;
pub const lfs_error_LFS_ERR_NAMETOOLONG: lfs_error = -36;

pub type lfs_type = u32;
pub const lfs_type_LFS_TYPE_REG: lfs_type = 0x001;
pub const lfs_type_LFS_TYPE_DIR: lfs_type = 0x002;
pub const lfs_type_LFS_TYPE_SPLICE: lfs_type = 0x400;
pub const lfs_type_LFS_TYPE_NAME: lfs_type = 0x000;
pub const lfs_type_LFS_TYPE_STRUCT: lfs_type = 0x200;
pub const lfs_type_LFS_TYPE_USERATTR: lfs_type = 0x300;
pub const lfs_type_LFS_TYPE_FROM: lfs_type = 0x100;
pub const lfs_type_LFS_TYPE_TAIL: lfs_type = 0x600;
pub const lfs_type_LFS_TYPE_GLOBALS: lfs_type = 0x700;
pub const lfs_type_LFS_TYPE_CRC: lfs_type = 0x500;
pub const lfs_type_LFS_TYPE_CREATE: lfs_type = 0x401;
pub const lfs_type_LFS_TYPE_DELETE: lfs_type = 0x4ff;
pub const lfs_type_LFS_TYPE_SUPERBLOCK: lfs_type = 0x0ff;
pub const lfs_type_LFS_TYPE_DIRSTRUCT: lfs_type = 0x200;
pub const lfs_type_LFS_TYPE_CTZSTRUCT: lfs_type = 0x202;
pub const lfs_type_LFS_TYPE_INLINESTRUCT: lfs_type = 0x201;
pub const lfs_type_LFS_TYPE_SOFTTAIL: lfs_type = 0x600;
pub const lfs_type_LFS_TYPE_HARDTAIL: lfs_type = 0x601;
pub const lfs_type_LFS_TYPE_MOVESTATE: lfs_type = 0x7ff;
pub const lfs_type_LFS_TYPE_CCRC: lfs_type = 0x500;
pub const lfs_type_LFS_TYPE_FCRC: lfs_type = 0x5ff;
pub const lfs_type_LFS_FROM_NOOP: lfs_type = 0x000;
pub const lfs_type_LFS_FROM_MOVE: lfs_type = 0x101;
pub const lfs_type_LFS_FROM_USERATTRS: lfs_type = 0x102;

pub type lfs_open_flags = i32;
pub const lfs_open_flags_LFS_O_RDONLY: lfs_open_flags = 1;
pub const lfs_open_flags_LFS_O_WRONLY: lfs_open_flags = 2;
pub const lfs_open_flags_LFS_O_RDWR: lfs_open_flags = 3;
pub const lfs_open_flags_LFS_O_CREAT: lfs_open_flags = 0x0100;
pub const lfs_open_flags_LFS_O_EXCL: lfs_open_flags = 0x0200;
pub const lfs_open_flags_LFS_O_TRUNC: lfs_open_flags = 0x0400;
pub const lfs_open_flags_LFS_O_APPEND: lfs_open_flags = 0x0800;
pub const lfs_open_flags_LFS_F_DIRTY: lfs_open_flags = 0x010000;
pub const lfs_open_flags_LFS_F_WRITING: lfs_open_flags = 0x020000;
pub const lfs_open_flags_LFS_F_READING: lfs_open_flags = 0x040000;
pub const lfs_open_flags_LFS_F_ERRED: lfs_open_flags = 0x080000;
pub const lfs_open_flags_LFS_F_INLINE: lfs_open_flags = 0x100000;

pub type lfs_whence_flags = i32;
pub const lfs_whence_flags_LFS_SEEK_SET: lfs_whence_flags = 0;
pub const lfs_whence_flags_LFS_SEEK_CUR: lfs_whence_flags = 1;
pub const lfs_whence_flags_LFS_SEEK_END: lfs_whence_flags = 2;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_config {
    pub context: *mut c_void,
    pub read: Option<
        unsafe extern "C" fn(
            c: *const lfs_config,
            block: lfs_block_t,
            off: lfs_off_t,
            buffer: *mut c_void,
            size: lfs_size_t,
        ) -> c_int,
    >,
    pub prog: Option<
        unsafe extern "C" fn(
            c: *const lfs_config,
            block: lfs_block_t,
            off: lfs_off_t,
            buffer: *const c_void,
            size: lfs_size_t,
        ) -> c_int,
    >,
    pub erase: Option<unsafe extern "C" fn(c: *const lfs_config, block: lfs_block_t) -> c_int>,
    pub sync: Option<unsafe extern "C" fn(c: *const lfs_config) -> c_int>,
    pub read_size: lfs_size_t,
    pub prog_size: lfs_size_t,
    pub block_size: lfs_size_t,
    pub block_count: lfs_size_t,
    pub block_cycles: i32,
    pub cache_size: lfs_size_t,
    pub lookahead_size: lfs_size_t,
    pub compact_thresh: lfs_size_t,
    pub read_buffer: *mut c_void,
    pub prog_buffer: *mut c_void,
    pub lookahead_buffer: *mut c_void,
    pub name_max: lfs_size_t,
    pub file_max: lfs_size_t,
    pub attr_max: lfs_size_t,
    pub metadata_max: lfs_size_t,
    pub inline_max: lfs_size_t,
    #[cfg(feature = "multiversion")]
    pub disk_version: u32,
}
impl Default for lfs_config {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_info {
    pub type_: u8,
    pub size: lfs_size_t,
    pub name: [c_char; 256],
}
impl Default for lfs_info {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_fsinfo {
    pub disk_version: u32,
    pub block_size: lfs_size_t,
    pub block_count: lfs_size_t,
    pub name_max: lfs_size_t,
    pub file_max: lfs_size_t,
    pub attr_max: lfs_size_t,
}
impl Default for lfs_fsinfo {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_attr {
    pub type_: u8,
    pub buffer: *mut c_void,
    pub size: lfs_size_t,
}
impl Default for lfs_attr {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_file_config {
    pub buffer: *mut c_void,
    pub attrs: *mut lfs_attr,
    pub attr_count: lfs_size_t,
}
impl Default for lfs_file_config {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_cache {
    pub block: lfs_block_t,
    pub off: lfs_off_t,
    pub size: lfs_size_t,
    pub buffer: *mut u8,
}
pub type lfs_cache_t = lfs_cache;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_mdir {
    pub pair: [lfs_block_t; 2],
    pub rev: u32,
    pub off: lfs_off_t,
    pub etag: u32,
    pub count: u16,
    pub erased: bool,
    pub split: bool,
    pub tail: [lfs_block_t; 2],
}
pub type lfs_mdir_t = lfs_mdir;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_dir {
    pub next: *mut lfs_dir,
    pub id: u16,
    pub type_: u8,
    pub m: lfs_mdir,
    pub pos: lfs_off_t,
    pub head: [lfs_block_t; 2],
}
pub type lfs_dir_t = lfs_dir;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_ctz {
    pub head: lfs_block_t,
    pub size: lfs_size_t,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_file {
    pub next: *mut lfs_file,
    pub id: u16,
    pub type_: u8,
    pub m: lfs_mdir,
    pub ctz: lfs_ctz,
    pub flags: u32,
    pub pos: lfs_off_t,
    pub block: lfs_block_t,
    pub off: lfs_off_t,
    pub cache: lfs_cache,
    pub cfg: *const lfs_file_config,
}
pub type lfs_file_t = lfs_file;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_superblock {
    pub version: u32,
    pub block_size: lfs_size_t,
    pub block_count: lfs_size_t,
    pub name_max: lfs_size_t,
    pub file_max: lfs_size_t,
    pub attr_max: lfs_size_t,
}
pub type lfs_superblock_t = lfs_superblock;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_gstate {
    pub tag: u32,
    pub pair: [lfs_block_t; 2],
}
pub type lfs_gstate_t = lfs_gstate;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_mlist {
    pub next: *mut lfs_mlist,
    pub id: u16,
    pub type_: u8,
    pub m: lfs_mdir,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs_lookahead {
    pub start: lfs_block_t,
    pub size: lfs_block_t,
    pub next: lfs_block_t,
    pub ckpoint: lfs_block_t,
    pub buffer: *mut u8,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct lfs {
    pub rcache: lfs_cache,
    pub pcache: lfs_cache,
    pub root: [lfs_block_t; 2],
    pub mlist: *mut lfs_mlist,
    pub seed: u32,
    pub gstate: lfs_gstate,
    pub gdisk: lfs_gstate,
    pub gdelta: lfs_gstate,
    pub lookahead: lfs_lookahead,
    pub cfg: *const lfs_config,
    pub block_count: lfs_size_t,
    pub name_max: lfs_size_t,
    pub file_max: lfs_size_t,
    pub attr_max: lfs_size_t,
    pub inline_max: lfs_size_t,
}
pub type lfs_t = lfs;

extern "C" {
    pub fn lfs_format(lfs: *mut lfs_t, config: *const lfs_config) -> c_int;
    pub fn lfs_mount(lfs: *mut lfs_t, config: *const lfs_config) -> c_int;
    pub fn lfs_unmount(lfs: *mut lfs_t) -> c_int;

    pub fn lfs_remove(lfs: *mut lfs_t, path: *const c_char) -> c_int;
    pub fn lfs_rename(lfs: *mut lfs_t, oldpath: *const c_char, newpath: *const c_char) -> c_int;
    pub fn lfs_stat(lfs: *mut lfs_t, path: *const c_char, info: *mut lfs_info) -> c_int;

    pub fn lfs_getattr(
        lfs: *mut lfs_t,
        path: *const c_char,
        type_: u8,
        buffer: *mut c_void,
        size: lfs_size_t,
    ) -> lfs_ssize_t;
    pub fn lfs_setattr(
        lfs: *mut lfs_t,
        path: *const c_char,
        type_: u8,
        buffer: *const c_void,
        size: lfs_size_t,
    ) -> c_int;
    pub fn lfs_removeattr(lfs: *mut lfs_t, path: *const c_char, type_: u8) -> c_int;

    pub fn lfs_file_opencfg(
        lfs: *mut lfs_t,
        file: *mut lfs_file_t,
        path: *const c_char,
        flags: c_int,
        config: *const lfs_file_config,
    ) -> c_int;
    pub fn lfs_file_close(lfs: *mut lfs_t, file: *mut lfs_file_t) -> c_int;
    pub fn lfs_file_sync(lfs: *mut lfs_t, file: *mut lfs_file_t) -> c_int;
    pub fn lfs_file_read(
        lfs: *mut lfs_t,
        file: *mut lfs_file_t,
        buffer: *mut c_void,
        size: lfs_size_t,
    ) -> lfs_ssize_t;
    pub fn lfs_file_write(
        lfs: *mut lfs_t,
        file: *mut lfs_file_t,
        buffer: *const c_void,
        size: lfs_size_t,
    ) -> lfs_ssize_t;
    pub fn lfs_file_seek(
        lfs: *mut lfs_t,
        file: *mut lfs_file_t,
        off: lfs_soff_t,
        whence: c_int,
    ) -> lfs_soff_t;
    pub fn lfs_file_tell(lfs: *mut lfs_t, file: *mut lfs_file_t) -> lfs_soff_t;
    pub fn lfs_file_rewind(lfs: *mut lfs_t, file: *mut lfs_file_t) -> c_int;
    pub fn lfs_file_size(lfs: *mut lfs_t, file: *mut lfs_file_t) -> lfs_soff_t;
    pub fn lfs_file_truncate(lfs: *mut lfs_t, file: *mut lfs_file_t, size: lfs_off_t) -> c_int;

    pub fn lfs_mkdir(lfs: *mut lfs_t, path: *const c_char) -> c_int;
    pub fn lfs_dir_open(lfs: *mut lfs_t, dir: *mut lfs_dir_t, path: *const c_char) -> c_int;
    pub fn lfs_dir_close(lfs: *mut lfs_t, dir: *mut lfs_dir_t) -> c_int;
    pub fn lfs_dir_read(lfs: *mut lfs_t, dir: *mut lfs_dir_t, info: *mut lfs_info) -> c_int;
    pub fn lfs_dir_seek(lfs: *mut lfs_t, dir: *mut lfs_dir_t, off: lfs_off_t) -> c_int;
    pub fn lfs_dir_tell(lfs: *mut lfs_t, dir: *mut lfs_dir_t) -> lfs_soff_t;
    pub fn lfs_dir_rewind(lfs: *mut lfs_t, dir: *mut lfs_dir_t) -> c_int;

    pub fn lfs_fs_stat(lfs: *mut lfs_t, fsinfo: *mut lfs_fsinfo) -> c_int;
    pub fn lfs_fs_size(lfs: *mut lfs_t) -> lfs_ssize_t;
    pub fn lfs_fs_traverse(
        lfs: *mut lfs_t,
        cb: Option<unsafe extern "C" fn(data: *mut c_void, block: lfs_block_t) -> c_int>,
        data: *mut c_void,
    ) -> c_int;
    pub fn lfs_fs_mkconsistent(lfs: *mut lfs_t) -> c_int;
    pub fn lfs_fs_gc(lfs: *mut lfs_t) -> c_int;
    pub fn lfs_fs_grow(lfs: *mut lfs_t, block_count: lfs_size_t) -> c_int;
}
