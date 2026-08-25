//! io-edge-hub bootloader: embassy-boot over internal flash (ACTIVE) and the
//! external W25Q128 (DFU + STATE).
//!
//! Boot flow: init on the reset clock (HSI 16 MHz, plenty for SPI NOR), run
//! embassy-boot's swap/revert state machine across the two flashes, then jump
//! to ACTIVE at 0x08020000 with interrupts masked — the app's own prologue
//! re-homes VTOR and clears leftover NVIC/EXTI state, mirroring the old
//! MCUboot handoff contract.
//!
//! Partition geometry lives in io_edge_hub_proto::fw_upg::partitions and is
//! shared verbatim with the firmware. Page size = max(ACTIVE::ERASE_SIZE =
//! 128K largest internal sector, DFU::ERASE_SIZE = 4K) = 128K; the ACTIVE
//! partition is exactly three 128K sectors so every page erase maps onto one
//! physical sector.
//!
//! IWDG note: once the application has unleashed the watchdog it keeps
//! running through a soft reset, so every long operation here feeds it
//! (the W25Q busy-poll writes KR directly; erase wrappers pet around HAL
//! calls). A full swap of a ~205 KB image stays well inside the 30 s window.

#![no_std]
#![no_main]

mod w25q;

use core::cell::RefCell;

use cortex_m_rt::entry;
use embassy_boot::{BootLoader, BootLoaderConfig};
use embassy_stm32::flash::Flash;
use embedded_storage::nor_flash::{NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};
use io_edge_hub_proto::fw_upg::partitions as P;

mod boot_layout {
    include!(concat!(env!("OUT_DIR"), "/boot_layout.rs"));
}
#[allow(unused)]
use boot_layout::LINK_SCRIPTS;

fn iwdg_pet() {
    // KR reload; harmless if the watchdog was never started
    unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
}

// ---- ACTIVE adapter: whole-chip HAL flash seen from the ACTIVE offset ----

/// Errors are all "other" — the driver reports no finer kinds.
#[derive(Debug)]
struct ActiveErr;
impl core::fmt::Display for ActiveErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("internal flash error")
    }
}
impl NorFlashError for ActiveErr {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

type HalFlash = Flash<'static, embassy_stm32::flash::Blocking>;

/// embassy-boot page for this layout: the largest internal sector.
const PAGE: u32 = P::PAGE_SIZE;

struct ActivePart {
    hal: HalFlash,
}

impl ActivePart {
    /// Erase covering sectors, feeding the IWDG between them: a 128 K
    /// sector erase is typ ~1 s / max ~5 s, three of them must not trip a
    /// running watchdog.
    fn erase_covering(&mut self, from: u32, to: u32) -> Result<(), ActiveErr> {
        let mut addr = P::ACTIVE_BASE + from;
        let end = P::ACTIVE_BASE + to;
        while addr < end {
            iwdg_pet();
            self.hal
                .blocking_erase(addr - 0x0800_0000, addr - 0x0800_0000 + PAGE)
                .map_err(|_| ActiveErr)?;
            addr += PAGE;
        }
        Ok(())
    }
}

impl ReadNorFlash for ActivePart {
    const READ_SIZE: usize = 1;
    fn read(&mut self, off: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.hal
            .blocking_read(P::ACTIVE_BASE - 0x0800_0000 + off, buf)
            .map_err(|_| ActiveErr)
    }
    fn capacity(&self) -> usize {
        P::ACTIVE_LEN as usize
    }
}

impl NorFlash for ActivePart {
    const WRITE_SIZE: usize = 4; // F4 word programming (matches the HAL)
    const ERASE_SIZE: usize = 128 * 1024;
    fn write(&mut self, off: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.hal
            .blocking_write(P::ACTIVE_BASE - 0x0800_0000 + off, data)
            .map_err(|_| ActiveErr)
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_covering(from, to)
    }
}

impl embedded_storage::nor_flash::ErrorType for ActivePart {
    type Error = ActiveErr;
}

// ---- external adapters: shared W25Q instance behind a RefCell ----

#[derive(Debug)]
struct ExtErr;
impl core::fmt::Display for ExtErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("w25q error")
    }
}
impl NorFlashError for ExtErr {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

struct ExtPart<'a> {
    drv: &'a RefCell<w25q::W25q>,
    base: u32,
    len: u32,
}

impl<'a> ExtPart<'a> {
    fn new(drv: &'a RefCell<w25q::W25q>, base: u32, len: u32) -> Self {
        Self { drv, base, len }
    }
    fn bounds_ok(&self, off: u32, l: usize) -> bool {
        off as usize + l <= self.len as usize
    }
}

impl ReadNorFlash for ExtPart<'_> {
    const READ_SIZE: usize = 1;
    fn read(&mut self, off: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        if !self.bounds_ok(off, buf.len()) {
            return Err(ExtErr);
        }
        self.drv
            .borrow_mut()
            .read(self.base + off, buf)
            .map_err(|_| ExtErr)
    }
    fn capacity(&self) -> usize {
        self.len as usize
    }
}

impl NorFlash for ExtPart<'_> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;
    fn write(&mut self, off: u32, data: &[u8]) -> Result<(), Self::Error> {
        if !self.bounds_ok(off, data.len()) {
            return Err(ExtErr);
        }
        // split at the W25Q's 256 B program-page boundaries (embassy-boot's
        // copy loop hands us page-sized chunks, but stay correct for any size)
        let mut off = off;
        let mut rest = data;
        while !rest.is_empty() {
            let chunk = (256 - ((self.base + off) % 256) as usize).min(rest.len());
            self.drv
                .borrow_mut()
                .write(self.base + off, &rest[..chunk])
                .map_err(|_| ExtErr)?;
            off += chunk as u32;
            rest = &rest[chunk..];
        }
        Ok(())
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to < from || !self.bounds_ok(from, (to - from) as usize) {
            return Err(ExtErr);
        }
        self.drv
            .borrow_mut()
            .erase(self.base + from, to - from)
            .map_err(|_| ExtErr)
    }
}

impl embedded_storage::nor_flash::ErrorType for ExtPart<'_> {
    type Error = ExtErr;
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Halt with the watchdog fed one last time: if the application had
    // unleashed the IWDG before rebooting into us, it resets out of here in
    // ≤30 s; otherwise stay halted until a debugger/power cycle.
    iwdg_pet();
    loop {
        cortex_m::asm::wfe();
    }
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    // Console UART init deliberately skipped: the bootloader is silent by
    // design (the app banner is the first log line after handoff).

    let mut ext = w25q::W25q::new(w25q::W25qPins {
        spi1: p.SPI1,
        sck: p.PA5,
        miso: p.PA6,
        mosi: p.PA7,
        cs: p.PA4,
    });

    if ext.is_some() {
        iwdg_pet();
        // RefCell soundness: single-threaded pre-jump code, borrows never
        // overlap (active part holds no reference to the external chip).
        let cell = RefCell::new(ext.take().unwrap());
        let active_flash = Flash::new_blocking(p.FLASH);
        let config = BootLoaderConfig {
            active: ActivePart { hal: active_flash },
            dfu: ExtPart::new(&cell, P::DFU_BASE, P::DFU_LEN),
            state: ExtPart::new(&cell, P::STATE_BASE, P::STATE_LEN),
        };
        let mut loader = BootLoader::new(config);
        // Copy loop steps by this buffer; it must divide the 128 K page
        // evenly (4096 does) and be ≥ STATE::WRITE_SIZE (= 1).
        let mut buf = embassy_boot::AlignedBuffer([0u8; 4096]);
        let _state = loader.prepare_boot(&mut buf.0);
        // State intentionally ignored: Boot/Swap both proceed to ACTIVE;
        // DfuDetach would need a USB DFU mode this product doesn't ship.
    } else {
        // External flash missing/unreadable: skip the swap machinery rather
        // than brick — boot whatever is in ACTIVE.
    }

    iwdg_pet();
    jump_active();
}

/// Hand off to the application with the same cleanup contract as the old
/// MCUboot loader: mask interrupts, kill enabled/pending NVIC lines, then
/// vector through the ACTIVE slot. The app re-inits everything else.
fn jump_active() -> ! {
    unsafe {
        cortex_m::interrupt::disable();
        for i in 0..3usize {
            core::ptr::write_volatile((0xE000_E180 + 4 * i) as *mut u32, 0xFFFF_FFFF); // ICER
            core::ptr::write_volatile((0xE000_E280 + 4 * i) as *mut u32, 0xFFFF_FFFF); // ICPR
        }
        cortex_m::asm::bootload(P::ACTIVE_BASE as *const u32);
    }
}
