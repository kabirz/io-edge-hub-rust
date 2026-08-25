//! io-edge-hub bootloader (embassy-boot 0.7 core).
//!
//! Partition map (see proto::fw_upg):
//! - active: internal flash 0x08020000, 384 KiB (three 128 KiB swap pages)
//! - DFU (staging): W25Q128 0x0, 512 KiB
//! - state: W25Q128 0x80000, 4 KiB
//!
//! On boot: read the state partition; when the application marked an
//! update, swap active<->DFU page by page (power-fail safe: progress lives
//! in the state partition), then jump to the active image. The signature is
//! NOT verified here — the application verifies (ed25519/salty) before
//! marking, so the bootloader carries no crypto.
//!
//! A missing NOR (JEDEC mismatch) is not fatal: boot the active image
//! directly. A prepare error after the NOR was found logs and still boots —
//! equivalent to a power cut mid-swap, which the next boot resumes.

#![no_std]
#![no_main]

mod console;
mod w25q;

include!(concat!(env!("OUT_DIR"), "/boot_version.rs"));

use core::cell::RefCell;

use embassy_boot::{AlignedBuffer, BootLoader, BootLoaderConfig, State};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::rcc::{
    self, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config as BoardConfig;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use io_edge_hub_proto::fw_upg as upg;

use crate::w25q::{W25q, W25qPins};

const RCC_APB2RSTR: *mut u32 = 0x4002_3824 as *mut u32;

/// Offset of the active partition from the internal flash base.
const ACTIVE_OFF: u32 = 0x2_0000;

fn board_config() -> BoardConfig {
    let mut cfg = BoardConfig::default();
    // HSE 13MHz /13 *336 /2 = 168MHz SYSCLK, APB1 42MHz, APB2 84MHz
    // (identical to the application so the console BRR matches)
    cfg.rcc = rcc::Config::new();
    cfg.rcc.hse = Some(Hse {
        freq: Hertz(13_000_000),
        mode: HseMode::Oscillator,
    });
    cfg.rcc.pll_src = PllSource::HSE;
    cfg.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV13,
        mul: PllMul::MUL336,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    cfg.rcc.sys = Sysclk::PLL1_P;
    cfg.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
    cfg.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
    cfg.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    cfg.rcc.ls = rcc::LsConfig::default_lse();
    cfg
}

/// embassy's internal-flash driver plus an inline IWDG feed on every
/// program/erase: sector erases run 1-2 s and the watchdog the application
/// left running must not reset the box mid-swap.
struct InternalFlash(Flash<'static, Blocking>);

impl embedded_storage::nor_flash::ErrorType for InternalFlash {
    type Error = embassy_stm32::flash::Error;
}

impl ReadNorFlash for InternalFlash {
    const READ_SIZE: usize = <Flash<'static, Blocking> as ReadNorFlash>::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl NorFlash for InternalFlash {
    const WRITE_SIZE: usize = <Flash<'static, Blocking> as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <Flash<'static, Blocking> as NorFlash>::ERASE_SIZE;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let r = self.0.blocking_write(offset, bytes);
        w25q::iwdg_feed();
        r
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let r = self.0.blocking_erase(from, to);
        w25q::iwdg_feed();
        r
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = embassy_stm32::init(board_config());
    console::init();
    console::line(BOOT_BANNER);

    let nor = W25q::new(W25qPins {
        spi1: dp.SPI1,
        sck: dp.PA5,
        miso: dp.PA6,
        mosi: dp.PA7,
        cs: dp.PA4,
    });

    if nor.is_some() {
        console::line("nor: w25q128 ok");
    } else {
        console::line("nor: absent -> booting active without swap");
    }

    if let Some(nor) = nor {
        static INT_CELL: static_cell::StaticCell<Mutex<NoopRawMutex, RefCell<InternalFlash>>> =
            static_cell::StaticCell::new();
        static NOR_CELL: static_cell::StaticCell<Mutex<NoopRawMutex, RefCell<W25q>>> =
            static_cell::StaticCell::new();
        let int = INT_CELL.init(Mutex::new(RefCell::new(InternalFlash(
            Flash::new_blocking(dp.FLASH),
        ))));
        let ext = NOR_CELL.init(Mutex::new(RefCell::new(nor)));

        let active = BlockingPartition::new(int, ACTIVE_OFF, upg::APP_MAX_SIZE);
        let dfu = BlockingPartition::new(ext, 0, upg::DFU_SIZE);
        let state = BlockingPartition::new(ext, upg::STATE_OFF, upg::STATE_SIZE);
        let mut boot = BootLoader::new(BootLoaderConfig { active, dfu, state });

        let mut buf = AlignedBuffer([0u8; 4096]);
        match boot.prepare_boot(buf.as_mut()) {
            Ok(State::Swap) => console::line("boot: swap done (confirm in app or revert)"),
            Ok(State::Revert) => console::line("boot: reverted to previous image"),
            Ok(State::DfuDetach) => console::line("boot: dfu-detach (unused)"),
            Ok(State::Boot) => {}
            Err(_) => console::line("boot: prepare error -> booting anyway"),
        }
    }

    console::line("boot: jumping to app");
    w25q::iwdg_feed();
    unsafe { jump() }
}

/// Reset the peripherals this stage used and hand over to the app: VTOR,
/// MSP and reset vector from the active partition.
unsafe fn jump() -> ! {
    // flush the console TX shift register, then disable USART1
    let mut spins = 0u32;
    while (0x4001_1000 as *mut u32).read_volatile() & (1 << 6) == 0 {
        spins += 1;
        if spins > 2_000_000 {
            break;
        }
    }
    (0x4001_100C as *mut u32).write_volatile(0); // CR1 = 0

    // APB2RSTR pulse: USART1 (bit4) + SPI1 (bit12)
    let rstr = RCC_APB2RSTR;
    rstr.write_volatile(rstr.read_volatile() | (1 << 4) | (1 << 12));
    rstr.write_volatile(rstr.read_volatile() & !((1 << 4) | (1 << 12)));

    cortex_m::interrupt::disable();
    // drop anything pending from this stage (no IRQs are enabled here, be
    // defensive anyway)
    for i in 0..3usize {
        (0xE000_E180 as *mut u32).add(i).write_volatile(0xFFFF_FFFF); // ICER
        (0xE000_E280 as *mut u32).add(i).write_volatile(0xFFFF_FFFF); // ICPR
    }
    let p = cortex_m::Peripherals::steal();
    p.SCB.vtor.write(upg::APP_FLASH_START);
    cortex_m::asm::bootload(upg::APP_FLASH_START as *const u32)
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    console::line("boot: PANIC");
    loop {
        w25q::iwdg_feed(); // let the watchdog reset us out of the panic
        cortex_m::asm::delay(168_000_000);
    }
}
