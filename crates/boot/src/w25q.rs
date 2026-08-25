//! Minimal W25Q128 SPI NOR driver for the bootloader.
//!
//! Blocking SPI1 polling on the reset clock (HSI 16 MHz → APB2 16 MHz, SPI
//! @ 4 MHz — init speed is irrelevant next to flash erase times). Every
//! busy-wait feeds the IWDG: the application may have unleashed the 30 s
//! watchdog right before rebooting into us, and a swap must survive it.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;

const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_WRITE_EN: u8 = 0x06;
const CMD_PAGE_PROG: u8 = 0x02;
const CMD_READ_DATA: u8 = 0x03;
const CMD_READ_SR1: u8 = 0x05;
const CMD_SECTOR_ER: u8 = 0x20;
const CMD_BLOCK_ER64: u8 = 0xD8;

const SR1_BUSY: u8 = 0x01;

const CHIP_SIZE: u32 = 0x0100_0000;
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4096;
const JEDEC_ID: u32 = 0xEF_4018;

// Datasheet max ×5-8 margin
const TMO_4K: u32 = 2000;
const TMO_64K: u32 = 16000;
const TMO_PROG: u32 = 50;

pub struct W25q {
    spi: Spi<'static, embassy_stm32::mode::Blocking, embassy_stm32::spi::mode::Master>,
    cs: Output<'static>,
}

pub struct W25qPins {
    pub spi1: embassy_stm32::Peri<'static, embassy_stm32::peripherals::SPI1>,
    pub sck: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA5>,
    pub miso: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA6>,
    pub mosi: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA7>,
    pub cs: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA4>,
}

fn iwdg_pet() {
    unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
}

impl W25q {
    /// Init SPI1 and verify the JEDEC ID. `None` = chip absent/unreadable.
    pub fn new(p: W25qPins) -> Option<Self> {
        // HSI 16 MHz on APB2 after reset; request 4 MHz (prescaler /4)
        let mut cfg = SpiConfig::default();
        cfg.frequency = Hertz(4_000_000);
        let spi = Spi::new_blocking(p.spi1, p.sck, p.mosi, p.miso, cfg);
        let cs = Output::new(p.cs, Level::High, Speed::VeryHigh);
        let mut w = Self { spi, cs };
        if w.jedec_id() == JEDEC_ID {
            Some(w)
        } else {
            None
        }
    }

    fn cmd_addr(&mut self, cmd: u8, addr: u32) -> Result<(), ()> {
        let b = [cmd, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
        self.spi.blocking_write(&b).map_err(|_| ())
    }

    fn jedec_id(&mut self) -> u32 {
        let cmd = [CMD_JEDEC_ID];
        let mut id = [0u8; 3];
        self.cs.set_level(Level::Low);
        let r = self
            .spi
            .blocking_write(&cmd)
            .and_then(|_| self.spi.blocking_read(&mut id));
        self.cs.set_level(Level::High);
        match r {
            Ok(()) => ((id[0] as u32) << 16) | ((id[1] as u32) << 8) | id[2] as u32,
            Err(_) => 0,
        }
    }

    fn read_sr1(&mut self) -> Result<u8, ()> {
        let cmd = [CMD_READ_SR1];
        let mut sr = [0u8; 1];
        self.cs.set_level(Level::Low);
        let r = self
            .spi
            .blocking_write(&cmd)
            .and_then(|_| self.spi.blocking_read(&mut sr));
        self.cs.set_level(Level::High);
        r.map(|_| sr[0]).map_err(|_| ())
    }

    /// Poll BUSY every ~1 ms, feeding the IWDG.
    fn wait_not_busy(&mut self, timeout_ms: u32) -> Result<(), ()> {
        for _ in 0..=timeout_ms {
            iwdg_pet();
            if self.read_sr1()? & SR1_BUSY == 0 {
                return Ok(());
            }
            cortex_m::asm::delay(16_000); // ~1 ms at 16 MHz reset clock
        }
        Err(())
    }

    pub fn read(&mut self, mut addr: u32, buf: &mut [u8]) -> Result<(), ()> {
        if addr >= CHIP_SIZE || CHIP_SIZE - addr < buf.len() as u32 {
            return Err(());
        }
        if buf.is_empty() {
            return Ok(());
        }
        let mut off = 0usize;
        while off < buf.len() {
            let chunk = (buf.len() - off).min(0xFFFF);
            self.cs.set_level(Level::Low);
            let r = self.cmd_addr(CMD_READ_DATA, addr).and_then(|_| {
                self.spi
                    .blocking_read(&mut buf[off..off + chunk])
                    .map_err(|_| ())
            });
            self.cs.set_level(Level::High);
            r?;
            addr += chunk as u32;
            off += chunk;
        }
        Ok(())
    }

    /// Page-program: len ≤ 256, never crossing a 256 B page.
    pub fn write(&mut self, addr: u32, buf: &[u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }
        if addr % PAGE_SIZE + buf.len() as u32 > PAGE_SIZE || addr + buf.len() as u32 > CHIP_SIZE {
            return Err(());
        }
        iwdg_pet();
        self.wren()?;
        self.cs.set_level(Level::Low);
        let r = self
            .cmd_addr(CMD_PAGE_PROG, addr)
            .and_then(|_| self.spi.blocking_write(buf).map_err(|_| ()));
        self.cs.set_level(Level::High);
        r?;
        self.wait_not_busy(TMO_PROG)
    }

    fn wren(&mut self) -> Result<(), ()> {
        let cmd = [CMD_WRITE_EN];
        self.cs.set_level(Level::Low);
        let r = self.spi.blocking_write(&cmd).map_err(|_| ());
        self.cs.set_level(Level::High);
        r
    }

    /// Erase a 4 KiB-aligned region, preferring 64 K blocks when aligned.
    pub fn erase(&mut self, mut addr: u32, mut len: u32) -> Result<(), ()> {
        if len == 0 || addr % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 || addr + len > CHIP_SIZE
        {
            return Err(());
        }
        while len > 0 {
            iwdg_pet();
            let (chunk, cmd, tmo) = if addr % 0x1_0000 == 0 && len >= 0x1_0000 {
                (0x1_0000, CMD_BLOCK_ER64, TMO_64K)
            } else {
                (SECTOR_SIZE, CMD_SECTOR_ER, TMO_4K)
            };
            self.erase_one(addr, cmd, tmo)?;
            addr += chunk;
            len -= chunk;
        }
        Ok(())
    }

    fn erase_one(&mut self, addr: u32, cmd: u8, timeout_ms: u32) -> Result<(), ()> {
        self.wren()?;
        self.cs.set_level(Level::Low);
        let r = self.cmd_addr(cmd, addr);
        self.cs.set_level(Level::High);
        r?;
        self.wait_not_busy(timeout_ms)
    }
}
