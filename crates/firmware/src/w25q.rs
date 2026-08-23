//! W25Q128 (16 MiB) SPI NOR driver, port of src/storage/w25qxx.c.
//!
//! Blocking SPI1 polling like the C firmware (no DMA, no interrupts). Every
//! operation: CS low -> command (+24-bit big-endian address) -> data -> CS
//! high. WREN before writes/erases; status-register busy polling with the
//! IWDG fed inline (long erases must not reset the box).

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;

const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_WRITE_EN: u8 = 0x06;
const CMD_PAGE_PROG: u8 = 0x02;
const CMD_READ_DATA: u8 = 0x03;
const CMD_READ_SR1: u8 = 0x05;
const CMD_SECTOR_ER: u8 = 0x20; // 4 KiB
const CMD_BLOCK_ER32: u8 = 0x52; // 32 KiB
const CMD_BLOCK_ER64: u8 = 0xD8; // 64 KiB

const SR1_BUSY: u8 = 0x01;

const CHIP_SIZE: u32 = 0x0100_0000; // 16 MiB
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4096;

const JEDEC_ID: u32 = 0xEF_4018; // Winbond + 128 Mbit

// Datasheet max 4K=400ms / 32K=1600 / 64K=2000; 5-8x margin like the C code
const TMO_4K: u32 = 2000;
const TMO_32K: u32 = 8000;
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

impl W25q {
    /// Init SPI1 @42 MHz and verify the JEDEC ID.
    pub fn new(p: W25qPins) -> Option<Self> {
        let mut cfg = SpiConfig::default();
        cfg.frequency = Hertz(42_000_000);
        let spi = Spi::new_blocking(p.spi1, p.sck, p.mosi, p.miso, cfg);
        let cs = Output::new(p.cs, Level::High, Speed::VeryHigh);
        let mut w = Self { spi, cs };
        let id = w.jedec_id();
        if id == JEDEC_ID {
            Some(w)
        } else {
            crate::log::err("w25q: bad JEDEC id");
            None
        }
    }

    fn cs_low(&mut self) {
        self.cs.set_level(Level::Low);
    }

    fn cs_high(&mut self) {
        self.cs.set_level(Level::High);
    }

    fn cmd_addr(&mut self, cmd: u8, addr: u32) -> Result<(), ()> {
        let b = [cmd, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
        self.spi.blocking_write(&b).map_err(|_| ())
    }

    fn jedec_id(&mut self) -> u32 {
        let cmd = [CMD_JEDEC_ID];
        let mut id = [0u8; 3];
        self.cs_low();
        let r = self
            .spi
            .blocking_write(&cmd)
            .and_then(|_| self.spi.blocking_read(&mut id));
        self.cs_high();
        match r {
            Ok(()) => ((id[0] as u32) << 16) | ((id[1] as u32) << 8) | id[2] as u32,
            Err(_) => 0,
        }
    }

    fn wren(&mut self) -> Result<(), ()> {
        let cmd = [CMD_WRITE_EN];
        self.cs_low();
        let r = self.spi.blocking_write(&cmd).map_err(|_| ());
        self.cs_high();
        r
    }

    fn read_sr1(&mut self) -> Result<u8, ()> {
        let cmd = [CMD_READ_SR1];
        let mut sr = [0u8; 1];
        self.cs_low();
        let r = self
            .spi
            .blocking_write(&cmd)
            .and_then(|_| self.spi.blocking_read(&mut sr));
        self.cs_high();
        r.map(|_| sr[0]).map_err(|_| ())
    }

    /// Poll the busy bit every ~1 ms, feeding the IWDG like w25qxx.c: a
    /// full-partition format takes minutes and must not trip the watchdog.
    fn wait_not_busy(&mut self, timeout_ms: u32) -> Result<(), ()> {
        for _ in 0..=timeout_ms {
            unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
            if self.read_sr1()? & SR1_BUSY == 0 {
                return Ok(());
            }
            cortex_m::asm::delay(168_000); // ~1 ms at 168 MHz
        }
        Err(())
    }

    pub fn read(&mut self, mut addr: u32, buf: &mut [u8]) -> Result<(), ()> {
        if addr >= CHIP_SIZE || CHIP_SIZE - addr < buf.len() as u32 {
            return Err(());
        }
        let mut off = 0usize;
        while off < buf.len() {
            let chunk = (buf.len() - off).min(0xFFFF);
            self.cs_low();
            let r = self
                .cmd_addr(CMD_READ_DATA, addr)
                .and_then(|_| self.spi.blocking_read(&mut buf[off..off + chunk]).map_err(|_| ()));
            self.cs_high();
            r?;
            addr += chunk as u32;
            off += chunk;
        }
        Ok(())
    }

    /// Page-program respecting write: len <= 256, never crossing a 256 B page.
    pub fn write(&mut self, addr: u32, buf: &[u8]) -> Result<(), ()> {
        if buf.is_empty() {
            return Ok(());
        }
        if addr % PAGE_SIZE + buf.len() as u32 > PAGE_SIZE || addr + buf.len() as u32 > CHIP_SIZE {
            return Err(());
        }
        self.wren()?;
        self.cs_low();
        let r = self
            .cmd_addr(CMD_PAGE_PROG, addr)
            .and_then(|_| self.spi.blocking_write(buf).map_err(|_| ()));
        self.cs_high();
        r?;
        self.wait_not_busy(TMO_PROG)
    }

    fn erase_one(&mut self, addr: u32, cmd: u8, timeout_ms: u32) -> Result<(), ()> {
        self.wren()?;
        self.cs_low();
        let r = self.cmd_addr(cmd, addr);
        self.cs_high();
        r?;
        self.wait_not_busy(timeout_ms)
    }

    /// Erase a 4 KiB-aligned region, preferring 64 K/32 K blocks when aligned
    /// (same chunking strategy as w25_erase).
    pub fn erase(&mut self, mut addr: u32, mut len: u32) -> Result<(), ()> {
        if len == 0 || addr % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 || addr + len > CHIP_SIZE {
            return Err(());
        }
        unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
        while len > 0 {
            let (chunk, cmd, tmo) = if addr % 0x1_0000 == 0 && len >= 0x1_0000 {
                (0x1_0000, CMD_BLOCK_ER64, TMO_64K)
            } else if addr % 0x8000 == 0 && len >= 0x8000 {
                (0x8000, CMD_BLOCK_ER32, TMO_32K)
            } else {
                (0x1000, CMD_SECTOR_ER, TMO_4K)
            };
            self.erase_one(addr, cmd, tmo)?;
            addr += chunk;
            len -= chunk;
        }
        unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
        Ok(())
    }
}
