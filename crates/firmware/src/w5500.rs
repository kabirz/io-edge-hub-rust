//! W5500 hardware-protocol-stack (TOE) socket driver: register-level access
//! over blocking SPI2. ARP/IP/ICMP/TCP/UDP all run inside the chip; the STM32
//! only moves payloads. This replaces the MACRAW + smoltcp path on this
//! branch (UDP config/upgrade + Modbus TCP only, no HTTP/FTP).
//!
//! Socket plan (the chip has 8 sockets and a fixed 16 KiB RX / 16 KiB TX
//! buffer pool, sizes in {0,1,2,4,8,16} KiB per socket):
//!   sock 0  UDP  :8600  RX 8K (upgrade windows)  TX 2K
//!   sock 1  TCP  :502   RX 2K                    TX 2K
//!   sock 2  TCP  :502   RX 2K                    TX 2K
//!   sock 3-7 closed (sizes zeroed so they give their default 2K back)
//! A 3rd Modbus master finds no listener and the chip answers RST — the same
//! cap and semantics the C firmware had.
//!
//! All methods are synchronous polling (bounded waits only); the net task
//! calls them from thread mode. Buffer-wrap handling splits transfers at the
//! socket buffer edge, mirroring the WIZnet access patterns.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{mode::Master, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peri;

// ---- SPI control phase (W5500 datasheet 3.1): [addr_hi addr_lo ctrl] ----
// BSB (block select): common regs / socket-n regs / socket-n TX / RX buffers
const BSB_COMMON: u8 = 0x00;
const BSB_SOCK_REG: u8 = 0x01; // + n
const BSB_SOCK_TX: u8 = 0x09; // + n
const BSB_SOCK_RX: u8 = 0x11; // + n

fn ctrl(bsb: u8, write: bool) -> u8 {
    (bsb << 3) | ((write as u8) << 2) // OM = 00: variable data length
}

// ---- common registers ----
const R_GAR: u16 = 0x0001; // gateway, 4B
const R_SUBR: u16 = 0x0005; // subnet mask, 4B
const R_SHAR: u16 = 0x0009; // MAC, 6B
const R_SIPR: u16 = 0x000F; // source IP, 4B
const R_VERSIONR: u16 = 0x001F; // reads 0x04
const R_PHYCFGR: u16 = 0x002E; // bit7 = link up

// ---- socket registers (block 0x01+n) ----
const S_MR: u16 = 0x0000;
const S_CR: u16 = 0x0001;
const S_IR: u16 = 0x0002;
const S_SR: u16 = 0x0003;
const S_PORT: u16 = 0x0004; // 2B
const S_DIPR: u16 = 0x000C; // 4B
const S_DPORT: u16 = 0x0010; // 2B
const S_TXBUF: u16 = 0x001E;
const S_RXBUF: u16 = 0x001F;
const S_TX_FSR: u16 = 0x0020; // 2B, free TX bytes
const S_TX_WR: u16 = 0x0024; // 2B
const S_RX_RSR: u16 = 0x0026; // 2B, received bytes
const S_RX_RD: u16 = 0x0028; // 2B

const MR_TCP_ND: u8 = 0x21; // TCP + no-delayed-ACK (good for Modbus)
const MR_UDP: u8 = 0x02;
const CR_OPEN: u8 = 0x01;
const CR_LISTEN: u8 = 0x02;
const CR_CLOSE: u8 = 0x10;
const CR_SEND: u8 = 0x20;
const CR_RECV: u8 = 0x40;

pub const SR_INIT: u8 = 0x13;
pub const SR_LISTEN: u8 = 0x14;
pub const SR_ESTABLISHED: u8 = 0x17;
pub const SR_CLOSE_WAIT: u8 = 0x1C;

pub const SOCK_UDP: u8 = 0;
pub const SOCK_MB1: u8 = 1;
pub const SOCK_MB2: u8 = 2;

const UDP_RX_KB: u16 = 8; // 8x1400B upgrade windows: 5 frames buffered,
                          // go-back-N covers the tail; 16K is impossible (Modbus needs 2x2K of the
                          // 16K pool and sizes must be powers of two).
const MB_KB: u16 = 2; // Modbus ADU max is 260B

pub struct W5500Pins {
    pub spi2: Peri<'static, embassy_stm32::peripherals::SPI2>,
    pub sck: Peri<'static, embassy_stm32::peripherals::PB13>,
    pub miso: Peri<'static, embassy_stm32::peripherals::PB14>,
    pub mosi: Peri<'static, embassy_stm32::peripherals::PB15>,
    pub cs: Peri<'static, embassy_stm32::peripherals::PB12>,
    pub rst: Peri<'static, embassy_stm32::peripherals::PD0>,
}

pub struct W5500 {
    spi: Spi<'static, Blocking, Master>,
    cs: Output<'static>,
    rx_kb: [u16; 8],
    tx_kb: [u16; 8],
}

impl W5500 {
    /// SPI2 @21 MHz, hardware reset, VERSIONR check, zero all socket buffers.
    /// Delays inside are async on purpose (called from a task).
    pub async fn new(p: W5500Pins) -> Option<Self> {
        let mut cfg = SpiConfig::default();
        cfg.frequency = Hertz(21_000_000);
        let spi = Spi::new_blocking(p.spi2, p.sck, p.mosi, p.miso, cfg);
        let cs = Output::new(p.cs, Level::High, Speed::VeryHigh);
        let mut rst = Output::new(p.rst, Level::Low, Speed::Low); // reset assert
        let mut w = Self {
            spi,
            cs,
            rx_kb: [0; 8],
            tx_kb: [0; 8],
        };
        embassy_time::Timer::after_millis(2).await; // RST low >= 500us
        rst.set_level(Level::High);
        embassy_time::Timer::after_millis(50).await; // power/PHY settle
        let ok = (0..3).any(|_| w.rd8(BSB_COMMON, R_VERSIONR) == 0x04);
        if !ok {
            return None;
        }
        // hand the default 2K+2K of every socket back to the pool before any
        // socket opens (buffer sizes only take effect while CLOSED)
        for n in 0u8..8 {
            w.wr8(BSB_SOCK_REG + n, S_TXBUF, 0);
            w.wr8(BSB_SOCK_REG + n, S_RXBUF, 0);
        }
        Some(w)
    }

    // ================= common =================

    pub fn set_netconf(&mut self, mac: &[u8; 6], ip: &[u8; 4], mask: &[u8; 4], gw: &[u8; 4]) {
        self.wr(BSB_COMMON, R_GAR, gw);
        self.wr(BSB_COMMON, R_SUBR, mask);
        self.wr(BSB_COMMON, R_SHAR, mac);
        self.wr(BSB_COMMON, R_SIPR, ip);
    }

    /// PHY link up (auto-negotiation defaults are untouched after reset).
    pub fn link_up(&mut self) -> bool {
        self.rd8(BSB_COMMON, R_PHYCFGR) & 0x80 != 0
    }

    // ================= UDP socket =================

    pub fn udp_open(&mut self, sock: u8, port: u16) {
        self.rx_kb[sock as usize] = UDP_RX_KB;
        self.tx_kb[sock as usize] = 2;
        self.wr8(BSB_SOCK_REG + sock, S_RXBUF, UDP_RX_KB as u8);
        self.wr8(BSB_SOCK_REG + sock, S_TXBUF, 2);
        self.wr8(BSB_SOCK_REG + sock, S_MR, MR_UDP);
        self.wr16(BSB_SOCK_REG + sock, S_PORT, port);
        self.cmd(sock, CR_OPEN);
    }

    /// Poll one datagram. Returns (payload len, sender ip, sender port).
    pub fn udp_recv(&mut self, sock: u8, buf: &mut [u8]) -> Option<(usize, [u8; 4], u16)> {
        if self.rd16(BSB_SOCK_REG + sock, S_RX_RSR) == 0 {
            return None;
        }
        let rd = self.rd16(BSB_SOCK_REG + sock, S_RX_RD);
        let mut hdr = [0u8; 8]; // [ip 4B][port 2B][len 2B] then payload
        self.buf_read(sock, false, rd, &mut hdr);
        let len = u16::from_be_bytes([hdr[6], hdr[7]]) as usize;
        let n = len.min(buf.len());
        self.buf_read(sock, false, rd.wrapping_add(8), &mut buf[..n]);
        self.wr16(
            BSB_SOCK_REG + sock,
            S_RX_RD,
            rd.wrapping_add((8 + len) as u16),
        );
        self.cmd(sock, CR_RECV);
        let ip = [hdr[0], hdr[1], hdr[2], hdr[3]];
        let port = u16::from_be_bytes([hdr[4], hdr[5]]);
        Some((n, ip, port))
    }

    /// Fire-and-forget send: silently dropped when the TX buffer is short on
    /// space (UDP is lossy by design; hosts retry / go-back-N).
    pub fn udp_send_to(&mut self, sock: u8, data: &[u8], ip: &[u8; 4], port: u16) {
        if (self.rd16(BSB_SOCK_REG + sock, S_TX_FSR) as usize) < data.len() {
            return;
        }
        self.wr(BSB_SOCK_REG + sock, S_DIPR, ip);
        self.wr16(BSB_SOCK_REG + sock, S_DPORT, port);
        let wr = self.rd16(BSB_SOCK_REG + sock, S_TX_WR);
        self.buf_write(sock, wr, data);
        self.wr16(
            BSB_SOCK_REG + sock,
            S_TX_WR,
            wr.wrapping_add(data.len() as u16),
        );
        self.cmd(sock, CR_SEND);
    }

    // ================= TCP server socket =================

    pub fn tcp_listen(&mut self, sock: u8, port: u16) {
        self.rx_kb[sock as usize] = MB_KB;
        self.tx_kb[sock as usize] = MB_KB;
        self.wr8(BSB_SOCK_REG + sock, S_RXBUF, MB_KB as u8);
        self.wr8(BSB_SOCK_REG + sock, S_TXBUF, MB_KB as u8);
        self.wr8(BSB_SOCK_REG + sock, S_MR, MR_TCP_ND);
        self.wr16(BSB_SOCK_REG + sock, S_PORT, port);
        self.cmd(sock, CR_OPEN);
        self.cmd(sock, CR_LISTEN);
    }

    pub fn tcp_state(&mut self, sock: u8) -> u8 {
        self.rd8(BSB_SOCK_REG + sock, S_SR)
    }

    /// Received-but-undrained byte count (peek, does not consume).
    pub fn tcp_rx_pending(&mut self, sock: u8) -> u16 {
        self.rd16(BSB_SOCK_REG + sock, S_RX_RSR)
    }

    /// Abort (RST when data is pending — smoltcp `abort()` semantics) and
    /// re-arm the listener: the port must never linger without a listener.
    pub fn tcp_close_reopen(&mut self, sock: u8, port: u16) {
        self.wr8(BSB_SOCK_REG + sock, S_IR, 0xFF); // clear stale flags
        self.cmd(sock, CR_CLOSE);
        self.tcp_listen(sock, port);
    }

    /// Drain up to buf.len() received bytes (0 when nothing pending).
    pub fn tcp_recv(&mut self, sock: u8, buf: &mut [u8]) -> usize {
        let rsr = self.rd16(BSB_SOCK_REG + sock, S_RX_RSR) as usize;
        if rsr == 0 {
            return 0;
        }
        let n = rsr.min(buf.len());
        let rd = self.rd16(BSB_SOCK_REG + sock, S_RX_RD);
        self.buf_read(sock, false, rd, &mut buf[..n]);
        self.wr16(BSB_SOCK_REG + sock, S_RX_RD, rd.wrapping_add(n as u16));
        self.cmd(sock, CR_RECV);
        n
    }

    /// False when the TX free space cannot take the frame yet (pipelined
    /// replies waiting for ACK): keep the reply and retry next poll.
    pub fn tcp_try_send(&mut self, sock: u8, data: &[u8]) -> bool {
        if (self.rd16(BSB_SOCK_REG + sock, S_TX_FSR) as usize) < data.len() {
            return false;
        }
        let wr = self.rd16(BSB_SOCK_REG + sock, S_TX_WR);
        self.buf_write(sock, wr, data);
        self.wr16(
            BSB_SOCK_REG + sock,
            S_TX_WR,
            wr.wrapping_add(data.len() as u16),
        );
        self.cmd(sock, CR_SEND);
        true
    }

    // ================= low level =================

    fn cmd(&mut self, sock: u8, op: u8) {
        self.wr8(BSB_SOCK_REG + sock, S_CR, op);
        // commands self-clear in a few SPI ticks; bound the wait so a wedged
        // chip cannot stall the poll loop forever
        for _ in 0..64 {
            if self.rd8(BSB_SOCK_REG + sock, S_CR) == 0 {
                return;
            }
        }
    }

    fn rx_mask(&self, sock: u8) -> u16 {
        self.rx_kb[sock as usize] * 1024 - 1
    }

    fn tx_mask(&self, sock: u8) -> u16 {
        self.tx_kb[sock as usize] * 1024 - 1
    }

    /// Read from a socket RX/TX buffer at raw pointer, split at the wrap.
    fn buf_read(&mut self, sock: u8, tx: bool, ptr: u16, out: &mut [u8]) {
        let mask = if tx {
            self.tx_mask(sock)
        } else {
            self.rx_mask(sock)
        };
        let bsb = if tx {
            BSB_SOCK_TX + sock
        } else {
            BSB_SOCK_RX + sock
        };
        let start = (ptr & mask) as usize;
        let first = out.len().min(mask as usize + 1 - start);
        self.rd(bsb, ptr & mask, &mut out[..first]);
        if first < out.len() {
            self.rd(bsb, 0, &mut out[first..]);
        }
    }

    fn buf_write(&mut self, sock: u8, ptr: u16, data: &[u8]) {
        let mask = self.tx_mask(sock);
        let bsb = BSB_SOCK_TX + sock;
        let start = (ptr & mask) as usize;
        let first = data.len().min(mask as usize + 1 - start);
        self.wr(bsb, ptr & mask, &data[..first]);
        if first < data.len() {
            self.wr(bsb, 0, &data[first..]);
        }
    }

    fn wr(&mut self, bsb: u8, addr: u16, data: &[u8]) {
        let hdr = [(addr >> 8) as u8, addr as u8, ctrl(bsb, true)];
        self.cs.set_level(Level::Low);
        let _ = self.spi.blocking_write(&hdr);
        let _ = self.spi.blocking_write(data);
        self.cs.set_level(Level::High);
    }

    fn rd(&mut self, bsb: u8, addr: u16, out: &mut [u8]) {
        let hdr = [(addr >> 8) as u8, addr as u8, ctrl(bsb, false)];
        self.cs.set_level(Level::Low);
        let _ = self.spi.blocking_write(&hdr);
        let _ = self.spi.blocking_read(out);
        self.cs.set_level(Level::High);
    }

    fn wr8(&mut self, bsb: u8, addr: u16, v: u8) {
        self.wr(bsb, addr, &[v]);
    }

    fn wr16(&mut self, bsb: u8, addr: u16, v: u16) {
        self.wr(bsb, addr, &v.to_be_bytes());
    }

    fn rd8(&mut self, bsb: u8, addr: u16) -> u8 {
        let mut b = [0u8; 1];
        self.rd(bsb, addr, &mut b);
        b[0]
    }

    fn rd16(&mut self, bsb: u8, addr: u16) -> u16 {
        let mut b = [0u8; 2];
        self.rd(bsb, addr, &mut b);
        u16::from_be_bytes(b)
    }
}
