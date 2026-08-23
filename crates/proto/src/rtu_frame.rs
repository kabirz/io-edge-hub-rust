//! Modbus RTU slave frame state machine, ported 1:1 from
//! src/modbus/rtu_frame.c. Transport feeds bytes + t3.5 expiry; this module
//! does frame assembly, CRC check, unicast filter, decode and reply assembly.

use crate::crc::crc16_modbus;
use crate::mb_server::{MB_SERVER_PDU_MAX, MbDiag, MbServer};
use crate::regmap::{RegHooks, RegMap};

pub const RTU_FRAME_MAX: usize = 256;
pub const RTU_FRAME_MIN: usize = 4;

/// t3.5 in ms (rtu_t35_ms): fixed 2ms above 19200 baud, else ceil to ms.
pub fn rtu_t35_ms(baud: u32) -> u32 {
    if baud == 0 || baud > 19200 {
        return 2;
    }
    let us = (38_500_000u32 + baud - 1) / baud;
    (us + 999) / 1000
}

/// Frame assembler. `rx_feed` may run from a UART context; `t35_expired`
/// runs from the RTU task after the silence timeout.
pub struct RtuFrame {
    rx_buf: [u8; RTU_FRAME_MAX],
    rx_len: usize,
    rx_overflow: bool,
    srv_unit: u8,
}

impl Default for RtuFrame {
    fn default() -> Self {
        Self::new()
    }
}

impl RtuFrame {
    pub const fn new() -> Self {
        Self {
            rx_buf: [0; RTU_FRAME_MAX],
            rx_len: 0,
            rx_overflow: false,
            srv_unit: 1,
        }
    }

    pub fn bind(&mut self, unit: u8) {
        self.srv_unit = unit;
        self.reset();
    }

    pub fn reset(&mut self) {
        self.rx_len = 0;
        self.rx_overflow = false;
    }

    /// Feed received bytes; kick the t3.5 timer in the transport.
    pub fn rx_feed(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        if !self.rx_overflow {
            let space = RTU_FRAME_MAX - self.rx_len;
            let take = bytes.len().min(space);
            self.rx_buf[self.rx_len..self.rx_len + take].copy_from_slice(&bytes[..take]);
            self.rx_len += take;
            if take < bytes.len() {
                self.rx_overflow = true; // drop rest until reset
            }
        }
        // caller kicks t3.5
    }

    /// t3.5 silence expired: process the assembled frame.
    /// Returns Some(reply frame) to transmit (unicast reply).
    pub fn t35_expired(
        &mut self,
        srv: &mut MbServer,
        regs: &mut RegMap,
        hooks: &mut impl RegHooks,
        tx_frame: &mut [u8; 1 + MB_SERVER_PDU_MAX + 2],
    ) -> Option<usize> {
        let len = self.rx_len;
        let overflow = self.rx_overflow;
        // snapshot then reset: bytes arriving during processing are next frame
        self.reset();

        if len == 0 {
            return None;
        }

        if overflow || len < RTU_FRAME_MIN {
            srv.diag_count(MbDiag::BusMsg);
            srv.diag_count(MbDiag::NoResp);
            return None;
        }

        let unit = self.rx_buf[0];
        let crc_rx = self.rx_buf[len - 2] as u16 | ((self.rx_buf[len - 1] as u16) << 8);
        let crc_calc = crc16_modbus(&self.rx_buf[..len - 2]);
        if crc_rx != crc_calc {
            srv.diag_count(MbDiag::BusMsg);
            srv.diag_count(MbDiag::CrcErr);
            srv.diag_count(MbDiag::NoResp);
            return None;
        }

        if unit != 0 && unit != self.srv_unit {
            srv.diag_count(MbDiag::BusMsg);
            srv.diag_count(MbDiag::NoResp);
            return None;
        }

        // deliver to decoder (unicast + broadcast): bus/srv counted inside
        let mut rsp_pdu = [0u8; MB_SERVER_PDU_MAX];
        let rsp_len = srv.process(&self.rx_buf[1..len - 2], &mut rsp_pdu, regs, hooks);

        if rsp_len == 0 || unit == 0 {
            srv.diag_count(MbDiag::NoResp);
            return None;
        }

        // reply: echo unit + PDU + crc16 LE
        tx_frame[0] = unit;
        tx_frame[1..1 + rsp_len].copy_from_slice(&rsp_pdu[..rsp_len]);
        let crc = crc16_modbus(&tx_frame[..1 + rsp_len]);
        tx_frame[1 + rsp_len] = (crc & 0xFF) as u8;
        tx_frame[2 + rsp_len] = (crc >> 8) as u8;
        Some(rsp_len + 3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmap::NoHooks;

    fn frame(unit: u8, pdu: &[u8]) -> Vec<u8> {
        let mut f = vec![unit];
        f.extend_from_slice(pdu);
        let crc = crc16_modbus(&f);
        f.push((crc & 0xFF) as u8);
        f.push((crc >> 8) as u8);
        f
    }

    #[test]
    fn unicast_roundtrip() {
        let mut rtu = RtuFrame::new();
        rtu.bind(1);
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut tx = [0u8; 1 + MB_SERVER_PDU_MAX + 2];

        let f = frame(1, &[0x03, 0x00, 0x0A, 0x00, 0x04]);
        rtu.rx_feed(&f);
        let n = rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).unwrap();
        assert_eq!(tx[0], 1);
        assert_eq!(tx[1], 3);
        // reply CRC valid
        let crc_rx = tx[n - 2] as u16 | ((tx[n - 1] as u16) << 8);
        assert_eq!(crc_rx, crc16_modbus(&tx[..n - 2]));
    }

    #[test]
    fn broadcast_side_effects_no_reply() {
        let mut rtu = RtuFrame::new();
        rtu.bind(1);
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut tx = [0u8; 1 + MB_SERVER_PDU_MAX + 2];
        let f = frame(0, &[0x06, 0x00, 0x00, 0x00, 0x77]);
        rtu.rx_feed(&f);
        assert!(rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).is_none());
        assert_eq!(regs.get_holding(0x00), 0x77);
        assert_eq!(srv.diag_get(MbDiag::NoResp), 1);
    }

    #[test]
    fn crc_error_silent_counted() {
        let mut rtu = RtuFrame::new();
        rtu.bind(1);
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut tx = [0u8; 1 + MB_SERVER_PDU_MAX + 2];
        let mut f = frame(1, &[0x03, 0x00, 0x00, 0x00, 0x01]);
        f[3] ^= 0xFF; // corrupt
        rtu.rx_feed(&f);
        assert!(rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).is_none());
        assert_eq!(srv.diag_get(MbDiag::CrcErr), 1);
        assert_eq!(srv.diag_get(MbDiag::BusMsg), 1);
    }

    #[test]
    fn other_unit_silent() {
        let mut rtu = RtuFrame::new();
        rtu.bind(1);
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut tx = [0u8; 1 + MB_SERVER_PDU_MAX + 2];
        let f = frame(5, &[0x03, 0x00, 0x00, 0x00, 0x01]);
        rtu.rx_feed(&f);
        assert!(rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).is_none());
        assert_eq!(srv.diag_get(MbDiag::BusMsg), 1);
    }

    #[test]
    fn short_and_overflow_frames_silent() {
        let mut rtu = RtuFrame::new();
        rtu.bind(1);
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut tx = [0u8; 1 + MB_SERVER_PDU_MAX + 2];
        rtu.rx_feed(&[0x01, 0x03]); // len < 4
        assert!(rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).is_none());
        // overflow: feed > 256 bytes
        rtu.rx_feed(&[0u8; 300]);
        assert!(rtu.t35_expired(&mut srv, &mut regs, &mut h, &mut tx).is_none());
        assert_eq!(srv.diag_get(MbDiag::NoResp), 2);
    }

    #[test]
    fn t35_values() {
        assert_eq!(rtu_t35_ms(9600), 5); // 38.5ms/9600 = 4.01 -> ceil 5? (38500000+9599)/9600=4011us -> ceil ms 5
        assert_eq!(rtu_t35_ms(19200), 3);
        assert_eq!(rtu_t35_ms(115200), 2);
        assert_eq!(rtu_t35_ms(0), 2);
    }
}
