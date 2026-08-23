//! Modbus PDU decoder, ported 1:1 from src/modbus/mb_server.c (uC/Modbus
//! derived). This module only sees the PDU (fc + data); ADU framing (MBAP /
//! RTU CRC / unit filter / broadcast suppression) belongs to the transports.

use crate::bytes;
use crate::regmap::{RegHooks, RegMap};

pub const MB_SERVER_PDU_MAX: usize = 256;

// function codes
pub const MB_FC01_COIL_RD: u8 = 0x01;
pub const MB_FC02_DI_RD: u8 = 0x02;
pub const MB_FC03_HOLDING_REG_RD: u8 = 0x03;
pub const MB_FC04_IN_REG_RD: u8 = 0x04;
pub const MB_FC05_COIL_WR: u8 = 0x05;
pub const MB_FC06_HOLDING_REG_WR: u8 = 0x06;
pub const MB_FC08_DIAGNOSTICS: u8 = 0x08;
pub const MB_FC15_COILS_WR: u8 = 0x0F;
pub const MB_FC16_HOLDING_REGS_WR: u8 = 0x10;

// FC08 subfunctions
pub const MB_FC08_SUBF_QUERY: u16 = 0x0000;
pub const MB_FC08_SUBF_CLR_CTR: u16 = 0x000A;
pub const MB_FC08_SUBF_BUS_MSG_CTR: u16 = 0x000B;
pub const MB_FC08_SUBF_BUS_CRC_CTR: u16 = 0x000C;
pub const MB_FC08_SUBF_BUS_EXCEPT_CTR: u16 = 0x000D;
pub const MB_FC08_SUBF_SERVER_MSG_CTR: u16 = 0x000E;
pub const MB_FC08_SUBF_SERVER_NO_RESP_CTR: u16 = 0x000F;

// exception codes
pub const MB_EXC_ILLEGAL_FC: u8 = 0x01;
pub const MB_EXC_ILLEGAL_DATA_ADDR: u8 = 0x02;
pub const MB_EXC_ILLEGAL_DATA_VAL: u8 = 0x03;

pub const MB_FP_EXTENSIONS_ADDR: u16 = 5000;

/// Diagnostic counters (FC08).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum MbDiag {
    BusMsg = 0,
    CrcErr = 1,
    Exc = 2,
    SrvMsg = 3,
    NoResp = 4,
}
pub use MbDiag::*;

/// PDU server: diagnostics + one PDU decode. Register access goes through the
/// RegMap (+hooks) so timestamp registers read live time and writes take
/// side effects exactly like the C firmware.
pub struct MbServer {
    diag: [u16; 5],
}

impl Default for MbServer {
    fn default() -> Self {
        Self::new()
    }
}

impl MbServer {
    pub const fn new() -> Self {
        Self { diag: [0; 5] }
    }

    pub fn diag_count(&mut self, c: MbDiag) {
        self.diag[c as usize] = self.diag[c as usize].saturating_add(1);
    }

    pub fn diag_get(&self, c: MbDiag) -> u16 {
        self.diag[c as usize]
    }

    /// Decode one PDU. Returns response length written to `out`
    /// (0 = silent drop, e.g. length violation).
    pub fn process(&mut self, in_: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if in_.is_empty() {
            return 0;
        }
        let fc = in_[0];
        let d = &in_[1..];

        // every PDU reaching the decoder counts bus_msg + srv_msg
        self.diag_count(BusMsg);
        self.diag_count(SrvMsg);

        match fc {
            MB_FC01_COIL_RD => self.fc01(d, out, regs),
            MB_FC02_DI_RD => self.fc02(d, out, regs),
            MB_FC03_HOLDING_REG_RD => self.fc03(d, out, regs, hooks),
            MB_FC04_IN_REG_RD => self.fc04(d, out, regs),
            MB_FC05_COIL_WR => self.fc05(d, out, regs, hooks),
            MB_FC06_HOLDING_REG_WR => self.fc06(d, out, regs, hooks),
            MB_FC08_DIAGNOSTICS => self.fc08(d, out),
            MB_FC15_COILS_WR => self.fc15(d, out, regs, hooks),
            MB_FC16_HOLDING_REGS_WR => self.fc16(d, out, regs, hooks),
            _ => self.exc_rsp(fc, MB_EXC_ILLEGAL_FC, out), // unknown FC -> 0x01
        }
    }

    fn exc_rsp(&mut self, fc: u8, code: u8, out: &mut [u8]) -> usize {
        self.diag_count(Exc);
        out[0] = fc | 0x80;
        out[1] = code;
        2
    }

    fn fc01(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap) -> usize {
        if d.len() != 4 {
            return 0; // silent drop
        }
        let mut coil_addr = bytes::get_be16(&d[0..2]);
        let coil_qty = bytes::get_be16(&d[2..4]);
        if coil_qty == 0 || coil_qty > 2000 {
            return self.exc_rsp(MB_FC01_COIL_RD, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        let num_bytes = ((coil_qty - 1) / 8 + 1) as usize;
        out[0] = MB_FC01_COIL_RD;
        out[1] = num_bytes as u8;
        out[2..2 + num_bytes].fill(0);
        for i in 0..coil_qty as usize {
            let state = match regs.io_coil_rd(coil_addr) {
                Ok(s) => s,
                Err(_) => {
                    return self.exc_rsp(MB_FC01_COIL_RD, MB_EXC_ILLEGAL_DATA_ADDR, out);
                }
            };
            if state {
                out[2 + i / 8] |= 1 << (i % 8);
            }
            coil_addr = coil_addr.wrapping_add(1);
        }
        num_bytes + 2
    }

    fn fc02(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let mut di_addr = bytes::get_be16(&d[0..2]);
        let di_qty = bytes::get_be16(&d[2..4]);
        if di_qty == 0 || di_qty > 2000 {
            return self.exc_rsp(MB_FC02_DI_RD, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        let num_bytes = ((di_qty - 1) / 8 + 1) as usize;
        out[0] = MB_FC02_DI_RD;
        out[1] = num_bytes as u8;
        out[2..2 + num_bytes].fill(0);
        for i in 0..di_qty as usize {
            let state = match regs.io_discrete_rd(di_addr) {
                Ok(s) => s,
                Err(_) => {
                    return self.exc_rsp(MB_FC02_DI_RD, MB_EXC_ILLEGAL_DATA_ADDR, out);
                }
            };
            if state {
                out[2 + i / 8] |= 1 << (i % 8);
            }
            di_addr = di_addr.wrapping_add(1);
        }
        num_bytes + 2
    }

    fn fc03(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let mut reg_addr = bytes::get_be16(&d[0..2]);
        let reg_qty = bytes::get_be16(&d[2..4]);
        if reg_qty == 0 || reg_qty > 125 {
            return self.exc_rsp(MB_FC03_HOLDING_REG_RD, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        if reg_addr >= MB_FP_EXTENSIONS_ADDR {
            return self.exc_rsp(MB_FC03_HOLDING_REG_RD, MB_EXC_ILLEGAL_FC, out);
        }
        let num_bytes = reg_qty as usize * 2;
        out[0] = MB_FC03_HOLDING_REG_RD;
        out[1] = num_bytes as u8;
        for i in 0..reg_qty as usize {
            if reg_addr >= crate::regmap::MODBUS_HOLDING_REGISTER_NUMBERS as u16 {
                return self.exc_rsp(MB_FC03_HOLDING_REG_RD, MB_EXC_ILLEGAL_DATA_ADDR, out);
            }
            let reg = regs.io_read_holding(reg_addr, hooks);
            bytes::put_be16(reg, &mut out[2 + i * 2..4 + i * 2]);
            reg_addr += 1;
        }
        num_bytes + 2
    }

    fn fc04(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let mut reg_addr = bytes::get_be16(&d[0..2]);
        let reg_qty = bytes::get_be16(&d[2..4]);
        if reg_qty == 0 || reg_qty > 125 {
            return self.exc_rsp(MB_FC04_IN_REG_RD, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        if reg_addr >= MB_FP_EXTENSIONS_ADDR {
            return self.exc_rsp(MB_FC04_IN_REG_RD, MB_EXC_ILLEGAL_FC, out);
        }
        let num_bytes = reg_qty as usize * 2;
        out[0] = MB_FC04_IN_REG_RD;
        out[1] = num_bytes as u8;
        for i in 0..reg_qty as usize {
            if reg_addr >= crate::regmap::MODBUS_INPUT_REGISTER_NUMBERS as u16 {
                return self.exc_rsp(MB_FC04_IN_REG_RD, MB_EXC_ILLEGAL_DATA_ADDR, out);
            }
            let reg = regs.get_input(reg_addr);
            bytes::put_be16(reg, &mut out[2 + i * 2..4 + i * 2]);
            reg_addr += 1;
        }
        num_bytes + 2
    }

    fn fc05(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let coil_addr = bytes::get_be16(&d[0..2]);
        let coil_val = bytes::get_be16(&d[2..4]);
        let state = coil_val != 0x0000;
        if regs.io_write_do_bit(coil_addr, state, hooks).is_err() {
            return self.exc_rsp(MB_FC05_COIL_WR, MB_EXC_ILLEGAL_DATA_ADDR, out);
        }
        out[0] = MB_FC05_COIL_WR;
        bytes::put_be16(coil_addr, &mut out[1..3]);
        bytes::put_be16(coil_val, &mut out[3..5]); // echo original value
        5
    }

    fn fc06(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let reg_addr = bytes::get_be16(&d[0..2]);
        let reg_val = bytes::get_be16(&d[2..4]);
        if regs.io_write_holding(reg_addr, reg_val, hooks).is_err() {
            return self.exc_rsp(MB_FC06_HOLDING_REG_WR, MB_EXC_ILLEGAL_DATA_ADDR, out);
        }
        out[0] = MB_FC06_HOLDING_REG_WR;
        bytes::put_be16(reg_addr, &mut out[1..3]);
        bytes::put_be16(reg_val, &mut out[3..5]);
        5
    }

    fn fc08(&mut self, d: &[u8], out: &mut [u8]) -> usize {
        if d.len() != 4 {
            return 0;
        }
        let sfunc = bytes::get_be16(&d[0..2]);
        let mut data = bytes::get_be16(&d[2..4]);
        match sfunc {
            MB_FC08_SUBF_QUERY => {}
            MB_FC08_SUBF_CLR_CTR => self.diag = [0; 5],
            MB_FC08_SUBF_BUS_MSG_CTR => data = self.diag_get(BusMsg),
            MB_FC08_SUBF_BUS_CRC_CTR => data = self.diag_get(CrcErr),
            MB_FC08_SUBF_BUS_EXCEPT_CTR => data = self.diag_get(Exc),
            MB_FC08_SUBF_SERVER_MSG_CTR => data = self.diag_get(SrvMsg),
            MB_FC08_SUBF_SERVER_NO_RESP_CTR => data = self.diag_get(NoResp),
            _ => return self.exc_rsp(MB_FC08_DIAGNOSTICS, MB_EXC_ILLEGAL_FC, out),
        }
        out[0] = MB_FC08_DIAGNOSTICS;
        bytes::put_be16(sfunc, &mut out[1..3]);
        bytes::put_be16(data, &mut out[3..5]);
        5
    }

    fn fc15(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if d.len() < 6 {
            return 0;
        }
        let coil_addr = bytes::get_be16(&d[0..2]);
        let coil_qty = bytes::get_be16(&d[2..4]);
        let num_bytes = d[4] as usize;
        if coil_qty == 0 || coil_qty > 2000 {
            return self.exc_rsp(MB_FC15_COILS_WR, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        if ((coil_qty - 1) / 8 + 1) as usize != num_bytes || d.len() != num_bytes + 5 {
            return self.exc_rsp(MB_FC15_COILS_WR, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        let mut temp = 0u8;
        for i in 0..coil_qty as usize {
            if i % 8 == 0 {
                temp = d[5 + i / 8];
            }
            let state = temp & 0x01 != 0;
            if regs.io_write_do_bit(coil_addr.wrapping_add(i as u16), state, hooks).is_err() {
                return self.exc_rsp(MB_FC15_COILS_WR, MB_EXC_ILLEGAL_DATA_ADDR, out);
            }
            temp >>= 1;
        }
        out[0] = MB_FC15_COILS_WR;
        bytes::put_be16(coil_addr, &mut out[1..3]);
        bytes::put_be16(coil_qty, &mut out[3..5]);
        5
    }

    fn fc16(&mut self, d: &[u8], out: &mut [u8], regs: &mut RegMap, hooks: &mut impl RegHooks) -> usize {
        if d.len() < 6 {
            return 0;
        }
        let reg_addr = bytes::get_be16(&d[0..2]);
        let reg_qty = bytes::get_be16(&d[2..4]);
        let num_bytes = d[4];
        if reg_qty == 0 || reg_qty > 125 {
            return self.exc_rsp(MB_FC16_HOLDING_REGS_WR, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        if reg_addr >= MB_FP_EXTENSIONS_ADDR {
            return self.exc_rsp(MB_FC16_HOLDING_REGS_WR, MB_EXC_ILLEGAL_FC, out);
        }
        // (dlen-5) != num_bytes -> 0x03
        if d.len() as u16 - 5 != num_bytes as u16 {
            return self.exc_rsp(MB_FC16_HOLDING_REGS_WR, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        // integer-division quirk preserved: num_bytes/reg_qty != 2 -> 0x03
        if num_bytes / reg_qty as u8 != 2 {
            return self.exc_rsp(MB_FC16_HOLDING_REGS_WR, MB_EXC_ILLEGAL_DATA_VAL, out);
        }
        for i in 0..reg_qty as usize {
            let val = bytes::get_be16(&d[5 + i * 2..7 + i * 2]);
            if regs.io_write_holding(reg_addr.wrapping_add(i as u16), val, hooks).is_err() {
                return self.exc_rsp(MB_FC16_HOLDING_REGS_WR, MB_EXC_ILLEGAL_DATA_ADDR, out);
            }
        }
        out[0] = MB_FC16_HOLDING_REGS_WR;
        bytes::put_be16(reg_addr, &mut out[1..3]);
        bytes::put_be16(reg_qty, &mut out[3..5]);
        5
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmap::NoHooks;
    use std::cell::RefCell;

    struct Mock {
        do_val: RefCell<u8>,
    }
    impl RegHooks for Mock {
        fn set_do(&mut self, v: u8) {
            *self.do_val.borrow_mut() = v;
        }
    }

    #[test]
    fn fc03_reads_defaults_and_version_reg() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        // FC03 addr=0 qty=18 -> 36 bytes, first reg DO=0, IP regs at 0x0A
        let req = [0x03, 0x00, 0x00, 0x00, 0x12];
        let n = srv.process(&req, &mut out, &mut regs, &mut h);
        assert_eq!(n, 38);
        assert_eq!(out[0], 3);
        assert_eq!(out[1], 36);
        assert_eq!(&out[2 + 0x0A * 2..2 + 0x0A * 2 + 8], &[0, 192, 0, 168, 0, 12, 0, 101][..8]);
    }

    #[test]
    fn fc03_qty_violation_exc3_fp_area_exc1() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        let n = srv.process(&[0x03, 0x00, 0x00, 0x00, 0x7E], &mut out, &mut regs, &mut h); // qty=126
        assert_eq!(&out[..n], &[0x83, 0x03]);
        // FP area >= 5000 -> ILLEGAL_FC
        let n = srv.process(&[0x03, 0x13, 0x88, 0x00, 0x01], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x83, 0x01]);
        // addr beyond 18 regs -> ILLEGAL_DATA_ADDR
        let n = srv.process(&[0x03, 0x00, 0x12, 0x00, 0x01], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x83, 0x02]);
    }

    #[test]
    fn fc06_write_drives_do() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        let n = srv.process(&[0x06, 0x00, 0x00, 0x00, 0xA5], &mut out, &mut regs, &mut h);
        assert_eq!(n, 5);
        assert_eq!(&out[..5], &[0x06, 0x00, 0x00, 0x00, 0xA5]);
        assert_eq!(*h.do_val.borrow(), 0xA5);
        // out of range -> exc 0x02
        let n = srv.process(&[0x06, 0x00, 0x12, 0x00, 0x01], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x86, 0x02]);
    }

    #[test]
    fn fc05_coil_write_and_echo() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        let n = srv.process(&[0x05, 0x00, 0x03, 0xFF, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x05, 0x00, 0x03, 0xFF, 0x00]);
        assert_eq!(*h.do_val.borrow(), 0x08);
        let n = srv.process(&[0x05, 0x00, 0x03, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x05, 0x00, 0x03, 0x00, 0x00]);
        assert_eq!(*h.do_val.borrow(), 0x00);
        // coil 8 out of range -> 0x02
        let n = srv.process(&[0x05, 0x00, 0x08, 0xFF, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x85, 0x02]);
    }

    #[test]
    fn fc01_coil_read_and_fc02_discrete() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        regs.update_holding(0x00, 0xA5).unwrap();
        regs.update_input(crate::regmap::INPUT_DI_IDX as u16, 0x8001).unwrap();
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        let n = srv.process(&[0x01, 0x00, 0x00, 0x00, 0x08], &mut out, &mut regs, &mut h);
        assert_eq!(n, 3);
        assert_eq!(&out[..3], &[0x01, 0x01, 0xA5]);
        let n = srv.process(&[0x02, 0x00, 0x00, 0x00, 0x10], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..4], &[0x02, 0x02, 0x01, 0x80]);
        // qty 0 -> exc 3; wrong dlen -> silent
        let n = srv.process(&[0x01, 0x00, 0x00, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x81, 0x03]);
        assert_eq!(srv.process(&[0x01, 0x00], &mut out, &mut regs, &mut h), 0);
    }

    #[test]
    fn fc08_diagnostics_echo_and_counters() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        // query echo
        let n = srv.process(&[0x08, 0x00, 0x00, 0xA5, 0x37], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x08, 0x00, 0x00, 0xA5, 0x37]);
        // after 1 PDU: bus=1 srv=1
        let n = srv.process(&[0x08, 0x00, 0x0B, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x08, 0x00, 0x0B, 0x00, 0x02]);
        // exceptions counted
        let _ = srv.process(&[0x63, 0x00], &mut out, &mut regs, &mut h); // unknown FC
        let n = srv.process(&[0x08, 0x00, 0x0D, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x08, 0x00, 0x0D, 0x00, 0x01]);
        // clear
        let _ = srv.process(&[0x08, 0x00, 0x0A, 0x00, 0x00], &mut out, &mut regs, &mut h);
        let n = srv.process(&[0x08, 0x00, 0x0B, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x08, 0x00, 0x0B, 0x00, 0x01]); // cleared then +this entry... this req counted
        // unknown subfunc -> exc 0x01
        let n = srv.process(&[0x08, 0x77, 0x77, 0x00, 0x00], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x88, 0x01]);
    }

    #[test]
    fn fc16_write_regs_with_quirks() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = Mock { do_val: RefCell::new(0) };
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        // write 2 regs at 0x03 (sample ms): 100, 250
        let n = srv.process(&[0x10, 0x00, 0x03, 0x00, 0x02, 0x04, 0x00, 100, 0x00, 250], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x10, 0x00, 0x03, 0x00, 0x02]);
        assert_eq!(regs.get_holding(0x03), 100);
        assert_eq!(regs.get_holding(0x04), 250);
        // length mismatch -> 0x03
        let n = srv.process(&[0x10, 0x00, 0x03, 0x00, 0x02, 0x05, 0, 0, 0, 0, 0, 0], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0x90, 0x03]);
    }

    #[test]
    fn unknown_fc_is_exc1() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MB_SERVER_PDU_MAX];
        let n = srv.process(&[0x63, 0x00, 0x01], &mut out, &mut regs, &mut h);
        assert_eq!(&out[..n], &[0xE3, 0x01]);
    }
}
