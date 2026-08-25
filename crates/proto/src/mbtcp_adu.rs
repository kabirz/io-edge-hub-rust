//! Modbus TCP ADU layer, ported 1:1 from src/modbus/mbtcp_adu.c.
//!
//! Order: MBAP parse (length clamp 256) -> proto != 0 server-failure reply
//! (before broadcast check) -> unit != 0 decode + reply (echo original unit)
//! -> unit == 0 broadcast: side effects run, no reply, NO_RESP counted.

use crate::bytes;
use crate::mb_server::{MbDiag, MbServer, MB_SERVER_PDU_MAX};
use crate::regmap::{RegHooks, RegMap};

pub const MB_EXC_SERVER_DEVICE_FAILURE: u8 = 0x04;
pub const MBTCP_MBAP_LEN_CLAMP: u16 = 256;
pub const MBTCP_ADU_TX_MAX: usize = MB_SERVER_PDU_MAX + 7;

const OFF_TRANS_ID: usize = 0;
const OFF_PROTO_ID: usize = 2;
const OFF_LENGTH: usize = 4;
const OFF_UNIT_ID: usize = 6;
const OFF_FC: usize = 7;

/// Process one complete TCP ADU. Returns reply length (0 = no response).
pub fn mbtcp_adu_process(
    in_: &[u8],
    out: &mut [u8],
    srv: &mut MbServer,
    regs: &mut RegMap,
    hooks: &mut impl RegHooks,
) -> usize {
    if in_.len() < OFF_FC + 1 || out.len() < MBTCP_ADU_TX_MAX {
        return 0;
    }
    let trans_id = bytes::get_be16(&in_[OFF_TRANS_ID..]);
    let proto_id = bytes::get_be16(&in_[OFF_PROTO_ID..]);
    let mbap_len = bytes::get_be16(&in_[OFF_LENGTH..]);
    let unit_id = in_[OFF_UNIT_ID];
    let fc = in_[OFF_FC];

    // proto != 0: server-failure reply (before broadcast check), echo proto
    if proto_id != 0 {
        bytes::put_be16(trans_id, &mut out[OFF_TRANS_ID..]);
        bytes::put_be16(proto_id, &mut out[OFF_PROTO_ID..]);
        bytes::put_be16(3, &mut out[OFF_LENGTH..]); // unit + fc + exc
        out[OFF_UNIT_ID] = unit_id;
        out[OFF_FC] = fc | 0x80;
        out[8] = MB_EXC_SERVER_DEVICE_FAILURE;
        return 9;
    }

    // length clamp: MIN(mbap_len, 256) - 2 + 1 = PDU (fc + data) length
    let mbap_len = mbap_len.min(MBTCP_MBAP_LEN_CLAMP);
    let mut pdu_len = if mbap_len >= 2 {
        (mbap_len - 2 + 1) as usize
    } else {
        1
    };
    // defensive: received bytes must cover the declared PDU
    if pdu_len > in_.len() - OFF_FC {
        pdu_len = in_.len() - OFF_FC;
    }

    let rsp_len = srv.process(
        &in_[OFF_FC..OFF_FC + pdu_len],
        &mut out[OFF_FC..],
        regs,
        hooks,
    );

    if unit_id == 0 {
        // broadcast: side effects executed, no reply
        srv.diag_count(MbDiag::NoResp);
        return 0;
    }
    if rsp_len == 0 {
        // PDU length violation: decoder dropped silently
        srv.diag_count(MbDiag::NoResp);
        return 0;
    }

    // reply: trans echo + proto 0 + length = 1 + PDU len + original unit
    bytes::put_be16(trans_id, &mut out[OFF_TRANS_ID..]);
    bytes::put_be16(0, &mut out[OFF_PROTO_ID..]);
    bytes::put_be16(rsp_len as u16 + 1, &mut out[OFF_LENGTH..]);
    out[OFF_UNIT_ID] = unit_id;
    OFF_FC + rsp_len
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmap::NoHooks;
    use crate::regmap::HOLDING_IP_OCTET1_IDX;

    #[test]
    fn roundtrip_fc03() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MBTCP_ADU_TX_MAX];
        // MBAP tid=0x1234 proto=0 len=6 unit=1 + FC03 addr 0x0A qty 4
        let req = [
            0x12, 0x34, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x00, 0x04,
        ];
        let n = mbtcp_adu_process(&req, &mut out, &mut srv, &mut regs, &mut h);
        assert_eq!(n, 7 + 10); // MBAP + (fc + bytecount + 4 regs)
        assert_eq!(&out[0..2], &[0x12, 0x34]);
        assert_eq!(&out[2..4], &[0, 0]);
        assert_eq!(&out[4..6], &[0, 11]); // len = unit + fc + 9 data bytes
        assert_eq!(out[6], 1);
        assert_eq!(out[7], 3);
        assert_eq!(out[8], 8);
        assert_eq!(&out[9..17], &[0, 192, 0, 168, 0, 12, 0, 101]);
        let _ = HOLDING_IP_OCTET1_IDX;
    }

    #[test]
    fn proto_nonzero_server_failure() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MBTCP_ADU_TX_MAX];
        let req = [
            0x00, 0x01, 0x00, 0x01, 0x00, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01,
        ];
        let n = mbtcp_adu_process(&req, &mut out, &mut srv, &mut regs, &mut h);
        assert_eq!(n, 9);
        assert_eq!(out[7], 0x83);
        assert_eq!(out[8], 0x04);
        assert_eq!(&out[2..4], &[0x00, 0x01]); // proto echoed
    }

    #[test]
    fn broadcast_executes_side_effects_no_reply() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MBTCP_ADU_TX_MAX];
        // unit=0 FC06 write DO=0x55
        let req = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x55,
        ];
        let n = mbtcp_adu_process(&req, &mut out, &mut srv, &mut regs, &mut h);
        assert_eq!(n, 0);
        assert_eq!(regs.get_holding(0x00), 0x55); // side effect happened
        assert_eq!(srv.diag_get(MbDiag::NoResp), 1);
    }

    #[test]
    fn truncated_pdu_silent() {
        let mut srv = MbServer::new();
        let mut regs = RegMap::new(0x0300);
        let mut h = NoHooks;
        let mut out = [0u8; MBTCP_ADU_TX_MAX];
        // FC03 with dlen 2 (not 4) -> decoder silent -> NO_RESP
        let req = [0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x03, 0x00, 0x00];
        let n = mbtcp_adu_process(&req, &mut out, &mut srv, &mut regs, &mut h);
        assert_eq!(n, 0);
        assert_eq!(srv.diag_get(MbDiag::NoResp), 1);
        // short frame < 8B -> silent, no counters
        let n = mbtcp_adu_process(&req[..6], &mut out, &mut srv, &mut regs, &mut h);
        assert_eq!(n, 0);
    }
}
