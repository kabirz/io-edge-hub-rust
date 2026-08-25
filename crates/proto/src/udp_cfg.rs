//! UDP config command layer, ported 1:1 from src/net/udp_cfg.c.
//!
//! Known commands always reply (invalid input/short length -> ok=0);
//! unknown commands (including 0x01-0x06 fw upgrade, owned by the fw layer)
//! return 0 = stay silent.
//!
//! Reboot contract: this layer only erases config -> writes reply -> sets the
//! pending flag; the transport must send the reply FIRST, then flush history
//! and reboot (see [`UdpCfgState::reboot_pending`]).

use crate::bytes;
use crate::regmap::{
    ip_addr_valid, RegHooks, RegMap, HOLDING_IP_OCTET1_IDX, HOLDING_RS485_BAUDRATE_IDX,
    HOLDING_SLAVE_ID_IDX,
};

pub const UDP_CFG_PORT: u16 = 8600;
pub const UDP_CFG_BCAST_PORT: u16 = UDP_CFG_PORT + 1;

pub const UDP_CMD_START: u8 = 0x01;
pub const UDP_CMD_DATA: u8 = 0x02;
pub const UDP_CMD_END: u8 = 0x03;
pub const UDP_CMD_GET_VERSION: u8 = 0x04;
pub const UDP_CMD_REBOOT: u8 = 0x05;
pub const UDP_CMD_DATA_V2: u8 = 0x06;
pub const UDP_CMD_SET_IP: u8 = 0x10;
pub const UDP_CMD_GET_IP: u8 = 0x11;
pub const UDP_CMD_SET_MODBUS: u8 = 0x12;
pub const UDP_CMD_GET_MODBUS: u8 = 0x13;
pub const UDP_CMD_SET_TIME: u8 = 0x14;
pub const UDP_CMD_GET_KEYHASH: u8 = 0x15;
pub const UDP_CMD_FACTORY_RESET: u8 = 0x19;

/// Firmware version pieces for the GET_VERSION reply "vM.m.p_git6".
/// `keyhash` serves the GET_KEYHASH (0x15) reply: upgrade hosts fetch the
/// device's signing-key fingerprint over the same UDP channel they upgrade
/// through, so rotating the key only touches the firmware.
#[derive(Clone, Copy)]
pub struct UdpVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
    pub git: &'static [u8; 6],
    pub keyhash: &'static [u8; 32],
}

/// Two-step FACTORY_RESET / REBOOT pending state (transport-visible).
#[derive(Debug, Clone)]
pub struct UdpCfgState {
    factory_pending_ms: u32,
    factory_confirmed: bool,
    factory_reboot_pending: bool,
    reboot_pending: bool,
}

impl Default for UdpCfgState {
    fn default() -> Self {
        Self::new()
    }
}

impl UdpCfgState {
    pub const fn new() -> Self {
        Self {
            factory_pending_ms: 0,
            factory_confirmed: false,
            factory_reboot_pending: false,
            reboot_pending: false,
        }
    }

    /// Power-cycle reset of all pending state.
    pub fn reset_pending(&mut self) {
        *self = Self::new();
    }

    /// Transport polls this AFTER the reply is on the wire; true means
    /// history_sync + cold reboot must happen now.
    pub fn take_reboot_pending(&mut self) -> bool {
        let p = self.factory_reboot_pending || self.reboot_pending;
        self.factory_reboot_pending = false;
        self.reboot_pending = false;
        p
    }

    pub fn reboot_pending(&self) -> bool {
        self.factory_reboot_pending || self.reboot_pending
    }

    fn factory_reset(
        &mut self,
        reply: &mut [u8],
        cmd: u8,
        now_ms: u32,
        cfg: &mut impl CfgHooks,
    ) -> usize {
        if !self.factory_confirmed {
            if now_ms.wrapping_sub(self.factory_pending_ms) > 5000 {
                // first command (or >5s since last): record time, await confirm
                self.factory_pending_ms = now_ms;
                return reply_ok(reply, cmd, 0);
            }
            self.factory_confirmed = true;
        }
        cfg.config_erase_all();
        self.factory_confirmed = false;
        self.factory_reboot_pending = true;
        reply_ok(reply, cmd, 1)
    }
}

/// Config-store side of the UDP layer (erase on factory reset).
pub trait CfgHooks {
    fn config_erase_all(&mut self);
}

pub struct NoCfg;
impl CfgHooks for NoCfg {
    fn config_erase_all(&mut self) {}
}

fn reply_ok(reply: &mut [u8], cmd: u8, ok: u8) -> usize {
    if reply.len() < 2 {
        return 0;
    }
    reply[0] = cmd;
    reply[1] = ok;
    2
}

/// Dispatch one UDP command. Returns reply length (0 = stay silent).
pub fn udp_app_cmd(
    cmd: u8,
    data: &[u8],
    reply: &mut [u8],
    regs: &mut RegMap,
    hooks: &mut impl RegHooks,
    cfg: &mut impl CfgHooks,
    st: &mut UdpCfgState,
    now_ms: u32,
    ver: &UdpVersion,
) -> usize {
    match cmd {
        UDP_CMD_SET_IP => {
            let mut ok = 0;
            if data.len() >= 4 && ip_addr_valid(data[0], data[1], data[2], data[3]) {
                let _ = regs.update_holding(HOLDING_IP_OCTET1_IDX as u16, data[0] as u16);
                let _ = regs.update_holding((HOLDING_IP_OCTET1_IDX + 1) as u16, data[1] as u16);
                let _ = regs.update_holding((HOLDING_IP_OCTET1_IDX + 2) as u16, data[2] as u16);
                let _ = regs.update_holding((HOLDING_IP_OCTET1_IDX + 3) as u16, data[3] as u16);
                hooks.holding_save();
                ok = 1;
            }
            reply_ok(reply, cmd, ok) // invalid input still always replies
        }

        UDP_CMD_GET_IP => {
            if reply.len() < 5 {
                return 0;
            }
            let ip = regs.ip_octets();
            reply[0] = cmd;
            reply[1..5].copy_from_slice(&ip);
            5
        }

        UDP_CMD_SET_MODBUS => {
            // slave_id (1B) + rs485_baud (BE16); takes effect after reboot
            let mut ok = 0;
            if data.len() >= 3 {
                let _ = regs.update_holding(HOLDING_SLAVE_ID_IDX as u16, data[0] as u16);
                let _ = regs.update_holding(
                    HOLDING_RS485_BAUDRATE_IDX as u16,
                    bytes::get_be16(&data[1..3]),
                );
                hooks.holding_save();
                ok = 1;
            }
            reply_ok(reply, cmd, ok) // short length still always replies
        }

        UDP_CMD_GET_MODBUS => {
            if reply.len() < 4 {
                return 0;
            }
            reply[0] = cmd;
            reply[1] = regs.get_holding(HOLDING_SLAVE_ID_IDX as u16) as u8;
            bytes::put_be16(
                regs.get_holding(HOLDING_RS485_BAUDRATE_IDX as u16),
                &mut reply[2..4],
            );
            4
        }

        UDP_CMD_SET_TIME => {
            let mut ok = 0;
            if data.len() >= 4 {
                ok = hooks.set_timestamp(bytes::get_be32(data)) as u8;
            }
            reply_ok(reply, cmd, ok) // short length still always replies
        }

        UDP_CMD_GET_VERSION => {
            // "vM.m.p_git6" without trailing NUL
            if reply.len() < 13 {
                return 0;
            }
            reply[0] = cmd;
            reply[1] = b'v';
            reply[2] = b'0' + ver.major.min(9);
            reply[3] = b'.';
            reply[4] = b'0' + ver.minor.min(9);
            reply[5] = b'.';
            reply[6] = b'0' + ver.patch.min(9);
            reply[7] = b'_';
            reply[8..14].copy_from_slice(ver.git);
            14
        }

        UDP_CMD_REBOOT => {
            st.reboot_pending = true;
            reply_ok(reply, cmd, 1)
        }

        // GET_KEYHASH: reply [cmd][keyhash 32B] — hosts upgrade over this
        // same channel, so they fetch the fingerprint here (not via HTTP)
        UDP_CMD_GET_KEYHASH => {
            if reply.len() < 1 + ver.keyhash.len() {
                return 0;
            }
            reply[0] = cmd;
            reply[1..33].copy_from_slice(ver.keyhash);
            33
        }

        UDP_CMD_FACTORY_RESET => st.factory_reset(reply, cmd, now_ms, cfg),

        _ => 0, // unknown (incl. 0x01-0x06 fw upgrade): silent
    }
}

/// Cross-subnet whitelist: GET_IP only (network discovery).
pub fn udp_cmd_bcast_allowed(cmd: u8) -> bool {
    cmd == UDP_CMD_GET_IP
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fw_upg;
    use crate::regmap::{RegHooks, RegMap};
    use std::cell::RefCell;

    struct MockHooks {
        ts_val: RefCell<Option<u32>>,
        ts_calls: RefCell<u32>,
        saves: RefCell<u32>,
    }
    impl RegHooks for MockHooks {
        fn set_timestamp(&mut self, t: u32) -> bool {
            *self.ts_val.borrow_mut() = Some(t);
            *self.ts_calls.borrow_mut() += 1;
            (946_684_800..=4_102_444_800).contains(&t)
        }
        fn holding_save(&mut self) {
            *self.saves.borrow_mut() += 1;
        }
    }

    struct MockCfg {
        erases: RefCell<u32>,
    }
    impl CfgHooks for MockCfg {
        fn config_erase_all(&mut self) {
            *self.erases.borrow_mut() += 1;
        }
    }

    fn ver() -> UdpVersion {
        UdpVersion {
            major: 0,
            minor: 3,
            patch: 0,
            git: b"538a9b",
            keyhash: &fw_upg::FW_KEYHASH,
        }
    }

    #[test]
    fn get_keyhash_replies_fingerprint() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();
        let n = udp_app_cmd(
            0x15,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(n, 33);
        assert_eq!(rep[0], 0x15);
        assert_eq!(&rep[1..33], &fw_upg::FW_KEYHASH[..]);
    }

    #[test]
    fn get_ip_defaults() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();
        let n = udp_app_cmd(
            0x11,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(n, 5);
        assert_eq!(&rep[..5], &[0x11, 192, 168, 12, 101]);
    }

    #[test]
    fn set_ip_valid_and_invalid() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        let n = udp_app_cmd(
            0x10,
            &[10, 20, 30, 40],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x10, 0x01]);
        assert_eq!(r.ip_octets(), [10, 20, 30, 40]);
        assert_eq!(*h.saves.borrow(), 1);

        for bad in [
            [192u8, 168, 12, 0],
            [192, 168, 12, 255],
            [127, 0, 0, 1],
            [224, 0, 0, 1],
            [240, 0, 0, 1],
        ] {
            let n = udp_app_cmd(
                0x10,
                &bad,
                &mut rep,
                &mut r,
                &mut h,
                &mut c,
                &mut st,
                0,
                &ver(),
            );
            assert_eq!(&rep[..n], &[0x10, 0x00]);
        }
        // short length still replies, regs untouched
        let n = udp_app_cmd(
            0x10,
            &[1, 2, 3],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x10, 0x00]);
        assert_eq!(r.ip_octets(), [10, 20, 30, 40]);

        let n = udp_app_cmd(
            0x11,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x11, 10, 20, 30, 40]);
    }

    #[test]
    fn set_get_modbus() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        let n = udp_app_cmd(
            0x12,
            &[5, 0x4B, 0x00],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x12, 0x01]);
        assert_eq!(r.get_holding(0x09), 5);
        assert_eq!(r.get_holding(0x08), 19200);

        let n = udp_app_cmd(
            0x12,
            &[7, 0x4B],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x12, 0x00]);
        assert_eq!(r.get_holding(0x09), 5);

        let n = udp_app_cmd(
            0x13,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x13, 5, 0x4B, 0x00]);
    }

    #[test]
    fn set_time_range_gate_via_hook() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        let mut t = [0u8; 4];
        bytes::put_be32(1_787_184_000, &mut t);
        let n = udp_app_cmd(
            0x14,
            &t,
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x14, 0x01]);
        assert_eq!(*h.ts_val.borrow(), Some(1_787_184_000));
        assert_eq!(*h.ts_calls.borrow(), 1);

        bytes::put_be32(946_684_799, &mut t);
        let n = udp_app_cmd(
            0x14,
            &t,
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x14, 0x00]);
        assert_eq!(*h.ts_calls.borrow(), 2);

        let n = udp_app_cmd(
            0x14,
            &t[..3],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x14, 0x00]);
        assert_eq!(*h.ts_calls.borrow(), 2);
    }

    #[test]
    fn get_version_format() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();
        let n = udp_app_cmd(
            0x04,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(n, 14);
        assert_eq!(&rep[..n], b"\x04v0.3.0_538a9b");
    }

    #[test]
    fn factory_reset_two_step() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            10_000,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x00]);
        assert!(!st.reboot_pending());
        assert_eq!(*c.erases.borrow(), 0);

        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            13_000,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x01]);
        assert_eq!(*c.erases.borrow(), 1);
        assert!(st.take_reboot_pending());
        assert!(!st.take_reboot_pending(), "flag cleared after take");
    }

    #[test]
    fn factory_reset_expires_after_5s() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        let _ = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            20_000,
            &ver(),
        );
        // +5001ms > 5000: re-arms the timer instead of confirming
        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            25_001,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x00]);
        assert!(!st.reboot_pending());
        // exactly +5000ms (not > 5000) from re-arm = confirm
        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            30_001,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x01]);
    }

    #[test]
    fn factory_reset_single_command_quirk_within_boot_5s() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();

        // uptime 4000ms: (4000 - 0) not > 5000 -> immediate confirm
        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            4_000,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x01]);
        assert!(st.take_reboot_pending());

        // boundary: exactly 5000ms still single-command; 5001ms restores two-step
        st.reset_pending();
        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            5_000,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x01]);
        st.reset_pending();
        let n = udp_app_cmd(
            0x19,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            5_001,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x19, 0x00]);
    }

    #[test]
    fn reboot_sets_pending_and_replies() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();
        let n = udp_app_cmd(
            0x05,
            &[],
            &mut rep,
            &mut r,
            &mut h,
            &mut c,
            &mut st,
            0,
            &ver(),
        );
        assert_eq!(&rep[..n], &[0x05, 0x01]);
        assert!(st.take_reboot_pending());
    }

    #[test]
    fn unknown_commands_silent() {
        let mut rep = [0u8; 64];
        let mut r = RegMap::new(0x0300);
        let mut h = MockHooks {
            ts_val: RefCell::new(None),
            ts_calls: RefCell::new(0),
            saves: RefCell::new(0),
        };
        let mut c = MockCfg {
            erases: RefCell::new(0),
        };
        let mut st = UdpCfgState::new();
        for cmd in [0x01u8, 0x06, 0x00, 0x0F, 0x16, 0x20, 0xFF] {
            assert_eq!(
                udp_app_cmd(
                    cmd,
                    &[1, 2, 3, 4, 5],
                    &mut rep,
                    &mut r,
                    &mut h,
                    &mut c,
                    &mut st,
                    0,
                    &ver()
                ),
                0
            );
        }
        assert!(!st.reboot_pending());
        assert_eq!(*c.erases.borrow(), 0);
    }

    #[test]
    fn bcast_whitelist_only_get_ip() {
        assert!(udp_cmd_bcast_allowed(0x11));
        for cmd in [0x10u8, 0x12, 0x13, 0x14, 0x19, 0x01, 0x00, 0xFF] {
            assert!(!udp_cmd_bcast_allowed(cmd));
        }
    }
}
