//! Register map, ported 1:1 from src/modbus/regmap.c.
//!
//! `RegMap` is the single source of truth for parameters and sampled data:
//! config persistence maps through it, sampling tasks write `input`, and all
//! write paths (Modbus FC06/16, web, UDP) go through [`RegMap::io_write_holding`]
//! so side effects stay consistent.

pub const DI_NUM: usize = 16;
pub const DO_NUM: usize = 8;
pub const AI_NUM: usize = 4;

pub const MODBUS_HOLDING_REGISTER_NUMBERS: usize = 18;
pub const MODBUS_INPUT_REGISTER_NUMBERS: usize = 6;

// input register indices
pub const INPUT_VER_IDX: usize = 0;
pub const INPUT_AI0_IDX: usize = 1;
pub const INPUT_AI1_IDX: usize = 2;
pub const INPUT_AI2_IDX: usize = 3;
pub const INPUT_AI3_IDX: usize = 4;
pub const INPUT_DI_IDX: usize = 5;

// holding register indices
pub const HOLDING_DO_IDX: usize = 0x00;
pub const HOLDING_DI_ENABLE_IDX: usize = 0x01;
pub const HOLDING_AI_ENABLE_IDX: usize = 0x02;
pub const HOLDING_DI_SAMPLE_MS_IDX: usize = 0x03;
pub const HOLDING_AI_SAMPLE_MS_IDX: usize = 0x04;
pub const HOLDING_HISTORY_ENABLE_IDX: usize = 0x05;
pub const HOLDING_CAN_ID_IDX: usize = 0x06;
pub const HOLDING_CAN_BAUDRATE_IDX: usize = 0x07;
pub const HOLDING_RS485_BAUDRATE_IDX: usize = 0x08;
pub const HOLDING_SLAVE_ID_IDX: usize = 0x09;
pub const HOLDING_IP_OCTET1_IDX: usize = 0x0A;
pub const HOLDING_IP_OCTET2_IDX: usize = 0x0B;
pub const HOLDING_IP_OCTET3_IDX: usize = 0x0C;
pub const HOLDING_IP_OCTET4_IDX: usize = 0x0D;
pub const HOLDING_TIMESTAMP_HI_IDX: usize = 0x0E;
pub const HOLDING_TIMESTAMP_LO_IDX: usize = 0x0F;
pub const HOLDING_CONFIG_SAVE_IDX: usize = 0x10;
pub const HOLDING_REBOOT_IDX: usize = 0x11;

/// Side effects invoked by holding-register writes (fake in host tests).
pub trait RegHooks {
    /// DO output + LED mirror (`reg & 0xFF`).
    fn set_do(&mut self, _val: u8) {}
    /// history enable toggle (0x05 write).
    fn history_enable_write(&mut self, _en: bool) {}
    /// RTC/system time set; range gate (2000..2100) is the hook's job.
    /// Returns whether the value was accepted.
    fn set_timestamp(&mut self, _ts: u32) -> bool {
        false
    }
    /// full parameter save (triggered by 0x10 write / UDP SET commands).
    fn holding_save(&mut self) {}
    /// flush history before reboot (called before `reboot_cold`).
    fn history_sync(&mut self) {}
    /// cold reboot; caller must not rely on it returning.
    fn reboot_cold(&mut self) {}
    /// live system time (seconds since epoch) for the timestamp registers.
    fn now_epoch(&self) -> u32 {
        0
    }
}

/// No-op hooks (defaults as in the C `io_hooks.h` fallbacks).
pub struct NoHooks;
impl RegHooks for NoHooks {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegMap {
    pub holding: [u16; MODBUS_HOLDING_REGISTER_NUMBERS],
    pub input: [u16; MODBUS_INPUT_REGISTER_NUMBERS],
}

impl RegMap {
    /// Compile-time defaults from regmap.c (IP 192.168.12.101, RS485 9600...).
    pub const fn new(version_reg: u16) -> Self {
        let mut holding = [0u16; MODBUS_HOLDING_REGISTER_NUMBERS];
        holding[HOLDING_DI_ENABLE_IDX] = 0xFFFF;
        holding[HOLDING_AI_ENABLE_IDX] = 0x000F;
        holding[HOLDING_DI_SAMPLE_MS_IDX] = 200;
        holding[HOLDING_AI_SAMPLE_MS_IDX] = 200;
        holding[HOLDING_CAN_ID_IDX] = 0x0111;
        holding[HOLDING_CAN_BAUDRATE_IDX] = 250;
        holding[HOLDING_RS485_BAUDRATE_IDX] = 9600;
        holding[HOLDING_SLAVE_ID_IDX] = 1;
        holding[HOLDING_IP_OCTET1_IDX] = 192;
        holding[HOLDING_IP_OCTET2_IDX] = 168;
        holding[HOLDING_IP_OCTET3_IDX] = 12;
        holding[HOLDING_IP_OCTET4_IDX] = 101;
        let mut input = [0u16; MODBUS_INPUT_REGISTER_NUMBERS];
        input[INPUT_VER_IDX] = version_reg;
        Self { holding, input }
    }

    pub fn get_holding(&self, addr: u16) -> u16 {
        self.holding.get(addr as usize).copied().unwrap_or(0)
    }

    /// FC03-style read: timestamp registers return live time.
    pub fn io_read_holding(&self, addr: u16, hooks: &impl RegHooks) -> u16 {
        match addr as usize {
            HOLDING_TIMESTAMP_HI_IDX => (hooks.now_epoch() >> 16) as u16,
            HOLDING_TIMESTAMP_LO_IDX => hooks.now_epoch() as u16,
            _ => self.get_holding(addr),
        }
    }

    /// Raw set without side effects (sampling / UDP handler internal use).
    pub fn update_holding(&mut self, addr: u16, reg: u16) -> Result<(), ()> {
        *self.holding.get_mut(addr as usize).ok_or(())? = reg;
        Ok(())
    }

    pub fn get_input(&self, addr: u16) -> u16 {
        self.input.get(addr as usize).copied().unwrap_or(0)
    }

    pub fn update_input(&mut self, addr: u16, reg: u16) -> Result<(), ()> {
        *self.input.get_mut(addr as usize).ok_or(())? = reg;
        Ok(())
    }

    /// Holding write with side effects (same semantics as FC06/FC16 writes).
    /// Same-value writes return early and skip ALL side effects.
    pub fn io_write_holding(
        &mut self,
        addr: u16,
        reg: u16,
        hooks: &mut impl RegHooks,
    ) -> Result<(), ()> {
        let a = *self.holding.get(addr as usize).ok_or(())?;
        if a == reg {
            return Ok(());
        }
        self.holding[addr as usize] = reg;
        match addr as usize {
            HOLDING_DO_IDX => hooks.set_do(reg as u8),
            HOLDING_SLAVE_ID_IDX => {} // takes effect after reboot
            HOLDING_HISTORY_ENABLE_IDX => hooks.history_enable_write(reg != 0),
            HOLDING_TIMESTAMP_LO_IDX => {
                let ts = ((self.holding[HOLDING_TIMESTAMP_HI_IDX] as u32) << 16) | reg as u32;
                hooks.set_timestamp(ts);
            }
            HOLDING_CONFIG_SAVE_IDX => {
                self.holding[addr as usize] = 0;
                hooks.holding_save();
            }
            HOLDING_REBOOT_IDX => {
                self.holding[addr as usize] = 0;
                if reg != 0 {
                    hooks.history_sync();
                    hooks.reboot_cold();
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Single DO bit write (RMW, FC05 semantics).
    pub fn io_write_do_bit(
        &mut self,
        bit: u16,
        state: bool,
        hooks: &mut impl RegHooks,
    ) -> Result<(), ()> {
        if bit as usize >= DO_NUM {
            return Err(());
        }
        let mut val = self.holding[HOLDING_DO_IDX];
        if state {
            val |= 1 << bit;
        } else {
            val &= !(1 << bit);
        }
        hooks.set_do(val as u8);
        self.holding[HOLDING_DO_IDX] = val & 0xFF;
        Ok(())
    }

    /// Coil (FC01) maps to DO bits.
    pub fn io_coil_rd(&self, addr: u16) -> Result<bool, ()> {
        if addr as usize >= DO_NUM {
            return Err(());
        }
        Ok((self.holding[HOLDING_DO_IDX] >> addr) & 1 != 0)
    }

    /// Discrete input (FC02) maps to DI bits.
    pub fn io_discrete_rd(&self, addr: u16) -> Result<bool, ()> {
        if addr as usize >= DI_NUM {
            return Err(());
        }
        Ok((self.input[INPUT_DI_IDX] >> addr) & 1 != 0)
    }

    pub fn ip_octets(&self) -> [u8; 4] {
        [
            self.holding[HOLDING_IP_OCTET1_IDX] as u8,
            self.holding[HOLDING_IP_OCTET1_IDX + 1] as u8,
            self.holding[HOLDING_IP_OCTET1_IDX + 2] as u8,
            self.holding[HOLDING_IP_OCTET1_IDX + 3] as u8,
        ]
    }
}

/// IPv4 validity gate from regmap.c: last octet not 0/0xFF, first octet not
/// 0/127/multicast(224-239)/reserved(>=240).
pub fn ip_addr_valid(a: u8, _b: u8, _c: u8, d: u8) -> bool {
    if d == 0 || d == 0xFF {
        return false;
    }
    if a == 0 || a == 127 || a >= 224 {
        return false;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    #[derive(Default)]
    struct Mock {
        do_val: RefCell<u8>,
        saves: RefCell<u32>,
        syncs: RefCell<u32>,
        reboots: RefCell<u32>,
        ts: RefCell<Option<u32>>,
        epoch: u32,
    }
    impl RegHooks for Mock {
        fn set_do(&mut self, v: u8) {
            *self.do_val.borrow_mut() = v;
        }
        fn set_timestamp(&mut self, ts: u32) -> bool {
            *self.ts.borrow_mut() = Some(ts);
            (946_684_800..=4_102_444_800).contains(&ts)
        }
        fn holding_save(&mut self) {
            *self.saves.borrow_mut() += 1;
        }
        fn history_sync(&mut self) {
            *self.syncs.borrow_mut() += 1;
        }
        fn reboot_cold(&mut self) {
            *self.reboots.borrow_mut() += 1;
        }
        fn now_epoch(&self) -> u32 {
            self.epoch
        }
    }

    fn regs() -> RegMap {
        RegMap::new(0x0300)
    }

    #[test]
    fn defaults_match_c() {
        let r = regs();
        assert_eq!(r.holding[HOLDING_DI_ENABLE_IDX], 0xFFFF);
        assert_eq!(r.holding[HOLDING_AI_ENABLE_IDX], 0x000F);
        assert_eq!(r.holding[HOLDING_DI_SAMPLE_MS_IDX], 200);
        assert_eq!(r.holding[HOLDING_CAN_ID_IDX], 0x0111);
        assert_eq!(r.holding[HOLDING_RS485_BAUDRATE_IDX], 9600);
        assert_eq!(r.holding[HOLDING_SLAVE_ID_IDX], 1);
        assert_eq!(r.ip_octets(), [192, 168, 12, 101]);
        assert_eq!(r.get_input(INPUT_VER_IDX as u16), 0x0300);
    }

    #[test]
    fn do_write_drives_output() {
        let mut r = regs();
        let mut h = Mock::default();
        r.io_write_holding(HOLDING_DO_IDX as u16, 0xA5, &mut h)
            .unwrap();
        assert_eq!(*h.do_val.borrow(), 0xA5);
        assert_eq!(r.holding[HOLDING_DO_IDX], 0xA5);
    }

    #[test]
    fn same_value_write_skips_side_effects() {
        let mut r = regs();
        let mut h = Mock::default();
        r.io_write_holding(HOLDING_CONFIG_SAVE_IDX as u16, 0, &mut h)
            .unwrap();
        assert_eq!(*h.saves.borrow(), 0);
    }

    #[test]
    fn config_save_zeros_and_saves() {
        let mut r = regs();
        let mut h = Mock::default();
        r.io_write_holding(HOLDING_CONFIG_SAVE_IDX as u16, 1, &mut h)
            .unwrap();
        assert_eq!(*h.saves.borrow(), 1);
        assert_eq!(r.holding[HOLDING_CONFIG_SAVE_IDX], 0);
    }

    #[test]
    fn reboot_write_syncs_then_reboots() {
        let mut r = regs();
        let mut h = Mock::default();
        r.io_write_holding(HOLDING_REBOOT_IDX as u16, 1, &mut h)
            .unwrap();
        assert_eq!(*h.syncs.borrow(), 1);
        assert_eq!(*h.reboots.borrow(), 1);
        assert_eq!(r.holding[HOLDING_REBOOT_IDX], 0);
    }

    #[test]
    fn timestamp_write_combines_hi_lo() {
        let mut r = regs();
        let mut h = Mock::default();
        r.update_holding(HOLDING_TIMESTAMP_HI_IDX as u16, 0x6A9C)
            .unwrap();
        r.io_write_holding(HOLDING_TIMESTAMP_LO_IDX as u16, 0xB400, &mut h)
            .unwrap();
        assert_eq!(*h.ts.borrow(), Some(0x6A9CB400));
    }

    #[test]
    fn timestamp_read_is_live() {
        let mut r = regs();
        let mut h = Mock::default();
        h.epoch = 0x1234_5678;
        assert_eq!(
            r.io_read_holding(HOLDING_TIMESTAMP_HI_IDX as u16, &h),
            0x1234
        );
        assert_eq!(
            r.io_read_holding(HOLDING_TIMESTAMP_LO_IDX as u16, &h),
            0x5678
        );
    }

    #[test]
    fn do_bit_write_rmw() {
        let mut r = regs();
        let mut h = Mock::default();
        r.io_write_holding(HOLDING_DO_IDX as u16, 0x00, &mut h)
            .unwrap();
        r.io_write_do_bit(3, true, &mut h).unwrap();
        assert_eq!(*h.do_val.borrow(), 0x08);
        assert!(r.io_coil_rd(3).unwrap());
        assert!(!r.io_coil_rd(2).unwrap());
        assert!(r.io_write_do_bit(8, true, &mut h).is_err());
        assert!(r.io_coil_rd(8).is_err());
    }

    #[test]
    fn discrete_read_maps_di() {
        let mut r = regs();
        r.update_input(INPUT_DI_IDX as u16, 0x8001).unwrap();
        assert!(r.io_discrete_rd(0).unwrap());
        assert!(r.io_discrete_rd(15).unwrap());
        assert!(!r.io_discrete_rd(7).unwrap());
        assert!(r.io_discrete_rd(16).is_err());
    }

    #[test]
    fn out_of_range_rejected() {
        let mut r = regs();
        let mut h = Mock::default();
        assert!(r.io_write_holding(18, 1, &mut h).is_err());
        assert!(r.update_holding(18, 1).is_err());
        assert_eq!(r.get_holding(18), 0);
    }

    #[test]
    fn ip_validity() {
        assert!(ip_addr_valid(192, 168, 12, 101));
        assert!(!ip_addr_valid(192, 168, 12, 0));
        assert!(!ip_addr_valid(192, 168, 12, 255));
        assert!(!ip_addr_valid(127, 0, 0, 1));
        assert!(!ip_addr_valid(224, 0, 0, 1));
        assert!(!ip_addr_valid(240, 0, 0, 1));
        assert!(!ip_addr_valid(0, 1, 2, 3));
        assert!(ip_addr_valid(10, 20, 30, 40));
    }
}
