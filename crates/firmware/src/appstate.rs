//! Global register-map state + hook implementations bridging proto to firmware.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use io_edge_hub_proto::regmap::{RegHooks, RegMap};
use io_edge_hub_proto::udp_cfg::{CfgHooks, UdpCfgState};

use crate::{reboot, systime};

pub mod version {
    include!(concat!(env!("OUT_DIR"), "/fw_version.rs"));
}

pub const VERSION_REG: u16 = ((version::FW_MAJOR as u16) << 12)
    | ((version::FW_MINOR as u16) << 8)
    | version::FW_PATCH as u16;

pub static REGS: Mutex<CriticalSectionRawMutex, RefCell<RegMap>> =
    Mutex::new(RefCell::new(RegMap::new(VERSION_REG)));

pub static UDP_STATE: Mutex<CriticalSectionRawMutex, RefCell<UdpCfgState>> =
    Mutex::new(RefCell::new(UdpCfgState::new()));

/// RegHooks bridging into real peripherals. DO GPIO wiring lands in M2;
/// history/persistence hooks in M3.
pub struct Hooks;

impl RegHooks for Hooks {
    fn set_do(&mut self, val: u8) {
        crate::io_gpio::set_do_led(val);
    }

    fn set_timestamp(&mut self, ts: u32) -> bool {
        let ok = systime::set_timestamp(ts);
        if ok {
            crate::log::inf("time set");
        }
        ok
    }

    fn holding_save(&mut self) {
        // M3: persist to config store on NOR
    }

    fn history_sync(&mut self) {
        // M3: flush history file
    }

    fn reboot_cold(&mut self) {
        reboot::cold();
    }

    fn now_epoch(&self) -> u32 {
        systime::now_epoch()
    }
}

pub struct Cfg;

impl CfgHooks for Cfg {
    fn config_erase_all(&mut self) {
        // M3: erase config slots on NOR
        crate::log::wrn("factory reset: config erase (no-op until M3)");
    }
}
