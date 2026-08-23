//! Global register-map state + hook implementations bridging proto to firmware.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use io_edge_hub_proto::mb_server::MbServer;
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

/// Shared PDU server (global diagnostics, like mb_server.c's static counters).
pub static MB_SERVER: Mutex<CriticalSectionRawMutex, RefCell<MbServer>> =
    Mutex::new(RefCell::new(MbServer::new()));

/// Delayed-reboot request (set_reboot_status): reboot a moment after the
/// triggering response reaches the wire; executed by the heartbeat task.
static REBOOT_AT: Mutex<CriticalSectionRawMutex, RefCell<Option<u64>>> =
    Mutex::new(RefCell::new(None));

pub fn set_reboot_status(on: bool) {
    let v = if on { Some(embassy_time::Instant::now().as_millis() as u64 + 250) } else { None };
    critical_section::with(|_cs| {
        REBOOT_AT.lock(|r| *r.borrow_mut() = v);
    });
}

/// Called from the heartbeat loop; performs the pending delayed reboot.
pub fn reboot_due() {
    let now = embassy_time::Instant::now().as_millis() as u64;
    let due = critical_section::with(|_cs| {
        REBOOT_AT.lock(|r| {
            let mut g = r.borrow_mut();
            match *g {
                Some(t) if now >= t => {
                    *g = None;
                    true
                }
                _ => false,
            }
        })
    });
    if due {
        crate::log::wrn("delayed reboot");
        reboot::cold();
    }
}

/// RegHooks bridging into real peripherals. DO GPIO wiring landed in M2;
/// history/persistence land in M3 (queued to the storage task).
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
        crate::storage::QUEUE.try_send(crate::storage::StorageCmd::CfgSave).ok();
    }

    fn history_enable_write(&mut self, en: bool) {
        if !en {
            // disable: close the file, keep the name for continuation
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::CloseKeepName)
                .ok();
        }
    }

    fn history_sync(&mut self) {
        crate::storage::QUEUE.try_send(crate::storage::StorageCmd::Sync).ok();
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
        crate::storage::QUEUE
            .try_send(crate::storage::StorageCmd::CfgEraseAll)
            .ok();
    }
}
