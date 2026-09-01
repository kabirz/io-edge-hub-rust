//! W5500 hardware-socket (TOE) networking: UDP :8600 config/upgrade server
//! plus link monitoring, on the branch that drops the smoltcp MACRAW path
//! (and with it HTTP/FTP). One net task polls sockets at 2 ms — the C
//! firmware's select loop in async clothes. Modbus TCP :502 is serviced by
//! the same task through crate::mbtcp::MbSock on sockets 1/2.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Ticker};

use io_edge_hub_proto::regmap::HOLDING_IP_OCTET1_IDX;
use io_edge_hub_proto::udp_cfg::{
    udp_app_cmd, udp_cmd_bcast_allowed, UdpVersion, UDP_CFG_BCAST_PORT, UDP_CFG_PORT,
};

use crate::appstate::{version, Cfg, Hooks, REGS, UDP_STATE};
use crate::w5500::{W5500Pins, SOCK_MB1, SOCK_MB2, SOCK_UDP, W5500};

/// UID-derived MAC: Wiznet OUI + XOR-fold of the 96-bit device UID.
pub fn derive_mac_from_uid() -> [u8; 6] {
    let uid = embassy_stm32::uid::uid();
    [
        0x00,
        0x08,
        0xDC,
        uid[0] ^ uid[3] ^ uid[6] ^ uid[9],
        uid[1] ^ uid[4] ^ uid[7] ^ uid[10],
        uid[2] ^ uid[5] ^ uid[8] ^ uid[11],
    ]
}

// MAC exposed to the web info JSON
pub static CUR_MAC: Mutex<CriticalSectionRawMutex, RefCell<[u8; 6]>> =
    Mutex::new(RefCell::new([0; 6]));

pub fn current_mac() -> [u8; 6] {
    critical_section::with(|_cs| {
        CUR_MAC.lock(|m| {
            let g = m.borrow();
            let v: [u8; 6] = *g;
            v
        })
    })
}

/// Link status refreshed by the net task's netmon poll (PHYCFGR).
pub static LINK_UP: Mutex<CriticalSectionRawMutex, RefCell<bool>> = Mutex::new(RefCell::new(false));

pub fn net_link_up() -> bool {
    critical_section::with(|_cs| LINK_UP.lock(|b| *b.borrow()))
}

/// Sole owner of the W5500: chip init, UDP :8600 service, both Modbus TCP
/// sockets, and the 500 ms netmon link poll. UDP upgrade commands (NOR
/// writes, millisecond-scale) run inline exactly as they did in the smoltcp
/// udp task; the socket buffers absorb the burst meanwhile.
#[embassy_executor::task]
pub async fn net_task(pins: W5500Pins) {
    crate::stackmark::probe("net");
    let mut w = match W5500::new(pins).await {
        Some(w) => w,
        None => {
            crate::log::err("net: w5500 init FAILED");
            loop {
                embassy_time::Timer::after_millis(1000).await;
            }
        }
    };

    // static IP snapshot from cfg (same source as the C firmware)
    let ip = critical_section::with(|_cs| {
        REGS.lock(|r| {
            let r = r.borrow();
            [
                r.holding[HOLDING_IP_OCTET1_IDX] as u8,
                r.holding[HOLDING_IP_OCTET1_IDX + 1] as u8,
                r.holding[HOLDING_IP_OCTET1_IDX + 2] as u8,
                r.holding[HOLDING_IP_OCTET1_IDX + 3] as u8,
            ]
        })
    });
    let mac = derive_mac_from_uid();
    w.set_netconf(&mac, &ip, &[255, 255, 255, 0], &[ip[0], ip[1], ip[2], 1]);
    critical_section::with(|_cs| CUR_MAC.lock(|m| *m.borrow_mut() = mac));

    w.udp_open(SOCK_UDP, UDP_CFG_PORT);
    w.tcp_listen(SOCK_MB1, crate::mbtcp::MBTCP_PORT);
    w.tcp_listen(SOCK_MB2, crate::mbtcp::MBTCP_PORT);
    crate::log::inf("net: w5500 toe up, udp 8600, mbtcp 502");

    let ver = UdpVersion {
        major: version::FW_MAJOR,
        minor: version::FW_MINOR,
        patch: version::FW_PATCH,
        git: version::FW_GIT,
    };

    let mut mb1 = crate::mbtcp::MbSock::new(SOCK_MB1, "mbtcp1");
    let mut mb2 = crate::mbtcp::MbSock::new(SOCK_MB2, "mbtcp2");

    // locals live in the task pool storage (statics), not on the shared stack
    let mut rx = [0u8; 1500];

    let mut ticker = Ticker::every(Duration::from_millis(2));
    let mut tick: u32 = 0;
    loop {
        crate::stackmark::probe("net");
        ticker.next().await;
        tick = tick.wrapping_add(1);

        // netmon: 500ms link poll (heartbeat reads net_link_up for the
        // link-down DO-all-off rule)
        if tick % 250 == 0 {
            let up = w.link_up();
            critical_section::with(|_cs| LINK_UP.lock(|b| *b.borrow_mut() = up));
        }

        // UDP :8600 — drain every pending datagram
        while let Some((n, ip, port)) = w.udp_recv(SOCK_UDP, &mut rx) {
            serve_udp(&mut w, &rx[..n], ip, port, &ver).await;
        }

        // Modbus TCP: two hardware sockets
        mb1.poll(&mut w);
        mb2.poll(&mut w);
    }
}

/// One datagram: the udp_task.c command surface, transport swapped for the
/// hardware UDP socket (same-subnet rules, fw channel, diagnostics, reboot
/// contract preserved verbatim).
async fn serve_udp(w: &mut W5500, rx: &[u8], ip: [u8; 4], port: u16, ver: &UdpVersion) {
    if rx.is_empty() {
        return;
    }
    let cmd = rx[0];

    // cross-subnet whitelist: GET_IP only
    let same = same_subnet24(&ip);
    if !same && !udp_cmd_bcast_allowed(cmd) {
        crate::log::wrn("udp: drop cross-subnet cmd");
        return;
    }

    // debug: cmd 0xFA dumps shell RX counters (dma-consumed / processed)
    if cmd == 0xFA {
        let mut rep = [0u8; 9];
        rep[0] = 0xFA;
        rep[1..5].copy_from_slice(
            &crate::shell::RX_COUNT
                .load(core::sync::atomic::Ordering::Relaxed)
                .to_le_bytes(),
        );
        rep[5..9].copy_from_slice(
            &crate::shell::RX_GOT
                .load(core::sync::atomic::Ordering::Relaxed)
                .to_le_bytes(),
        );
        w.udp_send_to(SOCK_UDP, &rep, &ip, port);
        return;
    }

    // debug: cmd 0xFB dumps fw upgrade diagnostics (finish failure cause)
    if cmd == 0xFB {
        let d = critical_section::with(|_cs| crate::fw::FW_DBG.lock(|f| *f.borrow()));
        let mut rep = [0u8; 65];
        rep[0] = 0xFB;
        for (i, v) in d.iter().enumerate() {
            rep[1 + i * 4..5 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        w.udp_send_to(SOCK_UDP, &rep, &ip, port);
        return;
    }

    // firmware-upgrade channel 0x01/0x02/0x03/0x06, handled inline:
    // START's whole-slot erase (~1s) delays its reply and DATA_V2 page
    // writes are millisecond-scale; the socket buffers keep absorbing.
    if let Some((rep, rlen)) = fw_udp_cmd(cmd, &rx[1..]).await {
        if rlen > 0 {
            w.udp_send_to(SOCK_UDP, &rep[..rlen], &ip, port);
        }
        return;
    }

    // debug: cmd 0xFC dumps storage RPC state
    if cmd == 0xFC {
        let (dl, seq) = critical_section::with(|_cs| {
            crate::storage::FILE_DL.lock(|f| {
                let g = f.borrow();
                (
                    (g.open, g.eof, g.err, g.size, g.sent, g.chunk_len),
                    crate::storage::RPC_SEQ.load(core::sync::atomic::Ordering::Relaxed),
                )
            })
        });
        let mut rep = [0u8; 16];
        rep[0] = 0xFC;
        rep[1] = seq as u8;
        rep[2] = (seq >> 8) as u8;
        rep[3] = (seq >> 16) as u8;
        rep[4] = (seq >> 24) as u8;
        rep[5] = dl.0 as u8; // open
        rep[6] = dl.1 as u8; // eof
        rep[7] = dl.2 as u8; // err
        rep[8] = (dl.3 >> 24) as u8;
        rep[9] = (dl.3 >> 16) as u8;
        rep[10] = (dl.3 >> 8) as u8;
        rep[11] = dl.3 as u8;
        rep[12] = (dl.4 >> 24) as u8;
        rep[13] = (dl.4 >> 16) as u8;
        rep[14] = (dl.4 >> 8) as u8;
        rep[15] = dl.4 as u8;
        w.udp_send_to(SOCK_UDP, &rep, &ip, port);
        return;
    }

    let mut rep = [0u8; 64];
    let rlen = critical_section::with(|_cs| {
        REGS.lock(|r| {
            UDP_STATE.lock(|st| {
                let mut h = Hooks;
                let mut c = Cfg;
                udp_app_cmd(
                    cmd,
                    &rx[1..],
                    &mut rep,
                    &mut r.borrow_mut(),
                    &mut h,
                    &mut c,
                    &mut st.borrow_mut(),
                    now_ms32(),
                    ver,
                )
            })
        })
    });
    if rlen == 0 {
        return; // unknown command: silent
    }

    if same {
        w.udp_send_to(SOCK_UDP, &rep[..rlen], &ip, port);
    } else {
        // cross-subnet: directed broadcast reply on port+1
        w.udp_send_to(
            SOCK_UDP,
            &rep[..rlen],
            &[255, 255, 255, 255],
            UDP_CFG_BCAST_PORT,
        );
    }

    // reboot contract: the reply is on the wire -> flush history ->
    // 100ms -> reset INLINE. This task must go silent until the reset; a
    // deadline polled in the background leaves a ~200ms window where
    // GET_VERSION probes still get answered, so a host polling for
    // "back online" reads the pre-reboot image (stale uptime) and later
    // traffic dies mid-swap.
    let reboot_now =
        critical_section::with(|_cs| UDP_STATE.lock(|st| st.borrow_mut().take_reboot_pending()));
    if reboot_now {
        crate::log::wrn("udp: delayed reboot");
        // flush history before the swap reboot; the 100ms wait below
        // also yields so the storage task can run it
        crate::storage::QUEUE
            .try_send(crate::storage::StorageCmd::Sync)
            .ok();
        embassy_time::Timer::after_millis(100).await;
        crate::reboot::system_reset();
    }
}

/// Firmware-channel command handlers; returns (reply, len) when the command
/// was a firmware-channel one (None -> not ours, generic layer takes it).
async fn fw_udp_cmd(cmd: u8, payload: &[u8]) -> Option<([u8; 8], usize)> {
    match cmd {
        // START [size LE32][keyhash 32B?] -> [01][status][v2_chunk LE16]
        0x01 => {
            if payload.len() < 4 {
                return Some(([0; 8], 0)); // malformed: silent
            }
            let total = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
            let kh: Option<&[u8; 32]> = if payload.len() >= 4 + 32 {
                match payload[4..36].try_into() {
                    Ok(k) => Some(k),
                    Err(_) => None,
                }
            } else {
                None
            };
            let rc = crate::fw::start(total, kh);
            crate::log::inf("fwupg: start");
            let mut rep = [0u8; 8];
            rep[0] = 0x01;
            rep[1] = match rc {
                0 => 1,
                -2 => 2,
                _ => 0,
            };
            rep[2] = (1400u16 & 0xFF) as u8;
            rep[3] = ((1400u16 >> 8) & 0xFF) as u8;
            Some((rep, 4))
        }
        // DATA [data<=511] -> [02][received LE32]
        0x02 => {
            if payload.len() > 511 {
                return Some(([0; 8], 0));
            }
            crate::fw::write(payload);
            let mut rep = [0u8; 8];
            rep[0] = 0x02;
            rep[1..5].copy_from_slice(&crate::fw::received().to_le_bytes());
            Some((rep, 5))
        }
        // DATA_V2 [offset LE32][data<=1400] -> [06][expected LE32];
        // writes only when the offset matches the received count
        // (out-of-order/duplicates dropped; host does go-back-N).
        // Pages are committed one at a time with a yield in between so
        // the poll loop keeps servicing the other sockets.
        0x06 => {
            if payload.len() < 5 {
                return Some(([0; 8], 0));
            }
            let off = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
            if off == crate::fw::received() {
                let mut data = &payload[4..];
                while !data.is_empty() {
                    let n = data.len().min(256);
                    crate::fw::write(&data[..n]);
                    data = &data[n..];
                    if !data.is_empty() {
                        embassy_futures::yield_now().await;
                    }
                }
            }
            let mut rep = [0u8; 8];
            rep[0] = 0x06;
            rep[1..5].copy_from_slice(&crate::fw::received().to_le_bytes());
            Some((rep, 5))
        }
        // END [test u8][crc LE16] -> [03][ok]
        0x03 => {
            if payload.len() < 3 {
                return Some(([0; 8], 0));
            }
            let permanent = payload[0] == 0;
            let crc = u16::from_le_bytes([payload[1], payload[2]]);
            let mut ok = 0u8;
            if crate::fw::finish(Some(crc)) {
                if crate::fw::boot_set_pending(permanent) {
                    ok = 1;
                }
            }
            crate::log::inf("fwupg: end");
            Some(([0x03, ok, 0, 0, 0, 0, 0, 0], 2))
        }
        _ => None,
    }
}

fn now_ms32() -> u32 {
    embassy_time::Instant::now().as_ticks() as u32 / (embassy_time::TICK_HZ as u32 / 1000)
}

/// Sender shares our /24.
fn same_subnet24(addr: &[u8; 4]) -> bool {
    let local = critical_section::with(|_cs| {
        REGS.lock(|r| {
            let r = r.borrow();
            (
                r.holding[HOLDING_IP_OCTET1_IDX] as u8,
                r.holding[HOLDING_IP_OCTET1_IDX + 1] as u8,
                r.holding[HOLDING_IP_OCTET1_IDX + 2] as u8,
            )
        })
    });
    (addr[0], addr[1], addr[2]) == local
}
