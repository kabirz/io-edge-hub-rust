//! W5500 MACRAW + embassy-net bring-up and the UDP :8600 config server
//! (transport layer of src/net/udp_task.c).

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack, StaticConfigV4};
use core::cell::RefCell;

use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::exti::{ExtiInput, InterruptHandler as ExtiIrqHandler};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use static_cell::StaticCell;

use io_edge_hub_proto::regmap::HOLDING_IP_OCTET1_IDX;
use io_edge_hub_proto::udp_cfg::{
    UDP_CFG_BCAST_PORT, UDP_CFG_PORT, UdpVersion, udp_app_cmd, udp_cmd_bcast_allowed,
};

use crate::appstate::{Cfg, Hooks, UDP_STATE, REGS, version};

use embassy_stm32::interrupt::typelevel::EXTI1 as EXTI1Irq;

bind_interrupts! {
    struct Irqs {
        DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
        DMA1_STREAM4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
        EXTI1 => ExtiIrqHandler<EXTI1Irq>;
    }
}

type SpiBus = Spi<'static, Async, spi::mode::Master>;
type W5500SpiDevice<'a> =
    embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<'a, NoopRawMutex, SpiBus, Output<'static>>;
type W5500Int = ExtiInput<'static, Async>;
type W5500Runner<'a> =
    embassy_net_wiznet::Runner<'a, embassy_net_wiznet::chip::W5500, W5500SpiDevice<'a>, W5500Int, Output<'static>>;

/// UID-derived MAC (main.c derive_mac_from_uid): Wiznet OUI + XOR-fold.
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

// accessors for the web info JSON (httpd.c reads these via w5500_macraw_)
pub static CUR_MAC: Mutex<CriticalSectionRawMutex, RefCell<[u8; 6]>> =
    Mutex::new(RefCell::new([0; 6]));

pub fn current_mac() -> [u8; 6] {
    critical_section::with(|_cs| CUR_MAC.lock(|m| {
        let g = m.borrow();
        let v: [u8; 6] = *g;
        v
    }))
}

/// Link status refreshed by the heartbeat netmon poll (link-up boolean only;
/// Stack itself is not Sync so it cannot live in a static).
pub static LINK_UP: Mutex<CriticalSectionRawMutex, RefCell<bool>> =
    Mutex::new(RefCell::new(false));

pub fn net_link_up() -> bool {
    critical_section::with(|_cs| LINK_UP.lock(|b| *b.borrow()))
}

pub struct NetPins {
    pub spi2: Peri<'static, peripherals::SPI2>,
    pub sck: Peri<'static, peripherals::PB13>,
    pub miso: Peri<'static, peripherals::PB14>,
    pub mosi: Peri<'static, peripherals::PB15>,
    pub cs: Peri<'static, peripherals::PB12>,
    pub int: Peri<'static, peripherals::PD1>,
    pub rst: Peri<'static, peripherals::PD0>,
    pub tx_dma: Peri<'static, peripherals::DMA1_CH4>,
    pub rx_dma: Peri<'static, peripherals::DMA1_CH3>,
}

pub async fn setup(spawner: &embassy_executor::Spawner, p: NetPins) -> &'static Stack<'static> {
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(21_000_000);
    let spi = Spi::new(p.spi2, p.sck, p.mosi, p.miso, p.tx_dma, p.rx_dma, Irqs, spi_cfg);
    let cs = Output::new(p.cs, Level::High, Speed::VeryHigh);
    let int = ExtiInput::new(p.int, unsafe { peripherals::EXTI1::steal() }, Pull::Up, Irqs);
    let rst = Output::new(p.rst, Level::High, Speed::Low);

    static SPI_BUS: StaticCell<AsyncMutex<NoopRawMutex, SpiBus>> = StaticCell::new();
    let bus = SPI_BUS.init(AsyncMutex::new(spi));
    let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(bus, cs);

    static STATE: StaticCell<embassy_net_wiznet::State<4, 4>> = StaticCell::new();
    let state = STATE.init(embassy_net_wiznet::State::new());
    crate::log::inf("net: spi up, w5500 init");
    let mac = derive_mac_from_uid();
    let (device, runner) = match embassy_net_wiznet::new::<_, _, embassy_net_wiznet::chip::W5500, _, _, _>(
        mac,
        state,
        spi_dev,
        int,
        rst,
    )
    .await
    {
        Ok(v) => v,
        Err(_e) => {
            crate::log::err("net: w5500 init FAILED");
            loop {
                embassy_time::Timer::after_millis(1000).await;
            }
        }
    };
    crate::log::inf("net: w5500 chip ok");

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
    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(Ipv4Address::new(ip[0], ip[1], ip[2], ip[3]), 24),
        gateway: Some(Ipv4Address::new(ip[0], ip[1], ip[2], 1)),
        dns_servers: Default::default(),
    });

    // 13 live sockets: udp 1 + mbtcp 3 + httpd 2 + ftp (3x ctrl+data) + ftp rejector
    static RES: StaticCell<embassy_net::StackResources<16>> = StaticCell::new();
    static STACK: StaticCell<Stack<'static>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        device,
        config,
        RES.init(embassy_net::StackResources::new()),
        0x6A9C_B400,
    );
    let stack: &'static Stack<'static> = STACK.init(stack);
    critical_section::with(|_cs| {
        CUR_MAC.lock(|m| *m.borrow_mut() = mac);
    });
    crate::log::inf("net: stack created");

    spawner.spawn(net_run_task(runner).expect("spawn w5500"));
    spawner.spawn(net_stack_task(net_runner).expect("spawn stack"));
    spawner.spawn(udp_task(*stack).expect("spawn udp"));
    stack
}

#[embassy_executor::task]
async fn net_run_task(runner: W5500Runner<'static>) {
    runner.run().await
}

#[embassy_executor::task]
async fn net_stack_task(mut runner: embassy_net::Runner<'static, embassy_net_driver_channel::Device<'static, 1514>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn udp_task(stack: Stack<'static>) {
    static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
    // V2 upgrade bursts arrive faster than they are written to NOR: a full
    // 8x1400 B window must fit. 16 K in CCRAM keeps the main-stack headroom.
    #[link_section = ".ccm.bss"]
    static RX_BUF: StaticCell<[u8; 16384]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut sock = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; 16]),
        RX_BUF.init([0u8; 16384]),
        TX_META.init([PacketMetadata::EMPTY; 8]),
        TX_BUF.init([0u8; 2048]),
    );
    if sock.bind(UDP_CFG_PORT).is_err() {
        crate::log::err("udp: bind 8600 failed");
        return;
    }
    crate::log::inf("udp: port 8600 listening");

    let ver = UdpVersion {
        major: version::FW_MAJOR,
        minor: version::FW_MINOR,
        patch: version::FW_PATCH,
        git: version::FW_GIT,
    };

    let mut rx = [0u8; 1500];
    loop {
        let (n, meta) = match sock.recv_from(&mut rx).await {
            Ok(v) => v,
            Err(_) => continue,
        };
        if n == 0 {
            continue;
        }
        let cmd = rx[0];

        // cross-subnet whitelist: GET_IP only
        let same = same_subnet24(&meta.endpoint.addr);
        if !same && !udp_cmd_bcast_allowed(cmd) {
            crate::log::wrn("udp: drop cross-subnet cmd");
            continue;
        }

        // debug: cmd 0xFA dumps shell RX counters (dma-consumed / processed)
        if cmd == 0xFA {
            let mut rep = [0u8; 9];
            rep[0] = 0xFA;
            rep[1..5].copy_from_slice(&crate::shell::RX_COUNT.load(core::sync::atomic::Ordering::Relaxed).to_le_bytes());
            rep[5..9].copy_from_slice(&crate::shell::RX_GOT.load(core::sync::atomic::Ordering::Relaxed).to_le_bytes());
            sock.send_to(&rep, meta.endpoint).await.ok();
            continue;
        }

        // debug: cmd 0xFB dumps fw upgrade diagnostics (finish failure cause)
        if cmd == 0xFB {
            let d = critical_section::with(|_cs| crate::fw::FW_DBG.lock(|f| *f.borrow()));
            let mut rep = [0u8; 65];
            rep[0] = 0xFB;
            for (i, v) in d.iter().enumerate() {
                rep[1 + i * 4..5 + i * 4].copy_from_slice(&v.to_le_bytes());
            }
            sock.send_to(&rep, meta.endpoint).await.ok();
            continue;
        }

        // firmware-upgrade channel (fw_udp.c): 0x01/0x02/0x03/0x06.
        // Synchronous here like the C worker's effect on timing: START's
        // whole-slot erase (~1s) delays its reply, DATA_V2 page writes are
        // millisecond-scale; the net stack keeps polling meanwhile.
        if let Some((rep, rlen)) = fw_udp_cmd(cmd, &rx[1..n]).await {
            if rlen > 0 {
                sock.send_to(&rep[..rlen], meta.endpoint).await.ok();
            }
            continue;
        }

        let mut rep = [0u8; 64];

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
            sock.send_to(&rep, meta.endpoint).await.ok();
            continue;
        }

        let rlen = critical_section::with(|_cs| {
            REGS.lock(|r| {
                UDP_STATE.lock(|st| {
                    let mut h = Hooks;
                    let mut c = Cfg;
                    udp_app_cmd(
                        cmd,
                        &rx[1..n],
                        &mut rep,
                        &mut r.borrow_mut(),
                        &mut h,
                        &mut c,
                        &mut st.borrow_mut(),
                        now_ms32(),
                        &ver,
                    )
                })
            })
        });
        if rlen == 0 {
            continue; // unknown command: silent
        }

        let target = if same {
            meta.endpoint
        } else {
            // cross-subnet: directed broadcast reply on port+1
            IpEndpoint {
                addr: IpAddress::Ipv4(Ipv4Address::new(255, 255, 255, 255)),
                port: UDP_CFG_BCAST_PORT,
            }
        };
        sock.send_to(&rep[..rlen], target).await.ok();

        // reboot contract: reply is on the wire -> flush history -> reboot
        let reboot_now =
            critical_section::with(|_cs| UDP_STATE.lock(|st| st.borrow_mut().take_reboot_pending()));
        if reboot_now {
            crate::log::wrn("udp: delayed reboot");
            // flush history before the swap reboot (history_sync in udp_task.c)
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::Sync)
                .ok();
            crate::reboot::cold();
        }
    }
}

/// fw_udp.c command handlers; returns (reply, len) when the command was a
/// firmware-channel one (None -> not ours, generic layer takes it).
async fn fw_udp_cmd(cmd: u8, payload: &[u8]) -> Option<([u8; 8], usize)> {
    match cmd {
        // START [size LE32][keyhash 32B?] -> [01][status][v2_chunk LE16]
        0x01 => {
            if payload.len() < 4 {
                return Some(([0; 8], 0)); // malformed: silent like the C gate
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
        // Pages are committed one at a time with a yield in between: the
        // whole-slot freeze of a single 1400 B write starves the net poll
        // task long enough for the W5500 MACRAW buffer to drop a window.
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

/// Sender shares our /24 (udp_task.c same_subnet24).
fn same_subnet24(addr: &IpAddress) -> bool {
    // proto-ipv6 disabled: IpAddress is a single-variant (Ipv4-only) enum
    let IpAddress::Ipv4(v4) = *addr;
    let octets = v4.octets();
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
    (octets[0], octets[1], octets[2]) == local
}
