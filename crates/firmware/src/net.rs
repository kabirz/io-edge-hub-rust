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
    defmt::info!("net: spi up, entering w5500 init");
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
    defmt::info!("net: w5500 chip ok");

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

    static RES: StaticCell<embassy_net::StackResources<12>> = StaticCell::new();
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
    static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut sock = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; 8]),
        RX_BUF.init([0u8; 2048]),
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
            crate::reboot::cold();
        }
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
