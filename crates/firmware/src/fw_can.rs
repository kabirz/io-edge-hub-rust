//! CAN1 firmware-upgrade channel (PA11 RX / PA12 TX, AF9).
//!
//! - protocol 0x101 cmd / 0x102 reply / 0x103 data / 0x104 keyhash
//!   / 0x105 version frames; flow control acks every 64 B
//! - START reopens unconditionally, keyhash only checked when all 5
//!   frames arrived, CONFIRM carries no CRC (MCUboot signature gates),
//!   REBOOT is not answered

use embassy_stm32::bind_interrupts;
use embassy_stm32::can::filter::Mask16;
use embassy_stm32::can::{
    Can, CanTx, Fifo, Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler, RxBuf,
    SceInterruptHandler, StandardId, TxInterruptHandler,
};
use embassy_stm32::peripherals::{CAN1, PA11, PA12};
use embassy_stm32::Peri;
use embassy_time::Timer;

use io_edge_hub_proto::fw_upg as upg;
use io_edge_hub_proto::regmap::{HOLDING_CAN_BAUDRATE_IDX, HOLDING_CAN_ID_IDX};

use crate::appstate::REGS;
use crate::fw;

bind_interrupts! {
    struct CanIrqs {
        CAN1_TX => TxInterruptHandler<CAN1>;
        CAN1_RX0 => Rx0InterruptHandler<CAN1>;
        CAN1_RX1 => Rx1InterruptHandler<CAN1>;
        CAN1_SCE => SceInterruptHandler<CAN1>;
    }
}

const PLATFORM_RX: u16 = 0x101;
const PLATFORM_TX: u16 = 0x102;
const FW_DATA_RX: u16 = 0x103;
const KEYHASH_RX: u16 = 0x104;
const VERSION_TX: u16 = 0x105;

// 0x101 command words (data LE32)
const CMD_START_UPDATE: u32 = 0;
const CMD_CONFIRM: u32 = 1;
const CMD_VERSION: u32 = 2;
const CMD_REBOOT: u32 = 3;

// 0x102 reply codes (data LE32)
const CODE_OFFSET: u32 = 0;
const CODE_UPDATE_SUCCESS: u32 = 1;
const CODE_VERSION: u32 = 2;
const CODE_CONFIRM: u32 = 3;
const CODE_FLASH_ERROR: u32 = 4;
const CODE_TRANSFER_ERROR: u32 = 5;
const CODE_KEYHASH_ERROR: u32 = 6;

const CONFIRM_MAGIC: u32 = 0x55AA_55AA;

const KEYHASH_CHUNK: usize = 7;
const KEYHASH_CHUNKS: usize = (upg::FW_KEYHASH_LEN + KEYHASH_CHUNK - 1) / KEYHASH_CHUNK;
const KEYHASH_FULL_MASK: u8 = (1u16 << KEYHASH_CHUNKS) as u8 - 1;
/// Flow control: OFFSET ack every 64 B (Zephyr authoritative semantics,
/// `fw_written % 64 == 0`). The host tool reads a reply after every 8
/// frames/64 B and tolerates a missing one with a 2 s timeout — at 512 B it
/// burns 7 timeouts per window, stretching a full push to >90 minutes.
const ACK_INTERVAL: u32 = 64;

/// Supported bitrates (800k is not realizable at 42 MHz PCLK1; others fall
/// back to 250k).
const SUPPORTED_KBPS: [u32; 6] = [50, 100, 125, 250, 500, 1000];
const FALLBACK_KBPS: u32 = 250;

#[embassy_executor::task]
pub async fn fw_can_task(
    can1: Peri<'static, CAN1>,
    rx: Peri<'static, PA11>,
    tx: Peri<'static, PA12>,
) {
    let (mut kbps, id) = critical_section::with(|_cs| {
        REGS.lock(|r| {
            let g = r.borrow();
            (
                g.get_holding(HOLDING_CAN_BAUDRATE_IDX as u16) as u32,
                g.get_holding(HOLDING_CAN_ID_IDX as u16) & 0x7FF,
            )
        })
    });
    if !SUPPORTED_KBPS.contains(&kbps) {
        crate::log::wrn("can baud unsupported, fallback 250k");
        kbps = FALLBACK_KBPS;
    }

    let mut can = Can::new(can1, rx, tx, CanIrqs);
    can.set_bitrate(kbps * 1000);

    // filter bank 0 / FIFO0, two 16-bit half-slots:
    // A = exact business ID, B = 0x100-0x1FF upgrade range
    let id_a = StandardId::new(id).unwrap();
    let id_b = StandardId::new(0x100).unwrap();
    let mask_full = StandardId::new(0x7FF).unwrap();
    let mask_top = StandardId::new(0x700).unwrap();
    can.modify_filters().enable_bank(
        0,
        Fifo::Fifo0,
        [
            Mask16::frames_with_std_id(id_a, mask_full),
            Mask16::frames_with_std_id(id_b, mask_top),
        ],
    );

    can.enable().await;

    // RX: interrupt-fed 128-frame ring — the 3-deep hardware FIFO overflows
    // during NOR page writes when a host bursts 512 B windows, so the ISR
    // drains it into the ring this task consumes.
    // TX: direct mailbox writes via CanTx, NOT BufferedCanTx — the buffered
    // TX interrupt dequeues a frame from its ring and DISCARDS it when
    // Registers::transmit returns WouldBlock, which it does while an
    // identical-ID frame is still pending in a mailbox (embassy 0.6,
    // priority scheduling). That silently ate every same-ID frame after the
    // first, e.g. the 2nd 0x105 version frame. CanTx::write parks on
    // WouldBlock and retries on the mailbox-empty interrupt, like C's
    // mod_can_send mailbox wait.
    let (mut tx, rx) = can.split();
    static RXB: static_cell::StaticCell<RxBuf<128>> = static_cell::StaticCell::new();
    let mut rx = rx.buffered(RXB.init(RxBuf::new()));

    let mut msg = heapless::String::<48>::new();
    use core::fmt::Write as _;
    let _ = write!(msg, "fwcan: {}kbps bus id 0x{:03x}", kbps, id);
    crate::log::inf(&msg);

    // keyhash accumulation (task-local)
    let mut rx_keybuf = [0u8; upg::FW_KEYHASH_LEN];
    let mut key_chunk_mask: u8 = 0;

    loop {
        crate::stackmark::probe("fwcan");
        let env = match rx.read().await {
            Ok(env) => env,
            Err(_) => continue,
        };
        let fid = match env.frame.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => continue,
        };
        let data = env.frame.data();
        match fid {
            PLATFORM_RX => {
                handle_platform(&mut tx, data, &mut rx_keybuf, &mut key_chunk_mask).await;
            }
            FW_DATA_RX => {
                handle_fw_data(&mut tx, data).await;
            }
            KEYHASH_RX => {
                handle_keyhash(data, &mut rx_keybuf, &mut key_chunk_mask);
            }
            _ => {} // business frames: no consumer yet
        }
    }
}

async fn send(tx: &mut CanTx<'static>, id: u16, data: &[u8]) {
    if let Ok(f) = Frame::new_standard(id, data) {
        let _ = tx.write(&f).await;
    }
}

async fn fw_reply(tx: &mut CanTx<'static>, code: u32, arg: u32) {
    let mut d = [0u8; 8];
    d[..4].copy_from_slice(&code.to_le_bytes());
    d[4..].copy_from_slice(&arg.to_le_bytes());
    send(tx, PLATFORM_TX, &d).await;
}

async fn handle_platform(
    tx: &mut CanTx<'static>,
    data: &[u8],
    rx_keybuf: &mut [u8; upg::FW_KEYHASH_LEN],
    key_chunk_mask: &mut u8,
) {
    if data.len() != 8 {
        fw_reply(tx, CODE_FLASH_ERROR, 0).await;
        return;
    }
    let cmd = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let arg = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);

    match cmd {
        CMD_START_UPDATE => {
            // unconditional reopen: a failed previous transfer must not
            // leave stale offsets around
            fw::abort();
            let kh = if *key_chunk_mask & KEYHASH_FULL_MASK == KEYHASH_FULL_MASK {
                *key_chunk_mask = 0;
                Some(&*rx_keybuf)
            } else {
                None
            };
            let rc = fw::start(arg, kh).await;
            if rc == -2 {
                crate::log::wrn("fwcan: keyhash mismatch");
                fw_reply(tx, CODE_KEYHASH_ERROR, 0).await;
                return;
            }
            if rc != 0 {
                crate::log::wrn("fwcan: start failed");
                fw_reply(tx, CODE_FLASH_ERROR, 0).await;
                return;
            }
            crate::log::inf("fwcan: start");
            fw_reply(tx, CODE_OFFSET, 0).await;
        }
        CMD_CONFIRM => {
            if !fw::active() {
                crate::log::wrn("fwcan: confirm before start");
                fw_reply(tx, CODE_TRANSFER_ERROR, 0).await;
                return;
            }
            // No CRC on the CAN channel: the readback still verifies the
            // programming result; image integrity gating is the app-side
            // trial boot (revert if the new image never confirms).
            if !fw::finish(None).await {
                crate::log::wrn("fwcan: confirm verify failed");
                fw_reply(tx, CODE_TRANSFER_ERROR, 0).await;
                return;
            }
            // `arg != 0` (permanent request) is accepted but no longer
            // changes bootloader semantics — every update boots as a trial.
            crate::log::inf("fwcan: confirmed, awaiting reboot");
            fw_reply(tx, CODE_CONFIRM, CONFIRM_MAGIC).await;
        }
        CMD_VERSION => {
            let ver = crate::appstate::version::FW_VERSION;
            fw_reply(tx, CODE_VERSION, ver.len() as u32).await;
            let mut off = 0usize;
            let mut seq = 0u8;
            while off < ver.len() {
                let chunk = (ver.len() - off).min(7);
                let mut d = [0u8; 8];
                d[0] = seq;
                d[1..1 + chunk].copy_from_slice(&ver.as_bytes()[off..off + chunk]);
                send(tx, VERSION_TX, &d).await;
                off += chunk;
                seq += 1;
            }
        }
        CMD_REBOOT => {
            // no reply: ~100ms drain, flush the history cache, then reset
            // inline
            crate::log::inf("fwcan: reboot requested");
            Timer::after_millis(100).await;
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::Sync)
                .ok();
            Timer::after_millis(500).await;
            crate::reboot::system_reset();
        }
        _ => {}
    }
}

async fn handle_fw_data(tx: &mut CanTx<'static>, data: &[u8]) {
    if !fw::active() {
        crate::log::wrn("fwcan: data before start");
        fw_reply(tx, CODE_TRANSFER_ERROR, 0).await;
        return;
    }
    if !fw::write(data) {
        crate::log::err("fwcan: write failed");
        fw_reply(tx, CODE_FLASH_ERROR, 0).await;
        return;
    }
    if !fw::flush().await {
        crate::log::err("fwcan: flash write failed");
        fw_reply(tx, CODE_FLASH_ERROR, 0).await;
        return;
    }
    let got = fw::received();
    if got == fw::total() {
        fw_reply(tx, CODE_UPDATE_SUCCESS, got).await;
    } else if got % ACK_INTERVAL == 0 {
        fw_reply(tx, CODE_OFFSET, got).await;
    }
}

fn handle_keyhash(data: &[u8], rx_keybuf: &mut [u8; upg::FW_KEYHASH_LEN], mask: &mut u8) {
    if data.is_empty() {
        return;
    }
    let seq = data[0] as usize;
    if seq >= KEYHASH_CHUNKS {
        crate::log::wrn("fwcan: keyhash bad seq");
        return;
    }
    let rem = upg::FW_KEYHASH_LEN - seq * KEYHASH_CHUNK;
    let chunk = rem.min(KEYHASH_CHUNK);
    if data.len() < 1 + chunk {
        crate::log::wrn("fwcan: keyhash short frame");
        return;
    }
    rx_keybuf[seq * KEYHASH_CHUNK..seq * KEYHASH_CHUNK + chunk]
        .copy_from_slice(&data[1..1 + chunk]);
    *mask |= 1u8 << seq;
}
