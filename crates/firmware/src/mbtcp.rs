//! Modbus TCP server on :502, port of src/modbus/tcp.c semantics:
//! max 2 concurrent masters, the 3rd connection is accepted then aborted.
//! Diagnostics are shared with RTU through the global MbServer (mb_server.c).

use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write as _;

use io_edge_hub_proto::mbtcp_adu::{MBTCP_ADU_TX_MAX, mbtcp_adu_process};

use crate::appstate::{Hooks, MB_SERVER, REGS};

pub const MBTCP_PORT: u16 = 502;

/// One serving socket: accept -> serve until closed -> accept again.
/// Buffers come in as task args: two instances must not share in-body statics
/// (StaticCell double-init panics). pool_size = 2: two concurrent masters.
/// Occupied serving slots (0..2); gates the rejector listener.
pub static BUSY: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

#[embassy_executor::task(pool_size = 2)]
pub async fn conn_task(
    stack: Stack<'static>,
    slot: usize,
    rx_buf: &'static mut [u8; 512],
    tx_buf: &'static mut [u8; 512],
) {
    let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);
    sock.set_timeout(Some(Duration::from_secs(120)));
    loop {
        crate::stackmark::probe(slot);
        // port-only endpoint = addr None (ANY); Some(0.0.0.0) would NOT match
        if sock.accept(MBTCP_PORT).await.is_err() {
            Timer::after_millis(100).await;
            continue;
        }
        BUSY.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        serve(&mut sock, slot).await;
        BUSY.fetch_sub(1, core::sync::atomic::Ordering::Relaxed);
        // hard reset: instant Closed -> instantly reusable for the next
        // accept (graceful close lingers in FIN/TIME_WAIT states and leaves
        // the port without a listener)
        sock.abort();
        Timer::after_millis(10).await;
    }
}

/// Third listener: accepts the excess connection then immediately aborts it
/// (tcp.c accepts-then-aborts when no serving slot is free — the client's
/// connect() succeeds and its request dies with a reset).
/// Arms ONLY while both serving slots are busy and disarms within 50 ms of
/// the load dropping — a lingering listener steals the next legit client
/// (the FTP rejector had the same latch bug).
#[embassy_executor::task]
pub async fn reject_task(stack: Stack<'static>, rx_buf: &'static mut [u8; 64], tx_buf: &'static mut [u8; 64]) {
    use embassy_futures::select::{select, Either};
    let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);
    sock.set_timeout(Some(Duration::from_secs(5)));
    loop {
        crate::stackmark::probe(crate::stackmark::slot::MB_REJECT);
        if BUSY.load(core::sync::atomic::Ordering::Relaxed) >= 2 {
            match select(sock.accept(MBTCP_PORT), Timer::after_millis(50)).await {
                Either::First(Ok(())) => {
                    sock.abort();
                }
                _ => {
                    sock.abort(); // busy window passed: disarm promptly
                }
            }
        } else {
            Timer::after_millis(5).await;
        }
    }
}

/// Serve one connection: accumulate frames per MBAP length, 500ms half-frame
/// deadline (mirrors src/modbus/tcp.c), single send per reply.
/// Total on-wire length of the ADU in `frame` (MBAP length clamped like tcp.c).
fn mbap_frame_len(frame: &[u8]) -> usize {
    6 + u16::from_be_bytes([frame[4], frame[5]]).min(256) as usize
}

async fn serve(sock: &mut TcpSocket<'static>, slot: usize) {
    let mut rbuf = [0u8; 300]; // raw read chunks
    let mut frame = [0u8; 264]; // MBAP + PDU
    let mut flen = 0usize;
    let mut deadline: Option<Instant> = None;
    let mut out = [0u8; MBTCP_ADU_TX_MAX];

    loop {
        crate::stackmark::probe(slot);
        if let Some(d) = deadline {
            if Instant::now() >= d {
                return; // half-frame timeout
            }
        }
        let n = match sock.read(&mut rbuf).await {
            Ok(0) => return, // closed
            Ok(n) => n,
            Err(_) => return,
        };
        let mut ix = 0;
        while ix < n {
            // copy only what the current frame still needs: a chunk holding
            // several pipelined ADUs must leave the tail for the next pass
            let want = if flen < 8 { 8 } else { mbap_frame_len(&frame) };
            let take = (n - ix).min(want - flen);
            frame[flen..flen + take].copy_from_slice(&rbuf[ix..ix + take]);
            flen += take;
            ix += take;

            if flen >= 8 && flen >= mbap_frame_len(&frame) {
                let rlen = critical_section::with(|_cs| {
                    REGS.lock(|r| {
                        MB_SERVER.lock(|s| {
                            let mut h = Hooks;
                            mbtcp_adu_process(
                                &frame[..mbap_frame_len(&frame)],
                                &mut out,
                                &mut s.borrow_mut(),
                                &mut r.borrow_mut(),
                                &mut h,
                            )
                        })
                    })
                });
                if rlen > 0 && sock.write_all(&out[..rlen]).await.is_err() {
                    return;
                }
                flen = 0;
                deadline = None;
            }
        }
        if flen > 0 && deadline.is_none() {
            deadline = Some(Instant::now() + Duration::from_millis(500));
        }
    }
}
