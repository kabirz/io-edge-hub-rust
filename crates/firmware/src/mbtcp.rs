//! Modbus TCP on W5500 hardware sockets 1/2: max 2 concurrent masters, the
//! 3rd connection finds no free listener and the chip's TCP engine answers
//! RST (the same cap and client-visible behavior as the C firmware's
//! accept-then-abort rejector, without needing one). Diagnostics are shared
//! with RTU through the global MbServer.
//!
//! MbSock is polled by net_task (the W5500 is not shared across tasks):
//! drain RX -> accumulate MBAP frames -> reply -> retry, with the same
//! 500 ms half-frame deadline and abort-style close/relisten as the smoltcp
//! version.

use embassy_time::{Duration, Instant};

use io_edge_hub_proto::mbtcp_adu::{mbtcp_adu_process, MBTCP_ADU_TX_MAX};

use crate::appstate::{Hooks, MB_SERVER, REGS};
use crate::w5500::{SR_CLOSE_WAIT, SR_ESTABLISHED, SR_INIT, SR_LISTEN, W5500};

pub const MBTCP_PORT: u16 = 502;

pub struct MbSock {
    sock: u8,
    name: &'static str,
    rbuf: [u8; 300],  // raw read chunks
    frame: [u8; 264], // MBAP + PDU
    flen: usize,
    deadline: Option<Instant>,
    out: [u8; MBTCP_ADU_TX_MAX],
    olen: usize, // >0: reply parked until TX frees up
}

/// Total on-wire length of the ADU in `frame` (MBAP length clamped).
fn mbap_frame_len(frame: &[u8]) -> usize {
    6 + u16::from_be_bytes([frame[4], frame[5]]).min(256) as usize
}

impl MbSock {
    pub fn new(sock: u8, name: &'static str) -> Self {
        Self {
            sock,
            name,
            rbuf: [0; 300],
            frame: [0; 264],
            flen: 0,
            deadline: None,
            out: [0; MBTCP_ADU_TX_MAX],
            olen: 0,
        }
    }

    fn reset_session(&mut self) {
        self.flen = 0;
        self.deadline = None;
        self.olen = 0;
    }

    /// One poll tick: service the socket, called every ~2 ms by net_task.
    pub fn poll(&mut self, w: &mut W5500) {
        crate::stackmark::probe(self.name);

        // retry a parked reply first (TX freed when the chip ACKed)
        if self.olen > 0 && w.tcp_try_send(self.sock, &self.out[..self.olen]) {
            self.olen = 0;
        }

        // ESTABLISHED, or CLOSE_WAIT with a pipelined request still in the
        // RX buffer (a master may FIN right after sending): drain first —
        // our side can still transmit in CLOSE_WAIT — and close on the next
        // idle tick.
        let sr = w.tcp_state(self.sock);
        let session =
            sr == SR_ESTABLISHED || (sr == SR_CLOSE_WAIT && w.tcp_rx_pending(self.sock) > 0);
        if !session {
            match sr {
                SR_INIT | SR_LISTEN => {
                    self.reset_session(); // between sessions: stay clean
                    return;
                }
                // CLOSED / CLOSE_WAIT / transient states: abort + re-arm
                // the listener so :502 never lingers unserved
                _ => {
                    w.tcp_close_reopen(self.sock, MBTCP_PORT);
                    self.reset_session();
                    return;
                }
            }
        }

        // half-frame deadline (500ms), same as the smoltcp version
        if let Some(d) = self.deadline {
            if Instant::now() >= d {
                w.tcp_close_reopen(self.sock, MBTCP_PORT);
                self.reset_session();
                return;
            }
        }

        loop {
            let n = w.tcp_recv(self.sock, &mut self.rbuf);
            if n == 0 {
                break;
            }
            let mut ix = 0;
            while ix < n {
                // copy only what the current frame still needs: a chunk
                // holding several pipelined ADUs must leave the tail for
                // the next pass
                let want = if self.flen < 8 {
                    8
                } else {
                    mbap_frame_len(&self.frame)
                };
                let take = (n - ix).min(want - self.flen);
                self.frame[self.flen..self.flen + take].copy_from_slice(&self.rbuf[ix..ix + take]);
                self.flen += take;
                ix += take;

                if self.flen >= 8 && self.flen >= mbap_frame_len(&self.frame) {
                    let rlen = critical_section::with(|_cs| {
                        REGS.lock(|r| {
                            MB_SERVER.lock(|s| {
                                let mut h = Hooks;
                                mbtcp_adu_process(
                                    &self.frame[..mbap_frame_len(&self.frame)],
                                    &mut self.out,
                                    &mut s.borrow_mut(),
                                    &mut r.borrow_mut(),
                                    &mut h,
                                )
                            })
                        })
                    });
                    if rlen > 0 {
                        if !w.tcp_try_send(self.sock, &self.out[..rlen]) {
                            // TX busy with un-ACKed replies: park it, retry
                            // on a later poll (order is preserved)
                            self.olen = rlen;
                        }
                    }
                    self.flen = 0;
                    self.deadline = None;
                }
            }
        }
        if self.flen > 0 && self.deadline.is_none() {
            self.deadline = Some(Instant::now() + Duration::from_millis(500));
        }
    }
}
