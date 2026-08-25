//! HTTP server on :80: gzip SPA, JSON APIs, POST commands, keep-alive,
//! pipelining, 405/404 semantics, 2-connection cap, 128 B POST body limit.

use core::fmt::Write as _;

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, Ipv4Address, Stack};

use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write as _;

use io_edge_hub_proto::regmap::{self as rm, RegHooks, RegMap};
use io_edge_hub_proto::web_json::{
    history_web_name_valid, json_get_i32, json_get_str, url_query_get,
};
use io_edge_hub_proto::ws::{ws_accept_key, ws_frame_hdr, FeedEvent, WsParser};

use crate::appstate::{version, Hooks, REGS};
use crate::{log, reboot, systime};

pub const HTTP_PORT: u16 = 80;

const RX_BUF: usize = 640;
const BODY_MAX: usize = 128;

static INDEX_GZ: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/index_html.gz"));

/// Single WS session: the 2nd upgrade gets 503 "ws busy".
static WS_ACTIVE: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task(pool_size = 2)]
pub async fn http_task(
    stack: Stack<'static>,
    name: &'static str,
    rx_buf: &'static mut [u8; RX_BUF],
    tx_buf: &'static mut [u8; 2048],
) {
    let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);
    sock.set_timeout(Some(Duration::from_secs(75))); // > 60s keep-alive idle
    loop {
        crate::stackmark::probe(name);
        if sock.accept(HTTP_PORT).await.is_err() {
            Timer::after_millis(100).await;
            continue;
        }
        serve(&mut sock, name).await;
        sock.abort();
        Timer::after_millis(10).await;
    }
}

/// One connection: request/response cycles until close/timeout/error.
async fn serve(sock: &mut TcpSocket<'static>, name: &'static str) {
    let mut rbuf = [0u8; RX_BUF];
    let mut rx_len = 0usize;
    let mut body = [0u8; BODY_MAX + 8];

    loop {
        crate::stackmark::probe(name);
        // 5s half-request timeout, 60s keep-alive idle
        let limit = if rx_len > 0 {
            Duration::from_secs(5)
        } else {
            Duration::from_secs(60)
        };
        let n = match embassy_futures::select::select(
            sock.read(&mut rbuf[rx_len.min(RX_BUF - 1)..]),
            Timer::after(limit),
        )
        .await
        {
            embassy_futures::select::Either::First(r) => match r {
                Ok(0) => return,
                Ok(n) => n,
                Err(_) => return,
            },
            embassy_futures::select::Either::Second(_) => return, // idle/half-request timeout
        };
        rx_len = (rx_len + n).min(RX_BUF - 1);
        rbuf[rx_len] = 0;

        // parse loop: possibly pipelined requests within the buffer
        loop {
            let he = match find_hdr_end(&rbuf[..rx_len]) {
                Some(i) => i,
                None => {
                    if rx_len >= RX_BUF - 1 {
                        respond(
                            sock,
                            "400 Bad Request",
                            "application/json",
                            b"{\"ok\":false,\"err\":\"header too large\"}",
                            false,
                        )
                        .await;
                        return;
                    }
                    break;
                }
            };
            let mut m = [0u8; 8];
            let mut target = [0u8; 96];
            let (mlen, tlen) = match parse_request_line(&rbuf[..he], &mut m, &mut target) {
                Some(v) => v,
                None => {
                    respond(
                        sock,
                        "400 Bad Request",
                        "application/json",
                        b"{\"ok\":false,\"err\":\"bad request\"}",
                        false,
                    )
                    .await;
                    return;
                }
            };
            let method = core::str::from_utf8(&m[..mlen]).unwrap_or("");
            let content_len = hdr_content_len(&rbuf[..he]);
            let cli_close = hdr_find(&rbuf[..he], b"Connection:")
                .map(|v| v.trim_ascii().eq_ignore_ascii_case(b"close"))
                .unwrap_or(false);

            if content_len > BODY_MAX {
                respond(
                    sock,
                    "400 Bad Request",
                    "application/json",
                    b"{\"ok\":false,\"err\":\"body too large\"}",
                    false,
                )
                .await;
                return;
            }

            if method == "GET" {
                let used = he + 4;
                // /ws upgrade: handshake then the connection becomes a WS session
                if target[..tlen] == *b"/ws" && hdr_eq(&rbuf[..he], b"Upgrade:", b"websocket") {
                    // claim the single-session slot BEFORE any await: the 101
                    // write/flush below yields, and a racing upgrade on the
                    // other task would otherwise pass the active check
                    if WS_ACTIVE
                        .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
                        .is_err()
                    {
                        respond(
                            sock,
                            "503 Service Unavailable",
                            "application/json",
                            b"{\"ok\":false,\"err\":\"ws busy\"}",
                            false,
                        )
                        .await;
                        // let the 503 segment get ACKed before the task-loop
                        // abort() discards it (RST eats unACKed data)
                        Timer::after_millis(50).await;
                        return;
                    }
                    if let Some(accept) = ws_extract_key(&rbuf[..he]) {
                        let accept = ws_accept_key(&accept);
                        let mut resp = heapless::String::<160>::new();
                        let _ = write!(
                            &mut resp,
                            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n\
Connection: Upgrade\r\nSec-WebSocket-Accept: {}\r\n\r\n",
                            core::str::from_utf8(&accept).unwrap_or("")
                        );
                        let _ = sock.write_all(resp.as_bytes()).await;
                        let _ = sock.flush().await;
                        log::inf("ws: connected");
                        let pending_len = rx_len - used;
                        ws_session(sock, &rbuf[used..used + pending_len]).await;
                        log::inf("ws: closed");
                    } else {
                        json_err(sock, "400 Bad Request", "bad request").await;
                    }
                    WS_ACTIVE.store(false, Ordering::Relaxed);
                    return;
                }
                dispatch(sock, method, &target[..tlen], None).await;
                if cli_close {
                    return;
                }
                rx_len -= used;
                rbuf.copy_within(used.., 0);
                continue;
            }

            // POST: wait for the full body
            let body_off = he + 4;
            if rx_len - body_off < content_len {
                break; // need more bytes
            }
            let used = body_off + content_len;
            body[..content_len].copy_from_slice(&rbuf[body_off..body_off + content_len]);
            dispatch(sock, method, &target[..tlen], Some(&body[..content_len])).await;
            if cli_close {
                return;
            }
            rx_len -= used;
            rbuf.copy_within(used.., 0);
            if rx_len > 0 {
                continue; // pipelined POST
            }
            break;
        }
    }
}

fn find_hdr_end(rx: &[u8]) -> Option<usize> {
    rx.windows(4).position(|w| w == b"\r\n\r\n")
}

/// sscanf "%7s %95s" equivalent: whitespace (space/tab) separated tokens.
fn parse_request_line(rx: &[u8], m: &mut [u8; 8], t: &mut [u8; 96]) -> Option<(usize, usize)> {
    let mut i = 0usize;
    while i < rx.len() && (rx[i] == b' ' || rx[i] == b'\t') {
        i += 1;
    }
    let ms = i;
    while i < rx.len() && rx[i] != b' ' && rx[i] != b'\t' && rx[i] != b'\r' && rx[i] != b'\n' {
        i += 1;
    }
    if i == ms || i - ms > 8 {
        return None;
    }
    m[..i - ms].copy_from_slice(&rx[ms..i]);
    let mlen = i - ms;
    while i < rx.len() && (rx[i] == b' ' || rx[i] == b'\t') {
        i += 1;
    }
    let ts = i;
    while i < rx.len() && rx[i] != b' ' && rx[i] != b'\t' && rx[i] != b'\r' && rx[i] != b'\n' {
        i += 1;
    }
    if i == ts {
        return None;
    }
    // sscanf "%95s" truncates long tokens instead of failing (C httpd gives
    // 404 for the truncated long path, not 400)
    let tlen = (i - ts).min(96);
    t[..tlen].copy_from_slice(&rx[ts..ts + tlen]);
    Some((mlen, tlen))
}

fn hdr_content_len(hdr: &[u8]) -> usize {
    match hdr_find(hdr, b"Content-Length:") {
        Some(v) => {
            let s = core::str::from_utf8(v).unwrap_or("").trim();
            s.parse::<usize>().unwrap_or(0)
        }
        None => 0,
    }
}

#[allow(dead_code)]
fn hdr_has(hdr: &[u8], needle: &[u8]) -> bool {
    hdr_find(hdr, needle).is_some()
}

/// "Key: value" comparison with the value trimmed (case-insensitive).
fn hdr_eq(hdr: &[u8], key: &[u8], want: &[u8]) -> bool {
    match hdr_find(hdr, key) {
        Some(v) => v.trim_ascii().eq_ignore_ascii_case(want),
        None => false,
    }
}

/// case-insensitive header lookup, value trimmed.
fn hdr_find<'a>(hdr: &'a [u8], key: &[u8]) -> Option<&'a [u8]> {
    let mut i = 0usize;
    while i < hdr.len() {
        let eol = hdr[i..]
            .iter()
            .position(|&b| b == b'\n')
            .map(|p| i + p)
            .unwrap_or(hdr.len());
        let line = &hdr[i..eol];
        if line.len() > key.len()
            && line[..key.len()].eq_ignore_ascii_case(key)
            && line[key.len()] == b' '
        {
            return Some(&line[key.len() + 1..]);
        }
        i = eol + 1;
    }
    None
}

async fn respond(
    sock: &mut TcpSocket<'static>,
    status: &str,
    ctype: &str,
    body: &[u8],
    keep: bool,
) {
    let mut hdr = heapless::String::<192>::new();
    let _ = write!(
        &mut hdr,
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: {}\r\n\r\n",
        status,
        ctype,
        body.len(),
        if keep { "keep-alive" } else { "close" }
    );
    let _ = sock.write_all(hdr.as_bytes()).await;
    let _ = sock.write_all(body).await;
    let _ = sock.flush().await;
}

async fn respond_extra(
    sock: &mut TcpSocket<'static>,
    status: &str,
    ctype: &str,
    body: &[u8],
    extra: &str,
) {
    let mut hdr = heapless::String::<256>::new();
    let _ = write!(
        &mut hdr,
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: keep-alive\r\n{}\r\n",
        status,
        ctype,
        body.len(),
        extra
    );
    let _ = sock.write_all(hdr.as_bytes()).await;
    let _ = sock.write_all(body).await;
    let _ = sock.flush().await;
}

/// Wait until the storage task processed one more RPC (generation advance).
async fn rpc_wait(seq_before: u32) -> bool {
    let deadline = Instant::now() + Duration::from_millis(2500);
    while crate::storage::RPC_SEQ.load(Ordering::Relaxed) <= seq_before {
        if Instant::now() >= deadline {
            return false;
        }
        Timer::after_millis(2).await;
    }
    true
}

async fn json_ok(sock: &mut TcpSocket<'static>) {
    respond(sock, "200 OK", "application/json", b"{\"ok\":true}", true).await;
}

async fn json_err(sock: &mut TcpSocket<'static>, status: &str, err: &str) {
    let mut b = heapless::String::<96>::new();
    let _ = write!(&mut b, "{{\"ok\":false,\"err\":\"{}\"}}", err);
    respond(sock, status, "application/json", b.as_bytes(), true).await;
}

async fn dispatch(sock: &mut TcpSocket<'static>, method: &str, target: &[u8], body: Option<&[u8]>) {
    let (path, query) = match target.iter().position(|&b| b == b'?') {
        Some(q) => (&target[..q], Some(&target[q + 1..])),
        None => (target, None),
    };
    let path = core::str::from_utf8(path).unwrap_or("");
    let is_get = method == "GET";
    let is_post = method == "POST";

    if is_get && path == "/" {
        respond_extra(
            sock,
            "200 OK",
            "text/html",
            INDEX_GZ,
            "Content-Encoding: gzip\r\n",
        )
        .await;
        return;
    }
    if is_get && path == "/api/info" {
        let mut b = heapless::String::<704>::new();
        build_info_json(&mut b);
        respond(sock, "200 OK", "application/json", b.as_bytes(), true).await;
        return;
    }
    if is_get && path == "/api/io" {
        let mut b = heapless::String::<256>::new();
        build_io_json(&mut b);
        respond(sock, "200 OK", "application/json", b.as_bytes(), true).await;
        return;
    }
    if is_get && path == "/api/regs" {
        let mut b = heapless::String::<256>::new();
        build_regs_json(&mut b);
        respond(sock, "200 OK", "application/json", b.as_bytes(), true).await;
        return;
    }
    if is_get && path == "/api/history" {
        let seq = crate::storage::RPC_SEQ.load(Ordering::Relaxed);
        if crate::storage::QUEUE
            .try_send(crate::storage::StorageCmd::SnapReq)
            .is_ok()
        {
            let _ = rpc_wait(seq).await;
        }
        // buffer cap and margin of the C httpd, kept for byte-identical JSON
        let mut b = heapless::String::<704>::new();
        let _ = write!(b, "{{\"files\":[");
        let snap = critical_section::with(|_cs| {
            crate::storage::FS_SNAP.lock(|s| {
                let g = s.borrow();
                (g.entries, g.count)
            })
        });
        for i in 0..snap.1 {
            let e = &snap.0[i];
            let name_end = e[..20].iter().position(|&c| c == 0).unwrap_or(20);
            let size = u32::from_be_bytes([e[20], e[21], e[22], e[23]]);
            if b.len() + 64 > b.capacity() {
                break;
            }
            let _ = write!(
                b,
                "{}{{\"name\":\"{}\",\"size\":{}}}",
                if i > 0 { "," } else { "" },
                core::str::from_utf8(&e[..name_end]).unwrap_or(""),
                size
            );
        }
        let _ = write!(b, "]}}");
        respond(sock, "200 OK", "application/json", b.as_bytes(), true).await;
        return;
    }
    if is_get && path == "/api/history/download" {
        let name = query.and_then(|q| url_query_get(q, "name"));
        let valid = name.is_some_and(|n| history_web_name_valid(n));
        if !valid {
            json_err(sock, "400 Bad Request", "invalid file name").await;
            return;
        }
        let name = name.unwrap();
        let mut nb = [0u8; 24];
        let nl = name.len().min(23);
        nb[..nl].copy_from_slice(&name[..nl]);
        let seq = crate::storage::RPC_SEQ.load(Ordering::Relaxed);
        crate::storage::QUEUE
            .try_send(crate::storage::StorageCmd::FileOpen(nb))
            .ok();
        let ok = rpc_wait(seq).await;
        let size = critical_section::with(|_cs| crate::storage::FILE_DL.lock(|f| f.borrow().size));
        if !ok || size == 0 {
            json_err(sock, "400 Bad Request", "invalid file name").await;
            return;
        }
        let mut hdr = heapless::String::<256>::new();
        let _ = write!(
            hdr,
            "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n\
Content-Length: {}\r\nConnection: close\r\n\
Content-Disposition: attachment; filename=\"{}\"\r\n\r\n",
            size,
            core::str::from_utf8(&nb[..nl]).unwrap_or("")
        );
        let _ = sock.write_all(hdr.as_bytes()).await;
        loop {
            let seq = crate::storage::RPC_SEQ.load(Ordering::Relaxed);
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::FileChunk)
                .ok();
            if !rpc_wait(seq).await {
                break;
            }
            let (chunk_len, eof, err, sent) = critical_section::with(|_cs| {
                crate::storage::FILE_DL.lock(|f| {
                    let g = f.borrow();
                    (g.chunk_len, g.eof, g.err, g.sent)
                })
            });
            if err {
                break;
            }
            // write the chunk BEFORE testing done: the final partial chunk
            // lands with sent == size and must still reach the wire
            if chunk_len > 0 {
                let chunk = critical_section::with(|_cs| {
                    crate::storage::FILE_DL.lock(|f| {
                        let g = f.borrow();
                        let mut c = [0u8; 2048];
                        c[..g.chunk_len].copy_from_slice(&g.chunk[..g.chunk_len]);
                        (c, g.chunk_len)
                    })
                });
                if sock.write_all(&chunk.0[..chunk.1]).await.is_err() {
                    break;
                }
            }
            if (eof && chunk_len == 0) || sent >= size {
                break;
            }
        }
        let _ = sock.flush().await;
        return;
    }
    if is_post && path == "/api/do" {
        let b = body.unwrap_or(&[]);
        let ok = match (json_get_i32(b, "index"), json_get_i32(b, "value")) {
            (Some(i), Some(v)) => (0..8).contains(&i) && web_write_do_bit(i as u16, v != 0),
            _ => false,
        };
        if ok {
            json_ok(sock).await;
        } else {
            json_err(sock, "400 Bad Request", "invalid index/value").await;
        }
        return;
    }
    if is_post && path == "/api/reg" {
        let b = body.unwrap_or(&[]);
        let ok = match (json_get_i32(b, "addr"), json_get_i32(b, "value")) {
            (Some(a), Some(v)) => web_cmd_reg(a, v),
            _ => false,
        };
        if ok {
            json_ok(sock).await;
        } else {
            json_err(sock, "400 Bad Request", "invalid addr/value").await;
        }
        return;
    }
    if is_post && path == "/api/time" {
        let b = body.unwrap_or(&[]);
        let ok = match json_get_i32(b, "ts") {
            Some(ts) => {
                let mut h = Hooks;
                h.set_timestamp(ts as u32)
            }
            None => false,
        };
        if ok {
            json_ok(sock).await;
        } else {
            json_err(sock, "400 Bad Request", "invalid timestamp").await;
        }
        return;
    }
    if is_post && path == "/api/save" {
        let mut h = Hooks;
        h.holding_save();
        json_ok(sock).await;
        return;
    }
    if is_post && path == "/api/reboot" {
        log::inf("web reboot requested");
        json_ok(sock).await;
        // delayed so the response reaches the wire first
        Timer::after_millis(150).await;
        reboot::cold();
        return;
    }
    if is_post && path == "/api/cfg" {
        let err = web_cmd_cfg(body.unwrap_or(&[]));
        match err {
            None => json_ok(sock).await,
            Some(e) => json_err(sock, "400 Bad Request", e).await,
        }
        return;
    }
    if is_post && path == "/api/history/delete" {
        let b = body.unwrap_or(&[]);
        let ok = json_get_str(b, "name").is_some_and(|n| {
            if !history_web_name_valid(n) {
                return false;
            }
            let mut nb = [0u8; 24];
            let nl = n.len().min(23);
            nb[..nl].copy_from_slice(&n[..nl]);
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::Del(nb))
                .is_ok()
        });
        if ok {
            crate::log::inf("history deleted (web)");
            json_ok(sock).await;
        } else {
            json_err(sock, "400 Bad Request", "delete failed").await;
        }
        return;
    }

    // known path wrong method -> 405, else 404
    const KNOWN: [&str; 14] = [
        "/",
        "/api/info",
        "/api/io",
        "/api/regs",
        "/api/history",
        "/api/history/download",
        "/ws",
        "/api/do",
        "/api/reg",
        "/api/time",
        "/api/cfg",
        "/api/save",
        "/api/reboot",
        "/api/history/delete",
    ];
    if KNOWN.contains(&path) {
        json_err(sock, "405 Method Not Allowed", "method not allowed").await;
        return;
    }
    respond(
        sock,
        "404 Not Found",
        "application/json",
        b"{\"ok\":false,\"err\":\"not found\"}",
        true,
    )
    .await;
}

fn web_write_do_bit(bit: u16, state: bool) -> bool {
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            r.borrow_mut()
                .io_write_do_bit(bit, state, &mut Hooks)
                .is_ok()
        })
    })
}

fn web_cmd_reg(addr: i32, value: i32) -> bool {
    if !(0..rm::MODBUS_HOLDING_REGISTER_NUMBERS as i32).contains(&addr)
        || !(0..=0xFFFF).contains(&value)
    {
        return false;
    }
    if addr as usize == rm::HOLDING_REBOOT_IDX {
        if value != 0 {
            crate::appstate::set_reboot_status(true);
        }
        return true;
    }
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            r.borrow_mut()
                .io_write_holding(addr as u16, value as u16, &mut Hooks)
                .is_ok()
        })
    })
}

/// POST /api/cfg validation: None = ok, Some(err msg).
fn web_cmd_cfg(body: &[u8]) -> Option<&'static str> {
    if let Some(ip) = json_get_str(body, "ip") {
        let s = core::str::from_utf8(ip).ok()?;
        let mut oct = [0u8; 4];
        let mut it = s.split('.');
        let mut ok = true;
        for o in oct.iter_mut() {
            match it.next().and_then(|p| p.parse::<u8>().ok()) {
                Some(v) => *o = v,
                None => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok || it.next().is_some() || !rm::ip_addr_valid(oct[0], oct[1], oct[2], oct[3]) {
            return Some("invalid ip");
        }
        for (i, v) in oct.iter().enumerate() {
            critical_section::with(|_cs| {
                REGS.lock(|r| {
                    r.borrow_mut()
                        .io_write_holding(
                            (rm::HOLDING_IP_OCTET1_IDX + i) as u16,
                            *v as u16,
                            &mut Hooks,
                        )
                        .ok();
                })
            });
        }
    }
    if let Some(v) = json_get_i32(body, "rs485") {
        if !(1200..=115200).contains(&v) {
            return Some("invalid rs485 baud");
        }
        let _ = web_cmd_reg(rm::HOLDING_RS485_BAUDRATE_IDX as i32, v);
    }
    if let Some(v) = json_get_i32(body, "sid") {
        if !(1..=247).contains(&v) {
            return Some("invalid slave id");
        }
        let _ = web_cmd_reg(rm::HOLDING_SLAVE_ID_IDX as i32, v);
    }
    if let Some(v) = json_get_i32(body, "can_bps") {
        if !matches!(v, 50 | 100 | 125 | 250 | 500 | 800 | 1000) {
            return Some("invalid can baud");
        }
        let _ = web_cmd_reg(rm::HOLDING_CAN_BAUDRATE_IDX as i32, v);
    }
    if let Some(v) = json_get_i32(body, "can_id") {
        if !(1..=0x7FF).contains(&v) {
            return Some("invalid can id");
        }
        let _ = web_cmd_reg(rm::HOLDING_CAN_ID_IDX as i32, v);
    }
    None
}

// ---- WebSocket ----

/// Extract the 24-byte Sec-WebSocket-Key value from the request header.
fn ws_extract_key(hdr: &[u8]) -> Option<[u8; 24]> {
    let v = hdr_find(hdr, b"Sec-WebSocket-Key:")?;
    if v.len() < 24 {
        return None;
    }
    let mut key = [0u8; 24];
    key.copy_from_slice(&v[..24]);
    Some(key)
}

/// Frame bytes appended by the (sync) frame callback; flushed by the async
/// session loop after feed() returns — socket writes must be .await'ed.
fn ws_queue_frame(out: &mut heapless::Vec<u8, 768>, opcode: u8, payload: &[u8]) {
    let mut hdr = [0u8; 10];
    let hl = ws_frame_hdr(&mut hdr, opcode, payload.len());
    out.extend_from_slice(&hdr[..hl]).ok();
    out.extend_from_slice(payload).ok();
}

/// One text frame (header + payload) built in a scratch buffer (push path).
fn ws_frame_buf<'a>(scratch: &'a mut [u8; 800], payload: &'a [u8]) -> &'a [u8] {
    let mut hdr = [0u8; 10];
    let hl = ws_frame_hdr(&mut hdr, 0x1, payload.len());
    scratch[..hl].copy_from_slice(&hdr[..hl]);
    scratch[hl..hl + payload.len()].copy_from_slice(payload);
    &scratch[..hl + payload.len()]
}

async fn ws_session(sock: &mut TcpSocket<'static>, pending: &[u8]) {
    let mut parser = WsParser::new();
    let mut rbuf = [0u8; 2048];
    let mut out: heapless::Vec<u8, 768> = heapless::Vec::new();
    let mut scratch = [0u8; 800];
    let mut alive = true;
    // fw upload CRC accumulated over accepted binary frames
    let mut fw_crc = 0u16;

    // frames complete inside the sync parser: queue replies, flush after
    let step = |parser: &mut WsParser,
                data: &[u8],
                out: &mut heapless::Vec<u8, 768>,
                alive: &mut bool,
                fw_crc: &mut u16| {
        *alive &= parser.feed(data, |p, ev| match ev {
            FeedEvent::Close => *alive = false,
            FeedEvent::Frame {
                fin,
                opcode,
                payload_len,
            } => {
                if !fin {
                    // fragmented frames unsupported
                    ws_queue_frame(out, 0x8, &[]);
                    *alive = false;
                    return;
                }
                match opcode {
                    0x1 => {
                        let n = payload_len.min(255);
                        ws_handle_cmd(out, &p.payload[..n], fw_crc);
                    }
                    // binary: firmware data frames straight into the page
                    // buffer (fw_upg_write), CRC over accepted frames
                    0x2 => {
                        if crate::fw::active() {
                            let n = payload_len.min(p.payload.len());
                            if crate::fw::write(&p.payload[..n]) {
                                *fw_crc = io_edge_hub_proto::fw_upg::crc16_ccitt(
                                    *fw_crc,
                                    &p.payload[..n],
                                );
                            } else {
                                crate::log::wrn("ws fw: write failed");
                            }
                        }
                    }
                    0x8 => {
                        // close: reply close then end the session
                        ws_queue_frame(out, 0x8, &[]);
                        *alive = false;
                    }
                    0x9 => ws_queue_frame(out, 0xA, &p.payload[..payload_len]), // ping->pong
                    0xA => {}                                                   // pong: ignore
                    _ => *alive = false,
                }
            }
        });
    };

    step(&mut parser, pending, &mut out, &mut alive, &mut fw_crc);
    let _ = sock.write_all(&out).await;
    let _ = sock.flush().await;
    if !alive {
        return;
    }

    // ~500ms to first push, then 1s io/regs + 10s info
    let mut push_ticker = Ticker::every(Duration::from_millis(1000));
    let mut info_ticker = Ticker::every(Duration::from_millis(10_000));
    Timer::after_millis(500).await;
    loop {
        let ev = embassy_futures::select::select3(
            sock.read(&mut rbuf),
            push_ticker.next(),
            info_ticker.next(),
        )
        .await;
        match ev {
            embassy_futures::select::Either3::First(r) => match r {
                Ok(0) | Err(_) => return,
                Ok(n) => {
                    out.clear();
                    step(&mut parser, &rbuf[..n], &mut out, &mut alive, &mut fw_crc);
                    let _ = sock.write_all(&out).await;
                    let _ = sock.flush().await;
                    if !alive {
                        return;
                    }
                }
            },
            embassy_futures::select::Either3::Second(_) => {
                let mut b = heapless::String::<256>::new();
                build_io_json(&mut b);
                let _ = sock
                    .write_all(ws_frame_buf(&mut scratch, b.as_bytes()))
                    .await;
                let mut b = heapless::String::<256>::new();
                build_regs_json(&mut b);
                let _ = sock
                    .write_all(ws_frame_buf(&mut scratch, b.as_bytes()))
                    .await;
                let _ = sock.flush().await;
            }
            embassy_futures::select::Either3::Third(_) => {
                let mut b = heapless::String::<704>::new();
                build_info_json(&mut b);
                let _ = sock
                    .write_all(ws_frame_buf(&mut scratch, b.as_bytes()))
                    .await;
                let _ = sock.flush().await;
            }
        }
    }
}

/// WS commands: quick cmds queued as reply frames. The payload is the full
/// JSON text; "cmd" selects the handler, remaining fields come from the same
/// buffer.
fn ws_handle_cmd(out: &mut heapless::Vec<u8, 768>, payload: &[u8], fw_crc: &mut u16) {
    let cmd = json_get_str(payload, "cmd").unwrap_or(b"");
    if cmd == b"do" {
        if let (Some(i), Some(v)) = (
            json_get_i32(payload, "index"),
            json_get_i32(payload, "value"),
        ) {
            if (0..8).contains(&i) && web_write_do_bit(i as u16, v != 0) {
                // do command answers with the fresh full IO snapshot
                let mut b = heapless::String::<256>::new();
                build_io_json(&mut b);
                ws_queue_frame(out, 0x1, b.as_bytes());
                return;
            }
        }
        ws_queue_frame(out, 0x1, b"{\"ok\":false,\"err\":\"bad index\"}");
        return;
    }
    if cmd == b"reg" {
        if let (Some(a), Some(v)) = (
            json_get_i32(payload, "addr"),
            json_get_i32(payload, "value"),
        ) {
            let ok = web_cmd_reg(a, v);
            ws_queue_frame(
                out,
                0x1,
                if ok {
                    b"{\"ok\":true}"
                } else {
                    b"{\"ok\":false}"
                },
            );
            return;
        }
        ws_queue_frame(out, 0x1, b"{\"ok\":false}");
        return;
    }
    if cmd == b"time" {
        if let Some(ts) = json_get_i32(payload, "ts") {
            let mut h = Hooks;
            let ok = h.set_timestamp(ts as u32);
            ws_queue_frame(
                out,
                0x1,
                if ok {
                    b"{\"ok\":true}"
                } else {
                    b"{\"ok\":false}"
                },
            );
            return;
        }
        ws_queue_frame(out, 0x1, b"{\"ok\":false}");
        return;
    }
    if cmd == b"cfg" {
        match web_cmd_cfg(payload) {
            None => ws_queue_frame(out, 0x1, b"{\"ok\":true}"),
            Some(e) => {
                let mut b = heapless::String::<96>::new();
                let _ = write!(&mut b, "{{\"ok\":false,\"err\":\"{}\"}}", e);
                ws_queue_frame(out, 0x1, b.as_bytes());
            }
        }
        return;
    }
    if cmd == b"save" {
        let mut h = Hooks;
        h.holding_save();
        ws_queue_frame(out, 0x1, b"{\"ok\":true}");
        return;
    }
    if cmd == b"factory_reset" {
        crate::storage::CTRL_QUEUE
            .try_send(crate::storage::StorageCmd::CfgEraseAll)
            .ok();
        crate::log::inf("factory reset via ws, rebooting");
        ws_queue_frame(out, 0x1, b"{\"ok\":true}");
        crate::appstate::set_reboot_status(true);
        return;
    }
    if cmd == b"fw_start" {
        let size = json_get_i32(payload, "size").unwrap_or(0);
        if size <= 0 {
            ws_queue_frame(out, 0x1, b"{\"ok\":false,\"err\":\"bad size\"}");
            return;
        }
        let mut kh_buf = [0u8; 32];
        let mut kh: Option<&[u8; 32]> = None;
        if let Some(b64) = json_get_str(payload, "keyhash") {
            match io_edge_hub_proto::fw_upg::b64_decode(b64, &mut kh_buf) {
                Some(32) => kh = Some(&kh_buf),
                _ => {
                    ws_queue_frame(out, 0x1, b"{\"ok\":false,\"err\":\"keyhash mismatch\"}");
                    return;
                }
            }
        }
        let rc = crate::fw::start(size as u32, kh);
        let r: &[u8] = match rc {
            0 => {
                crate::log::inf("ws fw: start");
                b"{\"ok\":true}"
            }
            -2 => b"{\"ok\":false,\"err\":\"keyhash mismatch\"}",
            -3 => b"{\"ok\":false,\"err\":\"already in progress\"}",
            _ => b"{\"ok\":false,\"err\":\"erase/init\"}",
        };
        ws_queue_frame(out, 0x1, r);
        return;
    }
    if cmd == b"fw_end" {
        if !crate::fw::active() {
            ws_queue_frame(out, 0x1, b"{\"ok\":false,\"err\":\"not in progress\"}");
            return;
        }
        let r = ws_fw_end(fw_crc);
        if r == b"{\"ok\":true}" {
            // flush history, then the heartbeat task does the graceful reboot
            crate::storage::QUEUE
                .try_send(crate::storage::StorageCmd::Sync)
                .ok();
            crate::appstate::set_reboot_status(true);
        }
        ws_queue_frame(out, 0x1, r);
        return;
    }
    ws_queue_frame(out, 0x1, b"{\"ok\":false,\"err\":\"unknown cmd\"}");
}

/// WS fw_end: size precheck, CRC/TLV readback verify, request the swap,
/// reply.
fn ws_fw_end(fw_crc: &mut u16) -> &'static [u8] {
    let got = crate::fw::received();
    if got == 0 {
        crate::fw::abort();
        return b"{\"ok\":false,\"err\":\"no data\"}";
    }
    if got != crate::fw::total() {
        crate::fw::abort();
        crate::log::wrn("ws fw: size mismatch");
        return b"{\"ok\":false,\"err\":\"size mismatch\"}";
    }
    if !crate::fw::finish(Some(*fw_crc)) {
        *fw_crc = 0;
        return b"{\"ok\":false,\"err\":\"crc mismatch\"}";
    }
    *fw_crc = 0;
    if !crate::fw::boot_set_pending(true) {
        return b"{\"ok\":false,\"err\":\"boot_request\"}";
    }
    crate::log::inf("ws fw: verified, rebooting for swap");
    b"{\"ok\":true}"
}

// ---- JSON builders ----

fn regs_snapshot() -> RegMap {
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            let mut copy = RegMap::new(0);
            copy.holding = r.borrow().holding;
            copy.input = r.borrow().input;
            copy
        })
    })
}

fn build_info_json(out: &mut heapless::String<704>) {
    let r = regs_snapshot();
    let g = |i: usize| r.get_holding(i as u16);
    let mac = crate::net::current_mac();
    let link = crate::net::net_link_up();
    let uptime_ms = embassy_time::Instant::now().as_millis() as u64;
    let (lfs_free, lfs_total) = critical_section::with(|_cs| {
        crate::storage::FS_SNAP.lock(|s| {
            let sn = s.borrow();
            (sn.free, sn.total)
        })
    });
    let _ = write!(
        out,
        "{{\"t\":\"info\",\"version\":\"{}\",\"build\":\"{}\",\"board\":\"io_edge_f407vet6\",\
\"hclk_mhz\":168,\"flash_kb\":512,\"sram_kb\":192,\"mac\":\"{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\",\
\"ip\":\"{}.{}.{}.{}\",\"slave_id\":{},\"rs485_baud\":{},\"can_id\":{},\"can_baud\":{},\
\"uptime_ms\":{},\"time\":{},\"hist_en\":{},\"lfs_free\":{},\"lfs_total\":{},\"net_up\":{},\
\"di_ms\":{},\"ai_ms\":{}",
        version::FW_VERSION,
        version::FW_BUILD,
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        g(rm::HOLDING_IP_OCTET1_IDX),
        g(rm::HOLDING_IP_OCTET2_IDX),
        g(rm::HOLDING_IP_OCTET3_IDX),
        g(rm::HOLDING_IP_OCTET4_IDX),
        g(rm::HOLDING_SLAVE_ID_IDX),
        g(rm::HOLDING_RS485_BAUDRATE_IDX),
        g(rm::HOLDING_CAN_ID_IDX),
        g(rm::HOLDING_CAN_BAUDRATE_IDX),
        uptime_ms,
        systime::now_epoch(),
        g(rm::HOLDING_HISTORY_ENABLE_IDX) != 0,
        lfs_free,
        lfs_total,
        link,
        g(rm::HOLDING_DI_SAMPLE_MS_IDX),
        g(rm::HOLDING_AI_SAMPLE_MS_IDX),
    );
    // public key fingerprint: host tools and the web page fetch it here so
    // a key rotation only touches the firmware, never the clients
    let _ = write!(out, ",\"keyhash\":\"");
    for b in io_edge_hub_proto::fw_upg::FW_KEYHASH {
        let _ = write!(out, "{:02x}", b);
    }
    let _ = write!(out, "\"}}");
}

fn build_io_json(out: &mut heapless::String<256>) {
    let r = regs_snapshot();
    let di = r.get_input(rm::INPUT_DI_IDX as u16);
    let do_v = r.get_holding(rm::HOLDING_DO_IDX as u16);
    let _ = write!(out, "{{\"t\":\"io\",\"di\":[");
    for i in 0..rm::DI_NUM {
        let _ = write!(out, "{}{}", if i > 0 { "," } else { "" }, (di >> i) & 1);
    }
    let _ = write!(out, "],\"do\":[");
    for i in 0..rm::DO_NUM {
        let _ = write!(out, "{}{}", if i > 0 { "," } else { "" }, (do_v >> i) & 1);
    }
    let _ = write!(out, "],\"ai\":[");
    for i in 0..rm::AI_NUM {
        let _ = write!(
            out,
            "{}{}",
            if i > 0 { "," } else { "" },
            r.get_input((rm::INPUT_AI0_IDX + i) as u16)
        );
    }
    let _ = write!(
        out,
        "],\"di_en\":{},\"ai_en\":{},\"ms\":{}}}",
        r.get_holding(rm::HOLDING_DI_ENABLE_IDX as u16),
        r.get_holding(rm::HOLDING_AI_ENABLE_IDX as u16),
        embassy_time::Instant::now().as_millis() as u64
    );
}

fn build_regs_json(out: &mut heapless::String<256>) {
    let r = regs_snapshot();
    let mut h = Hooks;
    let _ = write!(out, "{{\"t\":\"regs\",\"holding\":[");
    for i in 0..rm::MODBUS_HOLDING_REGISTER_NUMBERS {
        let v = r.io_read_holding(i as u16, &mut h);
        let _ = write!(out, "{}{}", if i > 0 { "," } else { "" }, v);
    }
    let _ = write!(out, "],\"input\":[");
    for i in 0..rm::MODBUS_INPUT_REGISTER_NUMBERS {
        let _ = write!(
            out,
            "{}{}",
            if i > 0 { "," } else { "" },
            r.get_input(i as u16)
        );
    }
    let _ = write!(out, "]}}");
}

// re-exported IP helper for build_info (unused warning guard)
#[allow(dead_code)]
fn _ip_unused(a: Ipv4Address) -> IpAddress {
    IpAddress::Ipv4(a)
}
