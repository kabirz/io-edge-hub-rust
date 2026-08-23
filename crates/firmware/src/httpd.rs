//! HTTP server on :80, port of src/web/httpd.c + web_cmds.c:
//! gzip SPA, JSON APIs, POST commands, keep-alive, pipelining, 405/404
//! semantics, 2-connection cap, 128 B POST body limit.

use core::fmt::Write as _;

use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, Stack, Ipv4Address};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write as _;

use io_edge_hub_proto::regmap::{
    self as rm, RegHooks, RegMap,
};
use io_edge_hub_proto::web_json::{history_web_name_valid, json_get_i32, json_get_str, url_query_get};

use crate::appstate::{Hooks, REGS, version};
use crate::{log, reboot, systime};

pub const HTTP_PORT: u16 = 80;

const RX_BUF: usize = 640;
const BODY_MAX: usize = 128;

static INDEX_GZ: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/index_html.gz"));

#[embassy_executor::task(pool_size = 2)]
pub async fn http_task(stack: Stack<'static>, rx_buf: &'static mut [u8; RX_BUF], tx_buf: &'static mut [u8; 2048]) {
    let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);
    sock.set_timeout(Some(Duration::from_secs(75))); // > 60s keep-alive idle
    loop {
        if sock.accept(HTTP_PORT).await.is_err() {
            Timer::after_millis(100).await;
            continue;
        }
        serve(&mut sock).await;
        sock.abort();
        Timer::after_millis(10).await;
    }
}

/// One connection: request/response cycles until close/timeout/error.
async fn serve(sock: &mut TcpSocket<'static>) {
    let mut rbuf = [0u8; RX_BUF];
    let mut rx_len = 0usize;
    let mut rx_idle = Instant::now();
    let mut body = [0u8; BODY_MAX + 8];

    loop {
        // 5s half-request timeout, 60s keep-alive idle (httpd.c)
        let limit = if rx_len > 0 { Duration::from_secs(5) } else { Duration::from_secs(60) };
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
        rx_idle = Instant::now();
        let _ = rx_idle;

        // parse loop: possibly pipelined requests within the buffer
        loop {
            let he = match find_hdr_end(&rbuf[..rx_len]) {
                Some(i) => i,
                None => {
                    if rx_len >= RX_BUF - 1 {
                        respond(sock, "400 Bad Request", "application/json",
                            b"{\"ok\":false,\"err\":\"header too large\"}", false).await;
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
                    respond(sock, "400 Bad Request", "application/json",
                        b"{\"ok\":false,\"err\":\"bad request\"}", false).await;
                    return;
                }
            };
            let method = core::str::from_utf8(&m[..mlen]).unwrap_or("");
            let content_len = hdr_content_len(&rbuf[..he]);
            let cli_close = hdr_has(&rbuf[..he], b"Connection: close");

            if content_len > BODY_MAX {
                respond(sock, "400 Bad Request", "application/json",
                    b"{\"ok\":false,\"err\":\"body too large\"}", false).await;
                return;
            }

            if method == "GET" {
                let used = he + 4;
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
    // method
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
    // target
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

fn hdr_has(hdr: &[u8], needle: &[u8]) -> bool {
    hdr_find(hdr, needle).is_some()
}

/// case-insensitive header lookup, value trimmed (httpd.c hdr_find)
fn hdr_find<'a>(hdr: &'a [u8], key: &[u8]) -> Option<&'a [u8]> {
    let mut i = 0usize;
    while i < hdr.len() {
        let eol = hdr[i..].iter().position(|&b| b == b'\n').map(|p| i + p).unwrap_or(hdr.len());
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

async fn respond(sock: &mut TcpSocket<'static>, status: &str, ctype: &str, body: &[u8], keep: bool) {
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

async fn respond_extra(sock: &mut TcpSocket<'static>, status: &str, ctype: &str, body: &[u8], extra: &str) {
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

async fn json_ok(sock: &mut TcpSocket<'static>) {
    respond(sock, "200 OK", "application/json", b"{\"ok\":true}", true).await;
}

async fn json_err(sock: &mut TcpSocket<'static>, status: &str, err: &str) {
    let mut b = heapless::String::<96>::new();
    let _ = write!(&mut b, "{{\"ok\":false,\"err\":\"{}\"}}", err);
    respond(sock, status, "application/json", b.as_bytes(), true).await;
}

async fn dispatch(sock: &mut TcpSocket<'static>, method: &str, target: &[u8], body: Option<&[u8]>) {
    // strip query
    let (path, query) = match target.iter().position(|&b| b == b'?') {
        Some(q) => (&target[..q], Some(&target[q + 1..])),
        None => (target, None),
    };
    let path = core::str::from_utf8(path).unwrap_or("");
    let is_get = method == "GET";
    let is_post = method == "POST";

    if is_get && path == "/" {
        respond_extra(sock, "200 OK", "text/html", INDEX_GZ,
            "Content-Encoding: gzip\r\n").await;
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
        // M4.1: no fs bridge yet — empty list (fs-unmounted semantic)
        respond(sock, "200 OK", "application/json", b"{\"files\":[]}", true).await;
        return;
    }
    if is_get && path == "/api/history/download" {
        let ok = query.is_some_and(|q| {
            url_query_get(q, "name").is_some_and(|n| {
                history_web_name_valid(n) && n.starts_with(b"data_") && n.len() >= 6
            })
        });
        if !ok {
            json_err(sock, "400 Bad Request", "invalid file name").await;
            return;
        }
        // M4.1: valid names resolve against an (empty) file set -> not openable
        json_err(sock, "400 Bad Request", "invalid file name").await;
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
        // M4.1: empty fs -> delete always fails like the C path
        json_err(sock, "400 Bad Request", "delete failed").await;
        return;
    }

    // known path wrong method -> 405, else 404
    const KNOWN: [&str; 14] = [
        "/", "/api/info", "/api/io", "/api/regs", "/api/history",
        "/api/history/download", "/ws",
        "/api/do", "/api/reg", "/api/time", "/api/cfg", "/api/save",
        "/api/reboot", "/api/history/delete",
    ];
    if KNOWN.contains(&path) {
        json_err(sock, "405 Method Not Allowed", "method not allowed").await;
        return;
    }
    respond(sock, "404 Not Found", "application/json",
        b"{\"ok\":false,\"err\":\"not found\"}", true).await;
}

fn web_write_do_bit(bit: u16, state: bool) -> bool {
    critical_section::with(|_cs| {
        REGS.lock(|r| r.borrow_mut().io_write_do_bit(bit, state, &mut Hooks).is_ok())
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

/// POST /api/cfg validation (web_cmd_exec_cfg); None = ok, Some(err msg).
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
                    r.borrow_mut().io_write_holding(
                        (rm::HOLDING_IP_OCTET1_IDX + i) as u16,
                        *v as u16,
                        &mut Hooks,
                    ).ok();
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

// ==================== JSON builders (web_cmds.c) ====================

fn regs_snapshot() -> RegMap {
    critical_section::with(|_cs| REGS.lock(|r| {
        let mut copy = RegMap::new(0);
        copy.holding = r.borrow().holding;
        copy.input = r.borrow().input;
        copy
    }))
}

fn build_info_json(out: &mut heapless::String<704>) {
    let r = regs_snapshot();
    let g = |i: usize| r.get_holding(i as u16);
    let mac = crate::net::current_mac();
    let mac_str = heapless::String::<18>::new();
    let _ = mac_str; // formatted below via write!
    let link = crate::net::net_link_up();
    let uptime_ms = embassy_time::Instant::now().as_millis() as u64;
    let _ = write!(
        out,
        "{{\"t\":\"info\",\"version\":\"v{}\",\"build\":\"{}\",\"board\":\"io_edge_f407vet6\",\
\"hclk_mhz\":168,\"flash_kb\":512,\"sram_kb\":192,\"mac\":\"{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\",\
\"ip\":\"{}.{}.{}.{}\",\"slave_id\":{},\"rs485_baud\":{},\"can_id\":{},\"can_baud\":{},\
\"uptime_ms\":{},\"time\":{},\"hist_en\":{},\"lfs_free\":0,\"lfs_total\":0,\"net_up\":{},\
\"di_ms\":{},\"ai_ms\":{}}}",
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
        link,
        g(rm::HOLDING_DI_SAMPLE_MS_IDX),
        g(rm::HOLDING_AI_SAMPLE_MS_IDX),
    );
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
        let _ = write!(out, "{}{}", if i > 0 { "," } else { "" }, r.get_input((rm::INPUT_AI0_IDX + i) as u16));
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
        let _ = write!(out, "{}{}", if i > 0 { "," } else { "" }, r.get_input(i as u16));
    }
    let _ = write!(out, "]}}");
}

// re-exported IP helper for build_info (unused warning guard)
#[allow(dead_code)]
fn _ip_unused(a: Ipv4Address) -> IpAddress {
    IpAddress::Ipv4(a)
}
