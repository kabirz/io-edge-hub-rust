//! FTP server on :21, port of src/net/ftpd.c (RFC 959).
//! PASV/EPSV (self-picked ephemeral port) + PORT/EPRT active mode,
//! TYPE A/I CR/LF conversion, REST both directions, APPE, ..-clamping
//! path normalization, admin/admin + anonymous read-only, 3 sessions,
//! 4th client 421. Storage rides the storage-task RPC.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_futures::poll_once;
use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read as _, Write as _};

use crate::storage::{FtpPath, FtpWrMode, StorageCmd, QUEUE, RPC_SEQ};

pub const FTP_PORT: u16 = 21;
const FTP_PASS: &str = "admin";

/// Occupied FTP session slots (0..3); gates the 421 rejector listener.
pub static FTP_BUSY: AtomicU8 = AtomicU8::new(0);
static PASV_PORT: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::task(pool_size = 3)]
pub async fn ftp_task(
    stack: Stack<'static>,
    crx: &'static mut [u8; 1024],
    ctx: &'static mut [u8; 1024],
    drx: &'static mut [u8; 2048],
    dtx: &'static mut [u8; 2048],
) {
    let mut ctrl = TcpSocket::new(stack, crx, ctx);
    let mut data = TcpSocket::new(stack, drx, dtx);
    ctrl.set_timeout(Some(Duration::from_secs(120)));
    data.set_timeout(Some(Duration::from_secs(15)));
    loop {
        if ctrl.accept(FTP_PORT).await.is_err() {
            Timer::after_millis(100).await;
            continue;
        }
        FTP_BUSY.fetch_add(1, Ordering::Relaxed);
        session(&mut ctrl, &mut data, stack).await;
        FTP_BUSY.fetch_sub(1, Ordering::Relaxed);
        ctrl.abort();
        data.abort();
        Timer::after_millis(10).await;
    }
}

/// 4th+ client: connect -> 421 -> close (only listens when all slots busy).
#[embassy_executor::task]
pub async fn ftp_reject_task(
    stack: Stack<'static>,
    rx_buf: &'static mut [u8; 128],
    tx_buf: &'static mut [u8; 128],
) {
    let mut sock = TcpSocket::new(stack, rx_buf, tx_buf);
    sock.set_timeout(Some(Duration::from_secs(5)));
    loop {
        if FTP_BUSY.load(Ordering::Relaxed) >= 3 {
            if sock.accept(FTP_PORT).await.is_ok() {
                let _ = sock.write_all(b"421 Too many users\r\n").await;
                let _ = sock.flush().await;
                Timer::after_millis(50).await;
                sock.abort();
            }
        }
        Timer::after_millis(5).await;
    }
}

struct Session {
    authed: bool,
    anon: bool,
    cwd: heapless::String<64>,
    type_ascii: bool,
    rest: u32,
    rename_from: FtpPath,
    rename_pending: bool,
    /// active-mode target (PORT/EPRT); passive when None
    port_ep: Option<IpEndpoint>,
    /// chosen PASV port for the next passive transfer
    passive_port: u16,
    quit: bool,
}

async fn send_line(sock: &mut TcpSocket<'static>, msg: &str) {
    let _ = sock.write_all(msg.as_bytes()).await;
    let _ = sock.write_all(b"\r\n").await;
    let _ = sock.flush().await;
}

async fn session(ctrl: &mut TcpSocket<'static>, data: &mut TcpSocket<'static>, stack: Stack<'static>) {
    let mut s = Session {
        authed: false,
        anon: false,
        cwd: heapless::String::new(),
        type_ascii: false,
        rest: 0,
        rename_from: [0; 96],
        rename_pending: false,
        port_ep: None,
        passive_port: 0,
        quit: false,
    };
    s.cwd.push('/').ok();
    send_line(ctrl, "220 io-edge-hub FTP service ready").await;

    let mut rbuf = [0u8; 1024];
    let mut rx_len = 0usize;
    while !s.quit {
        let n = match ctrl.read(&mut rbuf[rx_len..]).await {
            Ok(0) | Err(_) => return,
            Ok(n) => n,
        };
        rx_len += n;
        loop {
            let eol = match rbuf[..rx_len].iter().position(|&b| b == b'\n') {
                Some(i) => i,
                None => {
                    if rx_len >= rbuf.len() - 1 {
                        return; // line overflow
                    }
                    break;
                }
            };
            let mut line_len = eol;
            if line_len > 0 && rbuf[line_len - 1] == b'\r' {
                line_len -= 1;
            }
            let mut line = [0u8; 256];
            let ll = line_len.min(255);
            line[..ll].copy_from_slice(&rbuf[..ll]);
            let used = eol + 1;
            rbuf.copy_within(used..rx_len, 0);
            rx_len -= used;
            handle_command(ctrl, data, &mut s, &line[..ll], stack).await;
        }
    }
}

/// Stack-based ".." clamping path normalization (ftpd.c norm_path).
fn norm_path(cwd: &str, input: &str) -> heapless::String<64> {
    let mut parts: heapless::Vec<heapless::String<48>, 8> = heapless::Vec::new();
    let joined: heapless::String<160> = if input.starts_with('/') {
        input.chars().take(159).collect()
    } else {
        let mut j = heapless::String::new();
        let _ = write!(j, "{}/{}", cwd, input);
        j
    };
    for seg in joined.split('/') {
        match seg {
            "" | "." => continue,
            ".." => {
                parts.pop();
            }
            _ => {
                let _ = parts.push(seg.chars().take(47).collect());
            }
        }
    }
    let mut out = heapless::String::new();
    out.push('/').ok();
    for (i, p) in parts.iter().enumerate() {
        if i > 0 {
            out.push('/').ok();
        }
        let _ = write!(out, "{}", p);
    }
    out
}

fn ftp_path_bytes(p: &str) -> FtpPath {
    let mut b = [0u8; 96];
    let l = p.len().min(94);
    b[..l].copy_from_slice(&p.as_bytes()[..l]);
    b
}

async fn rpc_wait(seq_before: u32) -> bool {
    let deadline = Instant::now() + Duration::from_millis(2500);
    while RPC_SEQ.load(Ordering::Relaxed) <= seq_before {
        if Instant::now() >= deadline {
            return false;
        }
        Timer::after_millis(2).await;
    }
    true
}

fn rpc_send(cmd: StorageCmd) -> u32 {
    let seq = RPC_SEQ.load(Ordering::Relaxed);
    QUEUE.try_send(cmd).ok();
    seq
}

fn ftp_res() -> (bool, bool, u32) {
    critical_section::with(|_cs| crate::storage::FTP_RES.lock(|r| *r.borrow()))
}

fn fmt_ls_time(name: &str) -> heapless::String<20> {
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let b = name.as_bytes();
    if b.len() >= 13 && &b[..5] == b"data_" {
        let two = |i: usize| -> Option<u32> {
            core::str::from_utf8(&b[i..i + 2]).ok()?.parse().ok()
        };
        if let (Some(mon), Some(day), Some(h), Some(m)) = (two(5), two(7), two(10), two(12)) {
            if (1..=12).contains(&mon) && (1..=31).contains(&day) && h <= 23 && m <= 59 {
                let mut t = heapless::String::new();
                let _ = write!(t, "{} {:2} {:02}:{:02}", MONTHS[(mon - 1) as usize], day, h, m);
                return t;
            }
        }
    }
    let now = crate::systime::now_epoch() as i64 + 8 * 3600;
    let days = now.div_euclid(86_400);
    let secs = now.rem_euclid(86_400);
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let mut t = heapless::String::new();
    let _ = write!(
        t,
        "{} {:2} {:02}:{:02}",
        MONTHS[(m as usize - 1).min(11)],
        d,
        secs / 3600,
        (secs / 60) % 60
    );
    t
}

/// Hand the data connection to the pending transfer:
/// PORT/EPRT -> connect out; passive -> the listener was armed at PASV time,
/// wait for the client handshake to complete (writable == established).
async fn open_data(s: &mut Session, data: &mut TcpSocket<'static>) -> bool {
    match s.port_ep.take() {
        Some(ep) => {
            data.abort();
            data.connect(ep).await.is_ok()
        }
        None => {
            if s.passive_port == 0 {
                return false;
            }
            match select(data.wait_write_ready(), Timer::after_millis(10_000)).await {
                Either::First(_) => true,
                Either::Second(_) => false,
            }
        }
    }
}

fn pick_pasv_port() -> u16 {
    let n = PASV_PORT.fetch_add(1, Ordering::Relaxed);
    40000 + (n as u16 % 1000)
}

async fn handle_command(
    ctrl: &mut TcpSocket<'static>,
    data: &mut TcpSocket<'static>,
    s: &mut Session,
    line: &[u8],
    _stack: Stack<'static>,
) {
    let text = core::str::from_utf8(line).unwrap_or("");
    let (cmd, arg) = match text.find(' ') {
        Some(i) => (&text[..i], text[i + 1..].trim()),
        None => (text, ""),
    };
    let mut upper = heapless::String::<8>::new();
    for c in cmd.chars() {
        let _ = upper.push(c.to_ascii_uppercase());
    }
    let cmd = upper.as_str();

    match cmd {
        "USER" => {
            s.anon = arg == "anonymous" || arg == "ftp";
            s.rename_pending = false;
            send_line(ctrl, "331 Please specify the password").await;
        }
        "PASS" => {
            if s.anon || arg == FTP_PASS {
                s.authed = true;
                send_line(ctrl, "230 Login successful").await;
            } else {
                s.authed = false;
                send_line(ctrl, "530 Login incorrect").await;
            }
        }
        "SYST" => send_line(ctrl, "215 UNIX Type: L8").await,
        "FEAT" => {
            send_line(
                ctrl,
                "211-Features:\r\n SIZE\r\n PASV\r\n EPSV\r\n PORT\r\n EPRT\r\n REST STREAM\r\n TYPE A;I\r\n NLST\r\n MKD\r\n RMD\r\n211 END",
            )
            .await;
        }
        "TYPE" => {
            s.type_ascii = arg.starts_with('A') || arg.starts_with('a');
            send_line(ctrl, "200 Type set").await;
        }
        "PWD" => {
            let mut r = heapless::String::<96>::new();
            let _ = write!(r, "257 \"{}\" is the current directory", s.cwd);
            send_line(ctrl, &r).await;
        }
        "CWD" => {
            s.cwd = norm_path(&s.cwd, arg);
            send_line(ctrl, "250 Directory successfully changed").await;
        }
        "CDUP" => {
            s.cwd = norm_path(&s.cwd, "..");
            send_line(ctrl, "250 Directory successfully changed").await;
        }
        "PASV" | "EPSV" => {
            s.port_ep = None;
            s.passive_port = pick_pasv_port();
            let ip = local_ip();
            let p = s.passive_port;
            // Clients (incl. CPython ftplib) connect as soon as the 227/229
            // reply arrives, before sending the transfer command: arm the
            // listener NOW. accept() applies listen() on first poll, so one
            // poll registers LISTEN; the handshake completes autonomously.
            data.abort();
            let _ = poll_once(data.accept(p));
            let mut r = heapless::String::<96>::new();
            if cmd == "PASV" {
                let _ = write!(
                    r,
                    "227 Entering Passive Mode ({},{},{},{},{},{})",
                    ip[0], ip[1], ip[2], ip[3],
                    (p >> 8) & 0xFF, p & 0xFF
                );
            } else {
                let _ = write!(r, "229 Entering Extended Passive Mode (|||{}|)", p);
            }
            send_line(ctrl, &r).await;
        }
        "PORT" => match parse_port_arg(arg) {
            Some(ep) => {
                s.port_ep = Some(ep);
                send_line(ctrl, "200 PORT command successful").await;
            }
            None => send_line(ctrl, "501 Syntax error in parameters").await,
        },
        "EPRT" => match parse_eprt_arg(arg) {
            Some(ep) => {
                s.port_ep = Some(ep);
                send_line(ctrl, "200 EPRT command successful").await;
            }
            None => send_line(ctrl, "522 Network protocol not supported, use (1)").await,
        },
        "LIST" | "NLST" => cmd_list(ctrl, data, s, arg, cmd == "LIST").await,
        "RETR" => cmd_retr(ctrl, data, s, arg).await,
        "STOR" | "APPE" => cmd_stor(ctrl, data, s, arg, cmd == "APPE").await,
        "DELE" | "RMD" | "XRMD" => {
            let fp = norm_path(&s.cwd, arg);
            let can = s.authed && !s.anon;
            let seq = rpc_send(StorageCmd::FtpRemove(ftp_path_bytes(&fp)));
            let (ok, _, _) = if rpc_wait(seq).await && can {
                ftp_res()
            } else {
                (false, false, 0)
            };
            let msg = if cmd == "DELE" {
                if ok { "250 Delete OK" } else { "550 Delete failed" }
            } else if ok {
                "250 Remove OK"
            } else {
                "550 Cannot remove"
            };
            send_line(ctrl, msg).await;
        }
        "MKD" | "XMKD" => {
            let fp = norm_path(&s.cwd, arg);
            let seq = rpc_send(StorageCmd::FtpMkdir(ftp_path_bytes(&fp)));
            let (ok, _, _) = if rpc_wait(seq).await && s.authed && !s.anon {
                ftp_res()
            } else {
                (false, false, 0)
            };
            if ok {
                let mut r = heapless::String::<128>::new();
                let _ = write!(r, "257 \"{}\" created", arg);
                send_line(ctrl, &r).await;
            } else {
                send_line(ctrl, "550 Cannot create directory").await;
            }
        }
        "SIZE" => {
            let fp = norm_path(&s.cwd, arg);
            let seq = rpc_send(StorageCmd::FtpStat(ftp_path_bytes(&fp)));
            let _ = rpc_wait(seq).await;
            let (ok, is_dir, size) = ftp_res();
            if ok && !is_dir {
                let mut r = heapless::String::<32>::new();
                let _ = write!(r, "213 {}", size);
                send_line(ctrl, &r).await;
            } else {
                send_line(ctrl, "550 Not found").await;
            }
        }
        "REST" => {
            s.rest = arg.parse().unwrap_or(0);
            let mut r = heapless::String::<48>::new();
            let _ = write!(r, "350 Restart position accepted ({})", s.rest);
            send_line(ctrl, &r).await;
        }
        "RNFR" => {
            let fp = norm_path(&s.cwd, arg);
            let seq = rpc_send(StorageCmd::FtpStat(ftp_path_bytes(&fp)));
            let _ = rpc_wait(seq).await;
            let (ok, _, _) = ftp_res();
            if !ok || !s.authed || s.anon {
                send_line(ctrl, "550 No such file").await;
            } else {
                s.rename_from = ftp_path_bytes(&fp);
                s.rename_pending = true;
                send_line(ctrl, "350 Ready for RNTO").await;
            }
        }
        "RNTO" => {
            let fp = norm_path(&s.cwd, arg);
            let mut ok = false;
            if s.rename_pending && s.authed && !s.anon {
                let from = s.rename_from;
                let seq = rpc_send(StorageCmd::FtpRename(from, ftp_path_bytes(&fp)));
                let _ = rpc_wait(seq).await;
                let (r, _, _) = ftp_res();
                ok = r;
            }
            s.rename_pending = false;
            send_line(ctrl, if ok { "250 Rename successful" } else { "550 Rename failed" }).await;
        }
        "QUIT" => {
            send_line(ctrl, "221 Goodbye").await;
            s.quit = true;
        }
        "NOOP" | "ALLO" => send_line(ctrl, "200 OK").await,
        _ => send_line(ctrl, "502 Command not implemented").await,
    }
}

fn local_ip() -> [u8; 4] {
    use io_edge_hub_proto::regmap as rm;
    critical_section::with(|_cs| {
        crate::appstate::REGS.lock(|r| {
            let g = r.borrow();
            [
                g.get_holding(rm::HOLDING_IP_OCTET1_IDX as u16) as u8,
                g.get_holding(rm::HOLDING_IP_OCTET2_IDX as u16) as u8,
                g.get_holding(rm::HOLDING_IP_OCTET3_IDX as u16) as u8,
                g.get_holding(rm::HOLDING_IP_OCTET4_IDX as u16) as u8,
            ]
        })
    })
}

fn parse_port_arg(arg: &str) -> Option<IpEndpoint> {
    let vals: heapless::Vec<u16, 6> = arg
        .split(',')
        .map(|p| p.parse::<u16>().ok())
        .collect::<Option<_>>()?;
    if vals.iter().any(|&v| v > 255) {
        return None;
    }
    Some(IpEndpoint {
        addr: IpAddress::Ipv4(Ipv4Address::new(
            vals[0] as u8,
            vals[1] as u8,
            vals[2] as u8,
            vals[3] as u8,
        )),
        port: ((vals[4] as u16) << 8) | vals[5] as u16,
    })
}

fn parse_eprt_arg(arg: &str) -> Option<IpEndpoint> {
    let mut it = arg.split('|');
    it.next()?;
    let proto: u32 = it.next()?.parse().ok()?;
    let ip = it.next()?;
    let port: u16 = it.next()?.parse().ok()?;
    if proto != 1 || port == 0 {
        return None;
    }
    let a: heapless::Vec<u8, 4> = ip
        .split('.')
        .map(|p| p.parse::<u8>().ok())
        .collect::<Option<_>>()?;
    Some(IpEndpoint {
        addr: IpAddress::Ipv4(Ipv4Address::new(a[0], a[1], a[2], a[3])),
        port,
    })
}

async fn cmd_list(ctrl: &mut TcpSocket<'static>, data: &mut TcpSocket<'static>, s: &mut Session, arg: &str, long: bool) {
    // strip typical "LIST -a" style switches
    let mut path_arg = arg;
    if let Some(rest) = arg.strip_prefix("- ") {
        path_arg = rest;
    } else if arg.starts_with('-') && !arg.contains('/') {
        path_arg = "";
    }
    let fp = norm_path(&s.cwd, path_arg);

    let seq = rpc_send(StorageCmd::FtpLs(ftp_path_bytes(&fp)));
    if !rpc_wait(seq).await {
        send_line(ctrl, "550 No such file or directory").await;
        return;
    }
    let (entries, count) = critical_section::with(|_cs| {
        crate::storage::FTP_LS.lock(|l| {
            let g = l.borrow();
            (g.0, g.1)
        })
    });
    if count == 0 {
        send_line(ctrl, "550 No such file or directory").await;
        return;
    }
    send_line(ctrl, "150 Here comes the directory listing").await;
    if !open_data(s, data).await {
        send_line(ctrl, "425 No data connection").await;
        return;
    }
    let mut out = heapless::String::<256>::new();
    for i in 0..count {
        let e = &entries[i];
        let nlen = e[..24].iter().position(|&c| c == 0).unwrap_or(24);
        let name = core::str::from_utf8(&e[..nlen]).unwrap_or("");
        let size = u32::from_be_bytes([e[24], e[25], e[26], e[27]]);
        let is_dir = e[28] == 1;
        out.clear();
        if long {
            let _ = write!(
                out,
                "{} 1 owner group {:10} {} {}\r\n",
                if is_dir { "drwxr-xr-x" } else { "-rw-r--r--" },
                size,
                fmt_ls_time(name),
                name
            );
        } else {
            let _ = write!(out, "{}\r\n", name);
        }
        if data.write_all(out.as_bytes()).await.is_err() {
            break;
        }
    }
    let _ = data.flush().await;
    // graceful FIN (abort would RST and fail the client's final read)
    data.close();
    send_line(ctrl, "226 Directory send OK").await;
}

async fn cmd_retr(ctrl: &mut TcpSocket<'static>, data: &mut TcpSocket<'static>, s: &mut Session, arg: &str) {
    if !s.authed {
        send_line(ctrl, "530 Not logged in").await;
        return;
    }
    let fp = norm_path(&s.cwd, arg);
    let rest = s.rest;
    let seq = rpc_send(StorageCmd::FtpOpenRead { path: ftp_path_bytes(&fp), rest });
    let ok = rpc_wait(seq).await && ftp_res().0;
    let size = critical_section::with(|_cs| {
        crate::storage::FILE_DL.lock(|f| {
            let g = f.borrow();
            (g.size, g.open)
        })
    });
    s.rest = 0;
    if !ok || !size.1 {
        send_line(ctrl, "550 Failed to open file").await;
        return;
    }
    send_line(ctrl, "150 Opening data connection").await;
    if !open_data(s, data).await {
        let seq = rpc_send(StorageCmd::FtpCloseWrite); // closes read too via chunk eof path
        let _ = seq;
        send_line(ctrl, "425 No data connection").await;
        return;
    }
    // chunked read via the storage RPC; ASCII converts \n -> \r\n
    let mut chunk_ascii = [0u8; 1024];
    loop {
        let seq = rpc_send(StorageCmd::FileChunk);
        if !rpc_wait(seq).await {
            break;
        }
        let (chunk, len) = critical_section::with(|_cs| {
            crate::storage::FILE_DL.lock(|f| {
                let g = f.borrow();
                let mut c = [0u8; 512];
                c[..g.chunk_len].copy_from_slice(&g.chunk[..g.chunk_len]);
                (c, g.chunk_len)
            })
        });
        let (eof, err, sent) = critical_section::with(|_cs| {
            crate::storage::FILE_DL.lock(|f| {
                let g = f.borrow();
                (g.eof, g.err, g.sent)
            })
        });
        if err {
            break;
        }
        if len > 0 {
            if s.type_ascii {
                let mut o = 0usize;
                for &b in &chunk[..len] {
                    if b == b'\n' {
                        chunk_ascii[o] = b'\r';
                        o += 1;
                    }
                    chunk_ascii[o] = b;
                    o += 1;
                }
                if data.write_all(&chunk_ascii[..o]).await.is_err() {
                    break;
                }
            } else if data.write_all(&chunk[..len]).await.is_err() {
                break;
            }
        }
        if eof && (len == 0 || sent >= size.0 || sent >= size.0 + rest) {
            // sent counts from the REST offset (FILE_DL.size = remaining)
            break;
        }
    }
    let _ = data.flush().await;
    // graceful FIN: the client's recv must see EOF, not RST
    data.close();
    send_line(ctrl, "226 Transfer complete").await;
}

async fn cmd_stor(ctrl: &mut TcpSocket<'static>, data: &mut TcpSocket<'static>, s: &mut Session, arg: &str, is_appe: bool) {
    if !s.authed || s.anon {
        send_line(ctrl, "530 Permission denied").await;
        return;
    }
    let fp = norm_path(&s.cwd, arg);
    let mode = if is_appe {
        FtpWrMode::Append
    } else if s.rest > 0 {
        FtpWrMode::Rest(s.rest)
    } else {
        FtpWrMode::Trunc
    };
    let seq = rpc_send(StorageCmd::FtpOpenWrite { path: ftp_path_bytes(&fp), mode });
    let ok = rpc_wait(seq).await && ftp_res().0;
    s.rest = 0;
    if !ok {
        send_line(ctrl, "550 Failed to open file").await;
        return;
    }
    send_line(ctrl, "150 Ok to send data").await;
    if !open_data(s, data).await {
        let seq = rpc_send(StorageCmd::FtpCloseWrite);
        let _ = rpc_wait(seq).await;
        send_line(ctrl, "425 No data connection").await;
        return;
    }
    let mut buf = [0u8; 512];
    let mut pending_cr = false;
    loop {
        let n = match data.read(&mut buf).await {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        // ASCII: strip \r of \r\n pairs, hold a trailing \r for the next chunk
        let wlen = if s.type_ascii {
            let mut o = 0usize;
            let start = if pending_cr && n > 0 && buf[0] == b'\n' { 1 } else { 0 };
            pending_cr = false;
            let mut i = start;
            while i < n {
                if buf[i] == b'\r' && i + 1 < n && buf[i + 1] == b'\n' {
                    i += 1;
                    continue;
                }
                buf[o] = buf[i];
                o += 1;
                i += 1;
            }
            if n > start && buf[n - 1] == b'\r' && o > 0 {
                o -= 1;
                pending_cr = true;
            }
            o
        } else {
            n
        };
        if wlen > 0 {
            critical_section::with(|_cs| {
                crate::storage::WBUF.lock(|w| {
                    let mut g = w.borrow_mut();
                    g[..wlen].copy_from_slice(&buf[..wlen]);
                })
            });
            let seq = rpc_send(StorageCmd::FtpWriteChunk(wlen));
            if !rpc_wait(seq).await {
                break;
            }
        }
    }
    let seq = rpc_send(StorageCmd::FtpCloseWrite);
    let _ = rpc_wait(seq).await;
    // graceful FIN on our half (client already closed theirs)
    data.close();
    send_line(ctrl, "226 Transfer complete").await;
}
