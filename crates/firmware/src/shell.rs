//! Debug shell on USART1 (shares the console with the logger), 1:1 port of
//! src/sys/shell.c.
//!
//! RX: DMA2 circular ring (uart_raw) polled by this task — hardware drains
//! the DR during the millisecond PRIMASK freezes of NOR flash ops, so UART
//! bursts never hit the 1-byte-deep overrun a register-level RXNE interrupt
//! would suffer in this design.
//! TX: log::raw / log::line so output never interleaves with log lines.
//!
//! Line editing: echo, backspace, CRLF convergence, cursor left/right,
//! insert/delete mid-line, history up/down, context-aware Tab completion.

use io_edge_hub_proto::regmap as rm;

use crate::appstate::{Hooks, REGS, version};
use crate::{log, systime, uart_raw};

const LINE_MAX: usize = 96; // includes NUL, like SH_LINE_MAX
const HIST_MAX: usize = 8;
const ARG_MAX: usize = 6;

/// RX counters (UDP debug cmd 0xFA): dma-consumed / task-processed.
pub static RX_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RX_GOT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Wait for one byte from the DMA ring (2 ms poll; PRIMASK freezes only add
/// latency — the DMA has already captured the bytes).
async fn getchar() -> u8 {
    loop {
        let tail = (RX_COUNT.load(core::sync::atomic::Ordering::Relaxed) as usize)
            % uart_raw::RX_RING;
        if uart_raw::rx_available(tail) > 0 {
            RX_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return uart_raw::rx_peek(tail);
        }
        embassy_time::Timer::after_millis(2).await;
    }
}

fn prompt() {
    log::raw(b"io> ");
}

/// Whole-line redraw: \r + prompt + line + clear-tail, cursor back to pos.
fn redraw(line: &[u8], n: usize, pos: usize) {
    let mut buf = [0u8; LINE_MAX + 16];
    let mut m = 0usize;
    buf[..5].copy_from_slice(b"\rio> ");
    m = 5 + n;
    buf[5..m].copy_from_slice(&line[..n]);
    buf[m..m + 3].copy_from_slice(b"\x1b[K");
    m += 3;
    if pos < n {
        let mut s = heapless::String::<12>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("\x1b[{}D", n - pos),
        );
        buf[m..m + s.len()].copy_from_slice(s.as_bytes());
        m += s.len();
    }
    log::raw(&buf[..m]);
}

// ==================== history ====================

struct Hist {
    lines: [[u8; LINE_MAX]; HIST_MAX], // NUL-terminated
    len: u8,
    newest: u8,
}

impl Hist {
    fn new() -> Self {
        Self { lines: [[0; LINE_MAX]; HIST_MAX], len: 0, newest: 0 }
    }

    fn store(&mut self, line: &[u8]) {
        let cur = &self.lines[self.newest as usize];
        if self.len != 0 && cur[..line.len()] == *line && cur[line.len()] == 0 {
            return; // consecutive duplicate
        }
        self.newest = (self.newest + 1) % HIST_MAX as u8;
        let slot = &mut self.lines[self.newest as usize];
        *slot = [0; LINE_MAX];
        let n = line.len().min(LINE_MAX - 1);
        slot[..n].copy_from_slice(&line[..n]);
        if self.len < HIST_MAX as u8 {
            self.len += 1;
        }
    }

    /// nav: 0 = newest; None past the oldest.
    fn get(&self, nav: u8) -> Option<&[u8]> {
        if nav >= self.len {
            return None;
        }
        let idx = (self.newest + HIST_MAX as u8 - nav) % HIST_MAX as u8;
        let slot = &self.lines[idx as usize];
        let n = slot.iter().position(|&b| b == 0).unwrap_or(LINE_MAX);
        Some(&slot[..n])
    }
}

// ==================== Tab completion (command tree) ====================

struct Cmd {
    name: &'static str,
    sub: &'static [Cmd],
}
const LEAF: &[Cmd] = &[];

const IO_DO_CMDS: &[Cmd] = &[Cmd { name: "set", sub: LEAF }];
const IO_RS485_CMDS: &[Cmd] = &[
    Cmd { name: "baud", sub: LEAF },
    Cmd { name: "sid", sub: LEAF },
];
const IO_CAN_CMDS: &[Cmd] = &[
    Cmd { name: "id", sub: LEAF },
    Cmd { name: "bps", sub: LEAF },
];
const IO_CMDS: &[Cmd] = &[
    Cmd { name: "help", sub: LEAF },
    Cmd { name: "info", sub: LEAF },
    Cmd { name: "di", sub: LEAF },
    Cmd { name: "do", sub: IO_DO_CMDS },
    Cmd { name: "ai", sub: LEAF },
    Cmd { name: "rs485", sub: IO_RS485_CMDS },
    Cmd { name: "can", sub: IO_CAN_CMDS },
    Cmd { name: "ip", sub: LEAF },
    Cmd { name: "reg", sub: LEAF },
    Cmd { name: "save", sub: LEAF },
    Cmd { name: "factory", sub: LEAF },
];
const ROOT_CMDS: &[Cmd] = &[
    Cmd { name: "help", sub: LEAF },
    Cmd { name: "tasks", sub: LEAF },
    Cmd { name: "ps", sub: LEAF },
    Cmd { name: "reboot", sub: LEAF },
    Cmd { name: "io", sub: IO_CMDS },
];

/// Walk the command tree over the typed complete words; returns the
/// candidate table for the trailing (possibly empty) word, or None in the
/// parameter zone / off-tree (no completion).
fn complete_level(line: &[u8], n: usize) -> (Option<&'static [Cmd]>, usize, usize) {
    // returns (table, last-word-start, last-word-len)
    let mut tbl: Option<&[Cmd]> = Some(ROOT_CMDS);
    let mut i = 0usize;
    while i < n {
        while i < n && (line[i] == b' ' || line[i] == b'\t') {
            i += 1;
        }
        if i == n {
            break;
        }
        let w = i;
        while i < n && line[i] != b' ' && line[i] != b'\t' {
            i += 1;
        }
        if i == n {
            return (tbl, w, i - w); // trailing word
        }
        if let Some(t) = tbl {
            let mut found = false;
            for c in t {
                if c.name.as_bytes() == &line[w..i] {
                    tbl = if c.sub.is_empty() { None } else { Some(c.sub) };
                    found = true;
                    break;
                }
            }
            if !found {
                tbl = None;
            }
        }
    }
    (tbl, n, 0) // line end / trailing blank
}

fn complete(line: &mut [u8; LINE_MAX], n: &mut usize) {
    static PLACEHOLDER: Cmd = Cmd { name: "", sub: LEAF };
    let (tbl, wstart, wlen0) = complete_level(line, *n);
    let Some(tbl) = tbl else { return };
    let mut wlen = wlen0;
    let mut cand: [&'static Cmd; 16] = [&PLACEHOLDER; 16]; // overwritten below
    let mut ncand = 0usize;
    for c in tbl {
        if c.name.as_bytes().len() >= wlen
            && &c.name.as_bytes()[..wlen] == &line[wstart..wstart + wlen]
            && ncand < 16
        {
            cand[ncand] = c;
            ncand += 1;
        }
    }
    if ncand == 0 {
        return;
    }
    if ncand == 1 {
        let old = *n;
        for &b in &cand[0].name.as_bytes()[wlen..] {
            if *n + 2 >= LINE_MAX {
                break;
            }
            line[*n] = b;
            *n += 1;
        }
        line[*n] = b' ';
        *n += 1;
        line[*n] = 0;
        log::raw(&line[old..*n]);
        return;
    }
    // multiple: extend the longest common prefix
    let mut lcp = cand[0].name.len();
    for c in cand[..ncand].iter().skip(1) {
        let mut j = 0;
        while j < lcp
            && j < c.name.len()
            && cand[0].name.as_bytes()[j] == c.name.as_bytes()[j]
        {
            j += 1;
        }
        lcp = j;
    }
    while wlen < lcp && *n + 1 < LINE_MAX {
        line[*n] = cand[0].name.as_bytes()[wlen];
        *n += 1;
        wlen += 1;
    }
    line[*n] = 0;

    log::raw(b"\r\n");
    let mut row = heapless::String::<128>::new();
    for (i, c) in cand[..ncand].iter().enumerate() {
        let _ = core::fmt::write(&mut row, core::format_args!("{}{}", if i > 0 { "  " } else { "" }, c.name));
    }
    log::line(&row);
    prompt();
    log::raw(&line[..*n]);
}

// ==================== commands ====================

fn regs_snapshot() -> (heapless::Vec<u16, 64>, heapless::Vec<u16, 16>) {
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            let g = r.borrow();
            let mut h = heapless::Vec::new();
            h.extend_from_slice(&g.holding).ok();
            let mut i = heapless::Vec::new();
            i.extend_from_slice(&g.input).ok();
            (h, i)
        })
    })
}

fn reg_write(addr: u16, value: u16) -> bool {
    critical_section::with(|_cs| {
        REGS.lock(|r| {
            r.borrow_mut()
                .io_write_holding(addr, value, &mut Hooks)
                .is_ok()
        })
    })
}

fn cmd_help() {
    log::line("commands:");
    log::line("  help    this help");
    log::line("  tasks   task list (state / priority / min stack)");
    log::line("  reboot  graceful reboot (history sync + ~3s)");
    log::line("  io      io-edge-hub debug commands ('io help')");
}

/// Static task table (embassy tasks have no introspection; same shape as
/// uxTaskGetSystemState output).
const TASK_TABLE: &[(&str, char, u16, u16)] = &[
    ("embassy-main", 'B', 0, 2048),
    ("hb", 'B', 0, 512),
    ("net-poll", 'B', 0, 1024),
    ("udp-cfg", 'B', 0, 512),
    ("storage", 'B', 1, 1024),
    ("mbtcp1", 'B', 1, 512),
    ("mbtcp2", 'B', 1, 512),
    ("http1", 'B', 1, 1024),
    ("http2", 'B', 1, 1024),
    ("ftp1", 'B', 1, 1024),
    ("ftp2", 'B', 1, 1024),
    ("ftp3", 'B', 1, 1024),
    ("rtu", 'B', 1, 512),
    ("sh", 'R', 1, 640),
];

fn cmd_tasks() {
    log::line("task              st  prio  stack  num");
    for (i, (name, st, prio, stack)) in TASK_TABLE.iter().enumerate() {
        let mut s = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("{:<16} {}   {:<4}  {:<5}  {}", name, st, prio, stack, i),
        );
        log::line(&s);
    }
    log::line("st: X=running R=ready B=blocked S=suspended; stack = min free (words)");
}

fn cmd_reboot() {
    log::line("rebooting (history sync + ~3s)...");
    crate::storage::QUEUE
        .try_send(crate::storage::StorageCmd::Sync)
        .ok();
    crate::appstate::set_reboot_status(true);
}

fn cmd_io_info() {
    let (h, _) = regs_snapshot();
    let mac = crate::net::current_mac();
    let g = |i: usize| h.get(i).copied().unwrap_or(0);
    let mut s = heapless::String::<96>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!("version : {}", version::FW_VERSION),
    );
    log::line(&s);
    let mut s = heapless::String::<96>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!("build   : {}", version::FW_BUILD),
    );
    log::line(&s);
    log::line("board   : io_edge_f407vet6");
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "mac     : {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        ),
    );
    log::line(&s);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "ip      : {}.{}.{}.{}/24",
            g(rm::HOLDING_IP_OCTET1_IDX),
            g(rm::HOLDING_IP_OCTET1_IDX + 1),
            g(rm::HOLDING_IP_OCTET1_IDX + 2),
            g(rm::HOLDING_IP_OCTET1_IDX + 3)
        ),
    );
    log::line(&s);
    let mut s = heapless::String::<24>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!("link    : {}", if crate::net::net_link_up() { "up" } else { "down" }),
    );
    log::line(&s);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "rs485   : {} bps, slave id {} (8N1)",
            g(rm::HOLDING_RS485_BAUDRATE_IDX),
            g(rm::HOLDING_SLAVE_ID_IDX)
        ),
    );
    log::line(&s);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "can     : id 0x{:03x}, {} kbit/s",
            g(rm::HOLDING_CAN_ID_IDX),
            g(rm::HOLDING_CAN_BAUDRATE_IDX)
        ),
    );
    log::line(&s);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "uptime  : {} s",
            embassy_time::Instant::now().as_ticks() / embassy_time::TICK_HZ
        ),
    );
    log::line(&s);

    // RTC keeps UTC; display with the +8h offset (epoch -> civil date)
    let lt = systime::now_epoch() as i64 + 8 * 3600;
    let days = lt.div_euclid(86_400);
    let secs = lt.rem_euclid(86_400);
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = era * 400 + yoe as i64 + if m <= 2 { 1 } else { 0 };
    let mut s = heapless::String::<64>::new();
    if y >= 2020 {
        let _ = core::fmt::write(
            &mut s,
            core::format_args!(
                "time    : {:04}-{:02}-{:02} {:02}:{:02}:{:02} ({})",
                y,
                m,
                d,
                secs / 3600,
                (secs / 60) % 60,
                secs % 60,
                systime::now_epoch()
            ),
        );
    } else {
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("time    : 1970-01-01 00:00:00 ({})", systime::now_epoch()),
        );
    }
    log::line(&s);
}

fn cmd_io_di() {
    let (h, i) = regs_snapshot();
    let di = i.get(rm::INPUT_DI_IDX).copied().unwrap_or(0);
    let en = h.get(rm::HOLDING_DI_ENABLE_IDX).copied().unwrap_or(0);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(&mut s, core::format_args!("DI: 0x{:04x} (enable: 0x{:04x})", di, en));
    log::line(&s);
    let mut row = 0;
    while row < 16 {
        let mut s = heapless::String::<48>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!(
                "DI{:<3}-{:<3}: {} {} {} {} {} {} {} {}",
                row + 1,
                row + 8,
                (di >> row) & 1,
                (di >> (row + 1)) & 1,
                (di >> (row + 2)) & 1,
                (di >> (row + 3)) & 1,
                (di >> (row + 4)) & 1,
                (di >> (row + 5)) & 1,
                (di >> (row + 6)) & 1,
                (di >> (row + 7)) & 1
            ),
        );
        log::line(&s);
        row += 8;
    }
}

fn cmd_io_do() {
    let (h, _) = regs_snapshot();
    let do_v = h.get(rm::HOLDING_DO_IDX).copied().unwrap_or(0) & 0xFF;
    let mut s = heapless::String::<32>::new();
    let _ = core::fmt::write(&mut s, core::format_args!("DO: 0x{:02x}", do_v));
    log::line(&s);
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "DO1 -8  : {} {} {} {} {} {} {} {}",
            do_v & 1,
            (do_v >> 1) & 1,
            (do_v >> 2) & 1,
            (do_v >> 3) & 1,
            (do_v >> 4) & 1,
            (do_v >> 5) & 1,
            (do_v >> 6) & 1,
            (do_v >> 7) & 1
        ),
    );
    log::line(&s);
}

fn cmd_io_do_set(ch_s: &str, val_s: &str) {
    let Some(ch) = parse_u32(ch_s) else {
        log::line(&format2("invalid channel: ", ch_s));
        return;
    };
    if !(1..=8).contains(&ch) {
        log::line("invalid channel (1-8)");
        return;
    }
    let Some(v) = parse_u32(val_s) else {
        log::line("invalid value (0/1)");
        return;
    };
    if v > 1 {
        log::line("invalid value (0/1)");
        return;
    }
    let ok = critical_section::with(|_cs| {
        REGS.lock(|r| {
            r.borrow_mut()
                .io_write_do_bit(ch as u16 - 1, v != 0, &mut Hooks)
                .is_ok()
        })
    });
    if !ok {
        log::line("write failed");
        return;
    }
    let (h, _) = regs_snapshot();
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!(
            "DO{} = {} (DO: 0x{:02x})",
            ch,
            v,
            h.get(rm::HOLDING_DO_IDX).copied().unwrap_or(0) & 0xFF
        ),
    );
    log::line(&s);
}

fn cmd_io_ai() {
    let (h, i) = regs_snapshot();
    let units = ["mA", "mA", "V", "V"];
    let mut s = heapless::String::<32>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!("AI enable: 0x{:1x}", h.get(rm::HOLDING_AI_ENABLE_IDX).copied().unwrap_or(0) & 0x0F),
    );
    log::line(&s);
    for k in 0..4 {
        let raw = i.get(rm::INPUT_AI0_IDX + k).copied().unwrap_or(0);
        let mut s = heapless::String::<48>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("AI{}: {:5}.{:02} {} (raw {})", k + 1, raw / 100, raw % 100, units[k], raw),
        );
        log::line(&s);
    }
}

fn cmd_io_reg(args: &[&str]) {
    let (h, i) = regs_snapshot();
    if args.is_empty() {
        let mut s = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("holding registers ({}):", rm::MODBUS_HOLDING_REGISTER_NUMBERS),
        );
        log::line(&s);
        let mut c = 0usize;
        while c < rm::MODBUS_HOLDING_REGISTER_NUMBERS {
            let mut s = heapless::String::<96>::new();
            for j in c..(c + 6).min(rm::MODBUS_HOLDING_REGISTER_NUMBERS) {
                let _ = core::fmt::write(
                    &mut s,
                    core::format_args!("{}0x{:02x}={}", if j % 6 > 0 { " " } else { "" }, j, h.get(j).copied().unwrap_or(0)),
                );
            }
            log::line(&s);
            c += 6;
        }
        let mut s = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("input registers ({}):", rm::MODBUS_INPUT_REGISTER_NUMBERS),
        );
        log::line(&s);
        let mut c = 0usize;
        while c < rm::MODBUS_INPUT_REGISTER_NUMBERS {
            let mut s = heapless::String::<96>::new();
            for j in c..(c + 6).min(rm::MODBUS_INPUT_REGISTER_NUMBERS) {
                let _ = core::fmt::write(
                    &mut s,
                    core::format_args!("{}0x{:02x}={}", if j % 6 > 0 { " " } else { "" }, j, i.get(j).copied().unwrap_or(0)),
                );
            }
            log::line(&s);
            c += 6;
        }
        return;
    }
    let Some(addr) = parse_u32(args[0]) else {
        log::line(&format2("invalid addr: ", args[0]));
        return;
    };
    if addr as usize >= rm::MODBUS_HOLDING_REGISTER_NUMBERS {
        log::line(&format2("invalid addr: ", args[0]));
        return;
    }
    if args.len() == 1 {
        let mut s = heapless::String::<48>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!("holding[0x{:02x}] = {}", addr, h.get(addr as usize).copied().unwrap_or(0)),
        );
        log::line(&s);
        return;
    }
    let Some(v) = parse_u32(args[1]) else {
        log::line("invalid value (0-65535)");
        return;
    };
    if v > 0xFFFF {
        log::line("invalid value (0-65535)");
        return;
    }
    if !reg_write(addr as u16, v as u16) {
        log::line("write failed");
        return;
    }
    let (h, _) = regs_snapshot();
    let mut s = heapless::String::<48>::new();
    let _ = core::fmt::write(
        &mut s,
        core::format_args!("holding[0x{:02x}] = {}", addr, h.get(addr as usize).copied().unwrap_or(0)),
    );
    log::line(&s);
}

fn cmd_io_help() {
    log::line("io                     IO/config overview");
    log::line("io info                version / mac / ip / link / uptime");
    log::line("io di                  DI1-16 status");
    log::line("io do [set ch 0|1]     DO1-8 status / control");
    log::line("io ai                  AI1-4 values (mA / V)");
    log::line("io rs485 [baud n|sid n]");
    log::line("io can [id n|bps n]");
    log::line("io ip a.b.c.d          static ip (saved)");
    log::line("io reg [addr [value]]  register dump / read / write");
    log::line("io save                persist parameters");
    log::line("io factory             factory reset + reboot");
}

fn io_dispatch(args: &[&str]) {
    let (h, input) = regs_snapshot();
    let g = |i: usize| h.get(i).copied().unwrap_or(0);
    let gi = |i: usize| input.get(i).copied().unwrap_or(0);
    if args.is_empty() {
        let mut s = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!(
                "DI: 0x{:04x}  DO: 0x{:02x}  AI: {} {} {} {}",
                gi(rm::INPUT_DI_IDX),
                g(rm::HOLDING_DO_IDX) & 0xFF,
                gi(rm::INPUT_AI0_IDX),
                gi(rm::INPUT_AI0_IDX + 1),
                gi(rm::INPUT_AI0_IDX + 2),
                gi(rm::INPUT_AI0_IDX + 3)
            ),
        );
        log::line(&s);
        let mut s = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut s,
            core::format_args!(
                "rs485: {} bps sid {} | can: 0x{:03x} {} kbit/s",
                g(rm::HOLDING_RS485_BAUDRATE_IDX),
                g(rm::HOLDING_SLAVE_ID_IDX),
                g(rm::HOLDING_CAN_ID_IDX),
                g(rm::HOLDING_CAN_BAUDRATE_IDX)
            ),
        );
        log::line(&s);
        log::line("('io help' for subcommands)");
        return;
    }
    match args[0] {
        "help" => cmd_io_help(),
        "info" => cmd_io_info(),
        "di" => cmd_io_di(),
        "do" => {
            if args.len() >= 4 && args[1] == "set" {
                cmd_io_do_set(args[2], args[3]);
            } else {
                cmd_io_do();
            }
        }
        "ai" => cmd_io_ai(),
        "rs485" => {
            if args.len() >= 3 && args[1] == "baud" {
                if let Some(v) = parse_u32(args[2]) {
                    if (1200..=115200).contains(&v) {
                        reg_write(rm::HOLDING_RS485_BAUDRATE_IDX as u16, v as u16);
                        let (h, _) = regs_snapshot();
                        let mut s = heapless::String::<96>::new();
                        let _ = core::fmt::write(
                            &mut s,
                            core::format_args!(
                                "rs485 baud -> {} (reboot to apply, 'io save' to persist)",
                                h.get(rm::HOLDING_RS485_BAUDRATE_IDX).copied().unwrap_or(0)
                            ),
                        );
                        log::line(&s);
                        return;
                    }
                }
            } else if args.len() >= 3 && args[1] == "sid" {
                if let Some(v) = parse_u32(args[2]) {
                    if (1..=247).contains(&v) {
                        reg_write(rm::HOLDING_SLAVE_ID_IDX as u16, v as u16);
                        let (h, _) = regs_snapshot();
                        let mut s = heapless::String::<96>::new();
                        let _ = core::fmt::write(
                            &mut s,
                            core::format_args!(
                                "slave id -> {} (reboot to apply, 'io save' to persist)",
                                h.get(rm::HOLDING_SLAVE_ID_IDX).copied().unwrap_or(0)
                            ),
                        );
                        log::line(&s);
                        return;
                    }
                }
            }
            let mut s = heapless::String::<64>::new();
            let _ = core::fmt::write(
                &mut s,
                core::format_args!(
                    "rs485: {} bps, slave id {} (8N1)",
                    g(rm::HOLDING_RS485_BAUDRATE_IDX),
                    g(rm::HOLDING_SLAVE_ID_IDX)
                ),
            );
            log::line(&s);
            log::line("(changes take effect after reboot)");
        }
        "can" => {
            if args.len() >= 3 && args[1] == "id" {
                if let Some(v) = parse_u32(args[2]) {
                    if (1..=0x7FF).contains(&v) {
                        reg_write(rm::HOLDING_CAN_ID_IDX as u16, v as u16);
                        let (h, _) = regs_snapshot();
                        let mut s = heapless::String::<96>::new();
                        let _ = core::fmt::write(
                            &mut s,
                            core::format_args!(
                                "can id -> 0x{:03x} (reboot to apply, 'io save' to persist)",
                                h.get(rm::HOLDING_CAN_ID_IDX).copied().unwrap_or(0)
                            ),
                        );
                        log::line(&s);
                        return;
                    }
                }
            } else if args.len() >= 3 && args[1] == "bps" {
                if let Some(v) = parse_u32(args[2]) {
                    if [50u32, 100, 125, 250, 500, 800, 1000].contains(&v) {
                        reg_write(rm::HOLDING_CAN_BAUDRATE_IDX as u16, v as u16);
                        let (h, _) = regs_snapshot();
                        let mut s = heapless::String::<96>::new();
                        let _ = core::fmt::write(
                            &mut s,
                            core::format_args!(
                                "can bps -> {} kbit/s (reboot to apply, 'io save' to persist)",
                                h.get(rm::HOLDING_CAN_BAUDRATE_IDX).copied().unwrap_or(0)
                            ),
                        );
                        log::line(&s);
                        return;
                    }
                }
            }
            let mut s = heapless::String::<64>::new();
            let _ = core::fmt::write(
                &mut s,
                core::format_args!(
                    "can: id 0x{:03x}, {} kbit/s",
                    g(rm::HOLDING_CAN_ID_IDX),
                    g(rm::HOLDING_CAN_BAUDRATE_IDX)
                ),
            );
            log::line(&s);
            log::line("(changes take effect after reboot)");
        }
        "ip" => {
            if args.len() < 2 {
                log::line("invalid ip: ");
                return;
            }
            let Some(ip) = parse_ip(args[1]) else {
                log::line(&format2("invalid ip: ", args[1]));
                return;
            };
            // ip_addr_valid: no 0/127 first octet, not 255.255.255.255, host != 0/255
            if ip[0] == 0 || ip[0] == 127 || ip[0] >= 255 || ip[3] == 0 || ip[3] == 255 {
                log::line(&format2("invalid ip: ", args[1]));
                return;
            }
            reg_write(rm::HOLDING_IP_OCTET1_IDX as u16, ip[0] as u16);
            reg_write((rm::HOLDING_IP_OCTET1_IDX + 1) as u16, ip[1] as u16);
            reg_write((rm::HOLDING_IP_OCTET1_IDX + 2) as u16, ip[2] as u16);
            reg_write((rm::HOLDING_IP_OCTET1_IDX + 3) as u16, ip[3] as u16);
            crate::storage::CTRL_QUEUE
                .try_send(crate::storage::StorageCmd::CfgSave)
                .ok();
            let mut s = heapless::String::<64>::new();
            let _ = core::fmt::write(
                &mut s,
                core::format_args!("ip -> {}.{}.{}.{} (saved, reboot to apply)", ip[0], ip[1], ip[2], ip[3]),
            );
            log::line(&s);
        }
        "reg" => cmd_io_reg(&args[1..]),
        "save" => {
            crate::storage::CTRL_QUEUE
                .try_send(crate::storage::StorageCmd::CfgSave)
                .ok();
            log::line("parameters saved");
        }
        "factory" => {
            crate::storage::CTRL_QUEUE
                .try_send(crate::storage::StorageCmd::CfgEraseAll)
                .ok();
            crate::appstate::set_reboot_status(true);
            log::line("factory reset done, rebooting (defaults after reboot)");
        }
        other => {
            log::line(&format3("unknown io command: ", other, " ('io help')"));
        }
    }
}

fn dispatch(line: &[u8], n: usize) {
    // split on blanks (in-place like sh_split)
    let mut argv: [&str; ARG_MAX] = [""; ARG_MAX];
    let mut argc = 0usize;
    let mut i = 0usize;
    while i < n && argc < ARG_MAX {
        while i < n && (line[i] == b' ' || line[i] == b'\t') {
            i += 1;
        }
        if i >= n {
            break;
        }
        let w = i;
        while i < n && line[i] != b' ' && line[i] != b'\t' {
            i += 1;
        }
        argv[argc] = core::str::from_utf8(&line[w..i]).unwrap_or("");
        argc += 1;
    }
    if argc == 0 {
        return;
    }
    match argv[0] {
        "help" => cmd_help(),
        "tasks" | "ps" => cmd_tasks(),
        "reboot" => cmd_reboot(),
        "io" => io_dispatch(&argv[1..argc]),
        other => log::line(&format3("unknown command: ", other, " (help)")),
    }
}

// small helpers: format into heapless strings (no alloc in no_std)
fn format2(prefix: &str, mid: &str) -> heapless::String<128> {
    let mut s = heapless::String::<128>::new();
    let _ = core::fmt::write(&mut s, core::format_args!("{}{}", prefix, mid));
    s
}

fn format3(prefix: &str, mid: &str, suffix: &str) -> heapless::String<128> {
    let mut s = heapless::String::<128>::new();
    let _ = core::fmt::write(&mut s, core::format_args!("{}{}{}", prefix, mid, suffix));
    s
}

fn parse_u32(s: &str) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    let (radix, digits) = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        (16, h)
    } else {
        (10, s)
    };
    if digits.is_empty() {
        return None;
    }
    u32::from_str_radix(digits, radix).ok()
}

fn parse_ip(s: &str) -> Option<[u8; 4]> {
    let mut ip = [0u8; 4];
    let mut it = s.split('.');
    for slot in ip.iter_mut() {
        let p = it.next()?;
        if p.is_empty() || p.len() > 3 || !p.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        *slot = p.parse::<u8>().ok()?;
    }
    if it.next().is_some() {
        return None;
    }
    Some(ip)
}

// ==================== task: line editor ====================

/// Line editor over the DMA RX ring (uart_raw::init ran from main before
/// any logging; RX needs no interrupt enable at all).
#[embassy_executor::task]
pub async fn shell_task() {
    let mut line = [0u8; LINE_MAX];
    let mut draft = [0u8; LINE_MAX];
    let mut n = 0usize;
    let mut pos = 0usize;
    let mut esc = 0u8;
    let mut nav: i8 = -1;
    let mut prev_cr = false;
    let mut hist = Hist::new();

    log::line("");
    log::line("shell ready (help for commands)");
    prompt();

    loop {
        let c = getchar().await;
        let mut arrow = false;

        if esc != 0 {
            if esc == 1 && (c == b'[' || c == b'O') {
                esc = 2;
                continue;
            }
            if esc == 2 && (c.is_ascii_digit() || c == b';') {
                continue;
            }
            arrow = (b'A'..=b'D').contains(&c);
            esc = 0;
            if !arrow {
                continue;
            }
        } else if c == 0x1B {
            esc = 1;
            continue;
        }

        if arrow {
            match c {
                b'C' => {
                    if pos < n {
                        pos += 1;
                        log::raw(b"\x1b[C");
                    }
                }
                b'D' => {
                    if pos > 0 {
                        pos -= 1;
                        log::raw(b"\x1b[D");
                    }
                }
                b'A' => {
                    if nav < 0 {
                        draft = line;
                        nav = 0;
                    } else if (nav + 1) < hist.len as i8 {
                        nav += 1;
                    }
                    if let Some(h) = hist.get(nav as u8) {
                        let l = h.len();
                        line = [0; LINE_MAX];
                        line[..l].copy_from_slice(h);
                        n = l;
                        pos = n;
                        redraw(&line, n, pos);
                    }
                }
                b'B' => {
                    if nav < 0 {
                        continue;
                    }
                    nav -= 1;
                    let src: &[u8] = if nav < 0 {
                        &draft[..draft.iter().position(|&b| b == 0).unwrap_or(LINE_MAX)]
                    } else if let Some(h) = hist.get(nav as u8) {
                        h
                    } else {
                        continue;
                    };
                    let l = src.len();
                    line = [0; LINE_MAX];
                    line[..l].copy_from_slice(src);
                    n = l;
                    pos = n;
                    redraw(&line, n, pos);
                }
                _ => {}
            }
            continue;
        }

        if c == b'\n' && prev_cr {
            prev_cr = false;
            continue;
        }
        prev_cr = false;
        if c == b'\r' || c == b'\n' {
            prev_cr = c == b'\r';
            log::raw(b"\r\n");
            if n != 0 {
                hist.store(&line[..n]);
                nav = -1;
                dispatch(&line, n);
                n = 0;
                pos = 0;
                line[0] = 0;
            }
            prompt();
        } else if c == 0x08 || c == 0x7F {
            if pos > 0 {
                line.copy_within(pos..n, pos - 1);
                n -= 1;
                pos -= 1;
                line[n] = 0;
                if pos == n {
                    log::raw(b"\x08 \x08");
                } else {
                    redraw(&line, n, pos);
                }
            }
        } else if c == b'\t' {
            if pos < n {
                let mut s = heapless::String::<12>::new();
                let _ = core::fmt::write(&mut s, core::format_args!("\x1b[{}C", n - pos));
                log::raw(s.as_bytes());
                pos = n;
            }
            complete(&mut line, &mut n);
            pos = n;
        } else if (0x20..0x7F).contains(&c) && n + 2 < LINE_MAX {
            if pos == n {
                line[n] = c;
                n += 1;
                pos = n;
                line[n] = 0;
                log::raw(&[c]);
            } else {
                line.copy_within(pos..n, pos + 1);
                line[pos] = c;
                pos += 1;
                n += 1;
                line[n] = 0;
                redraw(&line, n, pos);
            }
        }
    }
}
