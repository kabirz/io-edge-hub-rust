//! FTP control-channel argument parsing (ftpd.c parse helpers): pure logic,
//! host-testable. The firmware layer wraps the returned octets/port into its
//! endpoint type.
//!
//! Hardening notes (code review): the part count is validated BEFORE any
//! indexing and no capacity-bounded collect is used — a pre-auth line like
//! `PORT 1,2,3` (index OOB) or `PORT 1,2,3,4,5,6,7` (heapless Vec overflow)
/// used to panic the device into a watchdog reboot.

/// Parse "h1,h2,h3,h4,p1,p2" (RFC 959 PORT). Returns (ip octets, port).
pub fn parse_port_arg(arg: &str) -> Option<([u8; 4], u16)> {
    let mut vals = [0u16; 6];
    let mut it = arg.split(',');
    for v in vals.iter_mut() {
        *v = it.next()?.parse().ok()?;
    }
    // a trailing empty part (trailing comma) is tolerated, extra data is not
    if it.next().is_some_and(|s| !s.is_empty()) {
        return None;
    }
    if vals.iter().any(|&v| v > 255) {
        return None;
    }
    Some((
        [vals[0] as u8, vals[1] as u8, vals[2] as u8, vals[3] as u8],
        ((vals[4] as u16) << 8) | vals[5] as u16,
    ))
}

/// Parse "|1|ip|port|" (RFC 2428 EPRT, IPv4 protocol 1 only).
pub fn parse_eprt_arg(arg: &str) -> Option<([u8; 4], u16)> {
    let mut it = arg.split('|');
    it.next()?;
    let proto: u32 = it.next()?.parse().ok()?;
    let ip = it.next()?;
    let port: u16 = it.next()?.parse().ok()?;
    // trailing delimiter is standard, extra data is not
    if it.next().is_some_and(|s| !s.is_empty()) {
        return None;
    }
    if proto != 1 || port == 0 {
        return None;
    }
    let mut a = [0u8; 4];
    let mut iit = ip.split('.');
    for v in a.iter_mut() {
        *v = iit.next()?.parse().ok()?;
    }
    if iit.next().is_some() {
        return None;
    }
    Some((a, port))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_valid() {
        assert_eq!(
            parse_port_arg("192,168,12,101,7,208"),
            Some(([192, 168, 12, 101], 2000))
        );
        assert_eq!(parse_port_arg("10,0,0,1,0,1"), Some(([10, 0, 0, 1], 1)));
    }

    #[test]
    fn port_rejects_wrong_part_counts_without_panic() {
        assert_eq!(parse_port_arg("1,2,3"), None); // was: index OOB panic
        assert_eq!(parse_port_arg("1"), None);
        assert_eq!(parse_port_arg(""), None);
        assert_eq!(parse_port_arg("1,2,3,4,5,6,7"), None); // was: collect overflow panic
        // a single trailing comma is tolerated (empty part), matching the
        // lenient C sscanf-style parsing
        assert_eq!(parse_port_arg("1,2,3,4,5,6,"), parse_port_arg("1,2,3,4,5,6"));
        assert_eq!(parse_port_arg("1,2,3,4,5,,6"), None); // empty part mid-list
    }

    #[test]
    fn port_rejects_bad_values() {
        assert_eq!(parse_port_arg("a,b,c,d,e,f"), None);
        assert_eq!(parse_port_arg("256,0,0,0,0,1"), None);
        assert_eq!(parse_port_arg("99999,0,0,0,0,1"), None); // not u16
    }

    #[test]
    fn eprt_valid_with_trailing_delimiter() {
        assert_eq!(
            parse_eprt_arg("|1|132.235.1.2|6275|"),
            Some(([132, 235, 1, 2], 6275))
        );
    }

    #[test]
    fn eprt_rejects_malformed_without_panic() {
        assert_eq!(parse_eprt_arg("|1|1.2.3|21|"), None); // 3 octets: was OOB panic
        assert_eq!(parse_eprt_arg("|1|1.2.3.4.5|21|"), None); // 5 octets: was overflow panic
        assert_eq!(parse_eprt_arg("|2|::1|21|"), None); // IPv6 proto unsupported
        assert_eq!(parse_eprt_arg("|1|1.2.3.4|0|"), None); // port 0
        assert_eq!(parse_eprt_arg("1|1.2.3.4|21|"), None); // missing leading |
    }
}
