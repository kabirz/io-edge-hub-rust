//! Minimal JSON field getters + URL query parsing, port of web_json.h.
//! Extracts scalar values from flat JSON objects without a full parser.

/// `"key":123` / `"key":"str"` extraction from a flat JSON body.
/// Returns None when the key is missing or the value is not the wanted type.
pub fn json_get_i32(body: &[u8], key: &str) -> Option<i32> {
    let v = json_raw_value(body, key)?;
    let s = core::str::from_utf8(v).ok()?;
    // number: optional sign + digits, nothing else
    let t = s.trim();
    if t.is_empty() {
        return None;
    }
    let (neg, digits) = match t.as_bytes()[0] {
        b'-' => (true, &t[1..]),
        b'+' => (false, &t[1..]),
        _ => (false, t),
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let mut n: i64 = 0;
    for b in digits.bytes() {
        n = n * 10 + (b - b'0') as i64;
        if n > i32::MAX as i64 + 1 {
            return None;
        }
    }
    if neg {
        n = -n;
    }
    if n < i32::MIN as i64 || n > i32::MAX as i64 {
        return None;
    }
    Some(n as i32)
}

pub fn json_get_str<'a>(body: &'a [u8], key: &str) -> Option<&'a [u8]> {
    let v = json_raw_value(body, key)?;
    if v.len() < 2 || v[0] != b'"' || v[v.len() - 1] != b'"' {
        return None;
    }
    Some(&v[1..v.len() - 1])
}

fn json_raw_value<'a>(body: &'a [u8], key: &str) -> Option<&'a [u8]> {
    // search for "key" (quoted) followed by optional ws, ':', optional ws
    let kb = key.as_bytes();
    let mut i = 0usize;
    while i + kb.len() + 2 <= body.len() {
        if &body[i..i + kb.len() + 2] == [b'"', b'"'] {
            // placeholder never matches; real check below
        }
        if body[i] == b'"' && body[i + 1..].starts_with(kb) {
            let after = i + 1 + kb.len();
            if after < body.len() && body[after] == b'"' {
                // candidate key; skip ws, expect ':'
                let mut j = after + 1;
                while j < body.len() && (body[j] == b' ' || body[j] == b'\t') {
                    j += 1;
                }
                if j < body.len() && body[j] == b':' {
                    j += 1;
                    while j < body.len() && (body[j] == b' ' || body[j] == b'\t') {
                        j += 1;
                    }
                    // value runs to the next ',' or '}' at depth 0 (strings may contain them)
                    let mut k = j;
                    let mut in_str = false;
                    while k < body.len() {
                        let c = body[k];
                        if in_str {
                            if c == b'\\' {
                                k += 1; // skip escaped char
                            } else if c == b'"' {
                                in_str = false;
                            }
                        } else if c == b'"' {
                            in_str = true;
                        } else if c == b',' || c == b'}' {
                            break;
                        }
                        k += 1;
                    }
                    if k > j {
                        return Some(&body[j..k]);
                    }
                }
            }
        }
        i += 1;
    }
    None
}

/// `?a=1&b=2` query: value of `key` into a byte slice (no URL decoding,
/// matching the C helper).
pub fn url_query_get<'a>(query: &'a [u8], key: &str) -> Option<&'a [u8]> {
    for pair in query.split(|&b| b == b'&') {
        let eq = pair.iter().position(|&b| b == b'=')?;
        if &pair[..eq] == key.as_bytes() && eq + 1 <= pair.len() {
            return Some(&pair[eq + 1..]);
        }
    }
    None
}

/// history_web_name_valid: data_ prefix + [A-Za-z0-9._-], 6..31 chars.
pub fn history_web_name_valid(name: &[u8]) -> bool {
    if name.len() < 6 || name.len() > 31 {
        return false;
    }
    if !name.starts_with(b"data_") {
        return false;
    }
    name.iter().all(|&b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i32_extraction() {
        assert_eq!(json_get_i32(b"{\"index\":0,\"value\":1}", "index"), Some(0));
        assert_eq!(json_get_i32(b"{\"index\":18,\"value\":1}", "value"), Some(1));
        assert_eq!(json_get_i32(b"{\"index\":-1}", "index"), Some(-1));
        assert_eq!(json_get_i32(b"{\"index\":\"x\"}", "index"), None);
        assert_eq!(json_get_i32(b"{\"value\":1}", "index"), None);
        assert_eq!(json_get_i32(b"{\"ts\":946684800}", "ts"), Some(946684_800));
    }

    #[test]
    fn str_extraction() {
        assert_eq!(json_get_str(b"{\"ip\":\"192.168.12.101\"}", "ip"), Some(&b"192.168.12.101"[..]));
        assert_eq!(json_get_str(b"{\"name\":\"data_1.raw\",\"x\":1}", "name"), Some(&b"data_1.raw"[..]));
        assert_eq!(json_get_str(b"{\"ip\":123}", "ip"), None);
    }

    #[test]
    fn query_parsing() {
        let q = b"name=data_1.raw&x=2";
        assert_eq!(url_query_get(q, "name"), Some(&b"data_1.raw"[..]));
        assert_eq!(url_query_get(q, "y"), None);
    }

    #[test]
    fn name_validation() {
        assert!(history_web_name_valid(b"data_0101_000000.raw"));
        assert!(!history_web_name_valid(b"../etc/passwd"));
        assert!(!history_web_name_valid(b"abc"));
        assert!(!history_web_name_valid(b"data_../../x"));
        assert!(!history_web_name_valid(b"no_data_1.raw"));
        assert!(!history_web_name_valid(b"data_a"));
    }
}
