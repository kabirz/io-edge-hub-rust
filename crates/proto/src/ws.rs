//! WebSocket support: SHA-1 + base64 for the handshake (ws.c), server-frame
//! encoding and client-frame parsing with the same limits/semantics.

// ==================== SHA-1 ====================

pub struct Sha1 {
    h: [u32; 5],
    total: u64,
    buf: [u8; 64],
    buf_len: usize,
}

impl Sha1 {
    pub const fn new() -> Self {
        Self {
            h: [0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0],
            total: 0,
            buf: [0; 64],
            buf_len: 0,
        }
    }

    pub fn update(&mut self, mut data: &[u8]) {
        self.total = self.total.wrapping_add(data.len() as u64);
        if self.buf_len > 0 {
            let take = (64 - self.buf_len).min(data.len());
            self.buf[self.buf_len..self.buf_len + take].copy_from_slice(&data[..take]);
            self.buf_len += take;
            data = &data[take..];
            if self.buf_len == 64 {
                let block = self.buf;
                self.block(&block);
                self.buf_len = 0;
            }
        }
        while data.len() >= 64 {
            let mut block = [0u8; 64];
            block.copy_from_slice(&data[..64]);
            self.block(&block);
            data = &data[64..];
        }
        if !data.is_empty() {
            self.buf[..data.len()].copy_from_slice(data);
            self.buf_len = data.len();
        }
    }

    fn block(&mut self, p: &[u8; 64]) {
        let mut w = [0u32; 80];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([p[4 * i], p[4 * i + 1], p[4 * i + 2], p[4 * i + 3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = self.h;
        for (i, wi) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | ((!b) & d), 0x5A827999),
                20..=39 => (b ^ c ^ d, 0x6ED9EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1BBCDC),
                _ => (b ^ c ^ d, 0xCA62C1D6),
            };
            let tmp = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(*wi);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = tmp;
        }
        self.h[0] = self.h[0].wrapping_add(a);
        self.h[1] = self.h[1].wrapping_add(b);
        self.h[2] = self.h[2].wrapping_add(c);
        self.h[3] = self.h[3].wrapping_add(d);
        self.h[4] = self.h[4].wrapping_add(e);
    }

    pub fn finalize(mut self) -> [u8; 20] {
        let bits = self.total.wrapping_mul(8);
        self.update(&[0x80]);
        while self.buf_len != 56 {
            self.update(&[0]);
        }
        // manually append length without counting it
        let mut block = self.buf;
        block[56..64].copy_from_slice(&bits.to_be_bytes());
        self.block(&block);
        let mut out = [0u8; 20];
        for i in 0..5 {
            out[4 * i..4 * i + 4].copy_from_slice(&self.h[i].to_be_bytes());
        }
        out
    }
}

// ==================== base64 (encode only, handshake) ====================

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encode into `dst`; returns the encoded length. 20 bytes -> 28 chars
/// (standard base64 with '=' padding, like the C b64_encode).
pub fn b64_encode(src: &[u8], dst: &mut [u8]) -> usize {
    const PAD: u8 = b'=';
    let mut n = 0usize;
    for chunk in src.chunks(3) {
        let b = [
            chunk[0],
            *chunk.get(1).unwrap_or(&0),
            *chunk.get(2).unwrap_or(&0),
        ];
        let v = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
        let chars = [
            B64[((v >> 18) & 63) as usize],
            B64[((v >> 12) & 63) as usize],
            B64[((v >> 6) & 63) as usize],
            B64[(v & 63) as usize],
        ];
        let keep = match chunk.len() {
            1 => 2,
            2 => 3,
            _ => 4,
        };
        for i in 0..4 {
            let c = if i < keep { chars[i] } else { PAD };
            if n < dst.len() {
                dst[n] = c;
                n += 1;
            }
        }
    }
    n
}

// ==================== handshake ====================

/// Compute the Sec-WebSocket-Accept value: SHA1(key + GUID) base64'd.
/// Returns 28 ASCII chars. (ws_handshake in ws.c)
pub fn ws_accept_key(key24: &[u8; 24]) -> [u8; 28] {
    const GUID: &[u8] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    let mut sha = Sha1::new();
    sha.update(key24);
    sha.update(GUID);
    let digest = sha.finalize();
    let mut out = [0u8; 28];
    let n = b64_encode(&digest, &mut out);
    debug_assert_eq!(n, 28);
    out
}

// ==================== server frame encode ====================

/// FIN + opcode, unmasked server frame; hdr is filled, returns header length.
pub fn ws_frame_hdr(hdr: &mut [u8; 10], opcode: u8, len: usize) -> usize {
    hdr[0] = 0x80 | opcode;
    if len < 126 {
        hdr[1] = len as u8;
        2
    } else if len <= 0xFFFF {
        hdr[1] = 126;
        hdr[2] = (len >> 8) as u8;
        hdr[3] = len as u8;
        4
    } else {
        hdr[1] = 127;
        for i in 0..8 {
            hdr[2 + i] = (len >> (56 - 8 * i)) as u8;
        }
        10
    }
}

// ==================== client frame parser (ws_feed state machine) ====================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedEvent {
    /// Complete text/binary payload
    Frame { fin: bool, opcode: u8, payload_len: usize },
    /// Parser wants the session closed (oversize / bad state)
    Close,
}

#[derive(Debug, PartialEq, Eq)]
pub enum WsFeed {
    Header,
    Len16,
    Len64,
    Mask,
    Payload,
}

pub struct WsParser {
    pub state: WsFeed,
    hdr: [u8; 10],
    hdr_got: usize,
    pub fin: bool,
    pub opcode: u8,
    masked: bool,
    mask: [u8; 4],
    pub plen: usize,
    pub got: usize,
    pub payload: [u8; PAYLOAD_MAX],
}

pub const PAYLOAD_MAX: usize = 10 * 1024 + 16;

impl WsParser {
    pub fn new() -> Self {
        Self {
            state: WsFeed::Header,
            hdr: [0; 10],
            hdr_got: 0,
            fin: false,
            opcode: 0,
            masked: false,
            mask: [0; 4],
            plen: 0,
            got: 0,
            payload: [0; PAYLOAD_MAX],
        }
    }

    /// Feed bytes; calls `event` for each completed frame. Returns false when
    /// the session must be closed.
    pub fn feed(&mut self, data: &[u8], mut event: impl FnMut(&mut Self, FeedEvent)) -> bool {
        for &b in data {
            match self.state {
                WsFeed::Header => {
                    self.hdr[self.hdr_got] = b;
                    self.hdr_got += 1;
                    if self.hdr_got < 2 {
                        continue;
                    }
                    self.fin = self.hdr[0] & 0x80 != 0;
                    self.opcode = self.hdr[0] & 0x0F;
                    self.masked = self.hdr[1] & 0x80 != 0;
                    self.plen = (self.hdr[1] & 0x7F) as usize;
                    self.got = 0;
                    self.hdr_got = 0;
                    self.state = match self.plen {
                        126 => WsFeed::Len16,
                        127 => WsFeed::Len64,
                        n if n > PAYLOAD_MAX => return false,
                        _ if self.masked => WsFeed::Mask,
                        _ => WsFeed::Payload,
                    };
                }
                WsFeed::Len16 => {
                    self.hdr[self.hdr_got] = b;
                    self.hdr_got += 1;
                    if self.hdr_got == 2 {
                        self.plen = ((self.hdr[0] as usize) << 8) | self.hdr[1] as usize;
                        if self.plen > PAYLOAD_MAX {
                            return false;
                        }
                        self.hdr_got = 0;
                        self.state = if self.masked { WsFeed::Mask } else { WsFeed::Payload };
                    }
                }
                WsFeed::Len64 => {
                    self.hdr[self.hdr_got] = b;
                    self.hdr_got += 1;
                    if self.hdr_got == 8 {
                        let mut v: u64 = 0;
                        for i in 0..8 {
                            v = (v << 8) | self.hdr[i] as u64;
                        }
                        if v as usize > PAYLOAD_MAX {
                            return false;
                        }
                        self.plen = v as usize;
                        self.hdr_got = 0;
                        self.state = if self.masked { WsFeed::Mask } else { WsFeed::Payload };
                    }
                }
                WsFeed::Mask => {
                    self.mask[self.hdr_got] = b;
                    self.hdr_got += 1;
                    if self.hdr_got == 4 {
                        self.hdr_got = 0;
                        self.state = WsFeed::Payload;
                    }
                }
                WsFeed::Payload => {
                    let idx = self.got;
                    if idx >= PAYLOAD_MAX {
                        return false;
                    }
                    self.payload[idx] = if self.masked { b ^ self.mask[idx & 3] } else { b };
                    self.got = idx + 1;
                    if self.got == self.plen {
                        event(self, FeedEvent::Frame { fin: self.fin, opcode: self.opcode, payload_len: self.got });
                        self.state = WsFeed::Header;
                        self.hdr_got = 0;
                    }
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{:02x}", x)).collect()
    }

    #[test]
    fn sha1_vectors() {
        let mut s = Sha1::new();
        s.update(b"abc");
        assert_eq!(hex(&s.finalize()), "a9993e364706816aba3e25717850c26c9cd0d89d");
        let mut s = Sha1::new();
        s.update(b"");
        assert_eq!(hex(&s.finalize()), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
        let mut s = Sha1::new();
        s.update(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq");
        assert_eq!(hex(&s.finalize()), "84983e441c3bd26ebaae4aa1f95129e5e54670f1");
    }

    #[test]
    fn rfc6455_accept_key() {
        // RFC 6455 sample: key "dGhlIHNhbXBsZSBub25jZQ==" ->
        // accept "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
        let key = b"dGhlIHNhbXBsZSBub25jZQ==";
        let accept = ws_accept_key(key);
        assert_eq!(&accept[..], b"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
    }

    #[test]
    fn masked_text_frame_roundtrip() {
        // client frame from RFC 6455 §5.7: masked "Hello"
        let frame = [0x81, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58];
        let mut p = WsParser::new();
        let mut seen = 0;
        let ok = p.feed(&frame, |_, ev| {
            if let FeedEvent::Frame { fin, opcode, payload_len } = ev {
                assert!(fin && opcode == 1 && payload_len == 5);
                seen += 1;
            }
        });
        assert!(ok);
        assert_eq!(seen, 1);
        assert_eq!(&p.payload[..5], b"Hello");
    }

    #[test]
    fn server_frame_header() {
        let mut hdr = [0u8; 10];
        assert_eq!(ws_frame_hdr(&mut hdr, 1, 5), 2);
        assert_eq!(&hdr[..2], &[0x81, 0x05]);
        assert_eq!(ws_frame_hdr(&mut hdr, 1, 300), 4);
        assert_eq!(&hdr[..4], &[0x81, 126, 0x01, 0x2C]);
    }
}
