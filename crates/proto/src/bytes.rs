//! Byte-order helpers, ported 1:1 from src/util/io_bytes.c.

pub fn get_be16(p: &[u8]) -> u16 {
    ((p[0] as u16) << 8) | p[1] as u16
}

pub fn get_be32(p: &[u8]) -> u32 {
    ((p[0] as u32) << 24) | ((p[1] as u32) << 16) | ((p[2] as u32) << 8) | p[3] as u32
}

pub fn get_le32(p: &[u8]) -> u32 {
    ((p[3] as u32) << 24) | ((p[2] as u32) << 16) | ((p[1] as u32) << 8) | p[0] as u32
}

pub fn put_be16(v: u16, p: &mut [u8]) {
    p[0] = (v >> 8) as u8;
    p[1] = v as u8;
}

pub fn put_be32(v: u32, p: &mut [u8]) {
    p[0] = (v >> 24) as u8;
    p[1] = (v >> 16) as u8;
    p[2] = (v >> 8) as u8;
    p[3] = v as u8;
}

pub fn put_le32(v: u32, p: &mut [u8]) {
    p[0] = v as u8;
    p[1] = (v >> 8) as u8;
    p[2] = (v >> 16) as u8;
    p[3] = (v >> 24) as u8;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn be16_roundtrip() {
        let mut b = [0u8; 2];
        put_be16(0x1234, &mut b);
        assert_eq!(b, [0x12, 0x34]);
        assert_eq!(get_be16(&b), 0x1234);
    }

    #[test]
    fn be32_roundtrip() {
        let mut b = [0u8; 4];
        put_be32(0x11223344, &mut b);
        assert_eq!(b, [0x11, 0x22, 0x33, 0x44]);
        assert_eq!(get_be32(&b), 0x11223344);
    }

    #[test]
    fn le32_roundtrip() {
        let mut b = [0u8; 4];
        put_le32(0x11223344, &mut b);
        assert_eq!(b, [0x44, 0x33, 0x22, 0x11]);
        assert_eq!(get_le32(&b), 0x11223344);
    }
}
