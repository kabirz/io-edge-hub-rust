#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

mod bindings;
pub use bindings::*;

// minimal libc string functions lfs.c links against (there is no libc in a
// no_std build; tinyrlibc equivalents, semantics per C99).
// Only for bare-metal targets — on a host build these clash with the C
// runtime's own definitions (LNK1169) and libc is available anyway.
use core::ffi::{c_char, c_int, c_ulong};

#[cfg(target_os = "none")]
#[no_mangle]
pub unsafe extern "C" fn strlen(s: *const c_char) -> usize {
    let mut n = 0usize;
    while *s.add(n) != 0 {
        n += 1;
    }
    n
}

#[cfg(target_os = "none")]
#[no_mangle]
pub unsafe extern "C" fn strchr(s: *const c_char, c: c_int) -> *mut c_char {
    let b = c as u8;
    let mut i = 0usize;
    loop {
        let cur = *s.add(i) as u8;
        if cur == b {
            return s.add(i) as *mut c_char;
        }
        if cur == 0 {
            return core::ptr::null_mut();
        }
        i += 1;
    }
}

#[cfg(target_os = "none")]
#[no_mangle]
pub unsafe extern "C" fn strspn(s: *const c_char, accept: *const c_char) -> c_ulong {
    let mut n = 0usize;
    'outer: while *s.add(n) != 0 {
        let c = *s.add(n) as u8;
        let mut a = 0usize;
        while *accept.add(a) != 0 {
            if *accept.add(a) as u8 == c {
                n += 1;
                continue 'outer;
            }
            a += 1;
        }
        break;
    }
    n as c_ulong
}

#[cfg(target_os = "none")]
#[no_mangle]
pub unsafe extern "C" fn strcspn(s: *const c_char, reject: *const c_char) -> c_ulong {
    let mut n = 0usize;
    'outer: while *s.add(n) != 0 {
        let c = *s.add(n) as u8;
        let mut r = 0usize;
        while *reject.add(r) != 0 {
            if *reject.add(r) as u8 == c {
                break 'outer;
            }
            r += 1;
        }
        n += 1;
    }
    n as c_ulong
}
