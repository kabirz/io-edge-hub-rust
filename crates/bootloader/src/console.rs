//! Boot console: raw-register USART1 PA9 TX @115200 (same wiring as the
//! application's console). TX-only polling — the bootloader never reads.

const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32; // GPIOAEN
const RCC_APB2ENR: *mut u32 = 0x4002_3844 as *mut u32; // USART1EN
const GPIOA_MODER: *mut u32 = 0x4002_0000 as *mut u32;
const GPIOA_AFRL: *mut u32 = 0x4002_0020 as *mut u32;
const GPIOA_AFRH: *mut u32 = 0x4002_0024 as *mut u32;
const USART1_SR: *mut u32 = 0x4001_1000 as *mut u32;
const USART1_DR: *mut u32 = 0x4001_1004 as *mut u32;
const USART1_BRR: *mut u32 = 0x4001_1008 as *mut u32;
const USART1_CR1: *mut u32 = 0x4001_100C as *mut u32;

pub fn init() {
    unsafe {
        RCC_AHB1ENR.write_volatile(RCC_AHB1ENR.read_volatile() | (1 << 0));
        RCC_APB2ENR.write_volatile(RCC_APB2ENR.read_volatile() | (1 << 4));
        // PA9: AF7 push-pull (MODER9=10, AFRH9=0111)
        GPIOA_MODER.write_volatile((GPIOA_MODER.read_volatile() & !(3 << 18)) | (2 << 18));
        let afrh = GPIOA_AFRH.read_volatile();
        GPIOA_AFRH.write_volatile((afrh & !(0xF << 4)) | (7 << 4));
        let _ = GPIOA_AFRL; // unused; keeps the register map documented
                            // 115200 8N1 from PCLK2 = 84 MHz: 84e6/115200 = 729.17 -> 729
        USART1_CR1.write_volatile(0);
        USART1_BRR.write_volatile(729);
        USART1_CR1.write_volatile((1 << 13) | (1 << 3)); // UE | TE
    }
}

pub fn put(b: u8) {
    unsafe {
        // bounded wait: a wedged UART must not hang the boot
        let mut spins = 0u32;
        while USART1_SR.read_volatile() & (1 << 7) == 0 {
            spins += 1;
            if spins > 2_000_000 {
                return;
            }
        }
        USART1_DR.write_volatile(b as u32);
    }
}

pub fn line(s: &str) {
    for &b in s.as_bytes() {
        put(b);
    }
    put(b'\r');
    put(b'\n');
}
