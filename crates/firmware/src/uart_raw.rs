//! Raw-register console UART: USART1 PA9/PA10 @115200.
//!
//! Why not the embassy driver: the logger needs a sync TX usable from inside
//! critical sections (TXE-poll on the register), and the shell RX must
//! survive the millisecond PRIMASK freezes of NOR flash operations — a
//! register-level RXNE interrupt loses bursts (1-byte DR + ORE), but a DMA2
//! circular channel keeps draining the DR in hardware while interrupts are
//! masked. TX stays busy-poll (PRIMASK-safe).

// ---- RCC ----
const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32; // GPIOAEN | DMA2EN
const RCC_APB2ENR: *mut u32 = 0x4002_3844 as *mut u32; // USART1EN

// ---- GPIOA ----
const GPIOA_MODER: *mut u32 = 0x4002_0000 as *mut u32;
#[allow(dead_code)] // PA0-7 alternate-function regs; only PA9/PA10 (AFRH) wired
const GPIOA_AFRL: *mut u32 = 0x4002_0020 as *mut u32;
const GPIOA_AFRH: *mut u32 = 0x4002_0024 as *mut u32;

// ---- USART1 ----
const USART1_SR: *mut u32 = 0x4001_1000 as *mut u32;
const USART1_DR: *mut u32 = 0x4001_1004 as *mut u32;
const USART1_BRR: *mut u32 = 0x4001_1008 as *mut u32;
const USART1_CR1: *mut u32 = 0x4001_100C as *mut u32;
const USART1_CR3: *mut u32 = 0x4001_1014 as *mut u32;

// ---- DMA2 (stream 2, channel 4 = USART1_RX): base + 0x10 + 0x18*2 = +0x40 ----
const DMA2_ST2_CR: *mut u32 = (0x4002_6400 + 0x40) as *mut u32;
const DMA2_ST2_NDTR: *mut u32 = (0x4002_6400 + 0x44) as *mut u32;
const DMA2_ST2_PAR: *mut u32 = (0x4002_6400 + 0x48) as *mut u32;
const DMA2_ST2_M0AR: *mut u32 = (0x4002_6400 + 0x4C) as *mut u32;
const DMA2_ST2_FCR: *mut u32 = (0x4002_6400 + 0x50) as *mut u32;

const DMA_SX_CR_EN: u32 = 1 << 0;
const DMA_SX_CR_CIRC: u32 = 1 << 8;
const DMA_SX_CR_MINC: u32 = 1 << 10;
const DMA_SX_CR_PSIZ_8: u32 = 0b00 << 11;
const DMA_SX_CR_MSIZ_8: u32 = 0b00 << 13;
const DMA_SX_CR_CHSEL_4: u32 = 0b100 << 25;

pub const RX_RING: usize = 256;
static mut RX_BUF: [u8; RX_RING] = [0; RX_RING];

/// Init GPIO/USART1/DMA2-Stream2 circular RX. Called once from main before
/// the logger/shell tasks start.
pub fn init() {
    unsafe {
        // clocks: GPIOA | DMA2 ; USART1
        RCC_AHB1ENR.write_volatile(RCC_AHB1ENR.read_volatile() | (1 << 0) | (1 << 22));
        RCC_APB2ENR.write_volatile(RCC_APB2ENR.read_volatile() | (1 << 4));

        // PA9 = AF7 (USART1_TX), PA10 = AF7 (USART1_RX); pull-up like log.c
        let m = GPIOA_MODER.read_volatile();
        GPIOA_MODER.write_volatile((m & !((3 << 18) | (3 << 20))) | (2 << 18) | (2 << 20));
        let h = GPIOA_AFRH.read_volatile();
        GPIOA_AFRH.write_volatile((h & !((0xF << 4) | (0xF << 8))) | (7 << 4) | (7 << 8));

        // 115200 8N1, TE+RE; PCLK2 = 84 MHz / 115200 = 729.17 -> 729 (0x2D9)
        USART1_CR1.write_volatile(0); // disable while configuring
        USART1_BRR.write_volatile(729);
        USART1_CR1.write_volatile((1 << 13) | (1 << 3) | (1 << 2)); // UE | TE | RE
        USART1_CR3.write_volatile(1 << 6); // DMAR: enable RX DMA requests

        // DMA2 stream2 ch4: circular byte ring from USART1 DR
        DMA2_ST2_CR.write_volatile(0);
        DMA2_ST2_FCR.write_volatile(0); // direct mode, no FIFO
        DMA2_ST2_PAR.write_volatile(USART1_DR as u32);
        // addr_of_mut!, not RX_BUF.as_ptr(): a shared reference to a
        // `static mut` is UB-adjacent (static_mut_refs lint) and a hard
        // error under edition 2024
        DMA2_ST2_M0AR.write_volatile(core::ptr::addr_of_mut!(RX_BUF) as u32);
        DMA2_ST2_NDTR.write_volatile(RX_RING as u32);
        DMA2_ST2_CR.write_volatile(
            DMA_SX_CR_CHSEL_4
                | DMA_SX_CR_MSIZ_8
                | DMA_SX_CR_PSIZ_8
                | DMA_SX_CR_MINC
                | DMA_SX_CR_CIRC
                | DMA_SX_CR_EN,
        );
    }
}

/// Blocking TX byte(s) — safe inside critical sections (pure register poll).
pub fn write(bytes: &[u8]) {
    unsafe {
        for &b in bytes {
            while USART1_SR.read_volatile() & (1 << 7) == 0 {} // TXE
            USART1_DR.write_volatile(b as u32);
        }
    }
}

/// Number of bytes available in the RX ring (DMA-maintained head vs our tail).
pub fn rx_available(tail: usize) -> usize {
    let ndtr = unsafe { DMA2_ST2_NDTR.read_volatile() } as usize;
    let head = RX_RING - ndtr;
    if head >= tail {
        head - tail
    } else {
        RX_RING - tail + head
    }
}

/// Read one byte at `tail` position (caller advances tail mod RX_RING).
/// Volatile: the DMA engine writes RX_BUF concurrently (single-byte reads
/// are atomic on Cortex-M; volatile keeps the compiler from caching it).
pub fn rx_peek(tail: usize) -> u8 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(RX_BUF[tail % RX_RING])) }
}
