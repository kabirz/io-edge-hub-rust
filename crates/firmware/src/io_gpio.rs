//! DO/LED GPIO outputs (PD7-PD14 DO8, PE8-PE15 LED8 mirror).

use core::cell::RefCell;

use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

type Outs = [Option<Output<'static>>; 8];

static DO: Mutex<CriticalSectionRawMutex, RefCell<Outs>> =
    Mutex::new(RefCell::new([None, None, None, None, None, None, None, None]));
static LED: Mutex<CriticalSectionRawMutex, RefCell<Outs>> =
    Mutex::new(RefCell::new([None, None, None, None, None, None, None, None]));

pub fn init(do_outs: Outs, led_outs: Outs) {
    critical_section::with(|_cs| {
        DO.lock(|o| *o.borrow_mut() = do_outs);
        LED.lock(|o| *o.borrow_mut() = led_outs);
    });
}

fn drive(outs: &mut Outs, val: u8) {
    for (i, o) in outs.iter_mut().enumerate() {
        if let Some(o) = o {
            o.set_level(((val >> i) & 1 != 0).into());
        }
    }
}

/// Drive DO + LED mirror from a DO byte (mb_set_do equivalent).
pub fn set_do_led(val: u8) {
    critical_section::with(|_cs| {
        DO.lock(|o| drive(&mut o.borrow_mut(), val));
        LED.lock(|o| drive(&mut o.borrow_mut(), val));
    });
}

pub fn do_byte() -> u8 {
    critical_section::with(|_cs| {
        DO.lock(|o| {
            o.borrow()
                .iter()
                .enumerate()
                .filter(|(_, op)| op.as_ref().is_some_and(|p| p.is_set_high()))
                .fold(0u8, |v, (i, _)| v | (1 << i))
        })
    })
}
