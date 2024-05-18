#![no_main]
#![no_std]

use floor_controller as _; // global logger + panicking-behavior + memory layout

use core::cell::RefCell;

use cortex_m::peripheral::NVIC;
use critical_section::{with, Mutex};
use hal::{
    self,
    clocks::Clocks,
    gpio::{self, Edge, Pin, PinMode, Port, Pull},
    low_power,
    pac::{self, interrupt},
    prelude::*,
    timer::{Timer, TimerInterrupt},
};

make_globals!((ONBOARD_LED, Pin), (DEBOUNCE_TIMER, Timer<pac::TIM2>),);

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    // this line is required if you want to take advantage of ST-Link
    hal::debug_workaround();

    let clock_cfg = Clocks::default();
    if clock_cfg.setup().is_err() {
        defmt::panic!("Unable to configure clock due to speed error")
    };

    let onboard_led = Pin::new(Port::A, 5, PinMode::Output);

    let mut btn_blue = Pin::new(Port::C, 13, PinMode::Input);
    btn_blue.pull(Pull::Floating);
    btn_blue.enable_interrupt(Edge::Falling);

    let mut debounce_timer = Timer::new_tim2(dp.TIM2, 5.0, Default::default(), &clock_cfg);
    debounce_timer.enable_interrupt(TimerInterrupt::Update);

    // Unmask interrupt lines
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI15_10);
        NVIC::unmask(pac::Interrupt::TIM2);

        cp.NVIC.set_priority(pac::Interrupt::EXTI15_10, 0);
        cp.NVIC.set_priority(pac::Interrupt::TIM2, 1);
    }

    // Setup as globally accessible variables so they can be accessed inside interrupts
    with(|cs| {
        DEBOUNCE_TIMER.borrow(cs).replace(Some(debounce_timer));
        ONBOARD_LED.borrow(cs).replace(Some(onboard_led));
    });

    defmt::debug!("Hardware initialized");

    loop {
        low_power::sleep_now();
    }
}

#[interrupt]
fn EXTI15_10() {
    with(|cs| {
        // Clear the interrupt flag, to prevent continous firing.
        gpio::clear_exti_interrupt(13);

        // A helper macro to access the pin and timer we stored in mutexes.
        access_global!(DEBOUNCE_TIMER, debounce_timer, cs);
        if debounce_timer.is_enabled() {
            return;
        }

        access_global!(ONBOARD_LED, onboard_led, cs);
        onboard_led.toggle();

        debounce_timer.enable();
    });
}

#[interrupt]
fn TIM2() {
    with(|cs| {
        access_global!(DEBOUNCE_TIMER, debounce_timer, cs);
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        debounce_timer.clear_interrupt(TimerInterrupt::Update);

        // Disable the timer until next time you press a button.
        debounce_timer.disable();
    });
}
