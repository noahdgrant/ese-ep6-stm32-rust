#![no_main]
#![no_std]

use template_stm32f3_rust as _; // global logger + panicking-behavior + memory layout

use cortex_m::delay::Delay;
use hal::{
    self,
    clocks::Clocks,
    gpio::{Pin, PinMode, Port},
    pac,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    // this line is required if you want to take advantage of ST-Link
    hal::debug_workaround();

    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Port::A, 5 because the LED is described as PA5
    let mut led = Pin::new(Port::A, 5, PinMode::Output);

    defmt::debug!("Hardware initialized");

    // Now, enjoy the lightshow!
    loop {
        led.toggle();
        delay.delay_ms(1_000);
        defmt::debug!("blink");
    }
}
