//! Floor and car controller test code for Engineering Project 6

#![no_main]
#![no_std]

use elevator as _; // global logger + panicking-behavior + memory layout

use bxcan::{
    filter::Mask32,
    {Frame, StandardId},
};
use cortex_m::{asm, peripheral::NVIC};
use nb::block;
use rtic::app;
use stm32f3xx_hal::{
    can::Can,
    gpio::{
        Alternate, Edge, Gpioa, Input, Output, Pin, PushPull, PA4, PA5, PA6, PA7, PB0, PB1, PB12,
        PB14, PB15, PC13, PC5, U,
    },
    pac::Interrupt,
    prelude::*,
};
use systick_monotonic::Systick;

const ID_CAR_CONTROLLER: u16 = 0x200;
const ID_FLOOR_1: u16 = 0x201;
const ID_FLOOR_2: u16 = 0x202;
const ID_FLOOR_3: u16 = 0x203;

// Change this to one of the above ID's to change the ID that is put on the CAN bus
const ID: u16 = ID_FLOOR_1;

const TEST_FLOOR_1: u8 = 0x85;
const TEST_FLOOR_2: u8 = 0x86;
const TEST_FLOOR_3: u8 = 0x87;

#[app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                Pin<Gpioa, U<12>, Alternate<PushPull, 9>>,
                Pin<Gpioa, U<11>, Alternate<PushPull, 9>>,
            >,
        >,
        led_floor_1: PA4<Output<PushPull>>,
        led_floor_2: PA6<Output<PushPull>>,
        led_floor_3: PA7<Output<PushPull>>,
        led_current_floor_1: PC5<Output<PushPull>>,
        led_current_floor_2: PB0<Output<PushPull>>,
        led_current_floor_3: PB1<Output<PushPull>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        btn_onboard: PC13<Input>,
        btn_floor_1: PB12<Input>,
        btn_floor_2: PB15<Input>,
        btn_floor_3: PB14<Input>,
        led_onboard: PA5<Output<PushPull>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::debug!("Start");

        // Get access to device peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut syscfg = cx.device.SYSCFG.constrain(&mut rcc.apb2);
        let mut exti = cx.device.EXTI;

        // Get GPIO banks
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb);

        // Setup clocks
        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        // Tell the scheduler which clock to use
        let mono = Systick::new(cx.core.SYST, 36_000_000);

        // Setup CAN
        let can_tx =
            gpioa
                .pa12
                .into_af_push_pull::<9>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        let can_rx =
            gpioa
                .pa11
                .into_af_push_pull::<9>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        // http://www.bittiming.can-wiki.info
        let mut can = bxcan::Can::builder(Can::new(cx.device.CAN, can_tx, can_rx, &mut rcc.apb1))
            .set_bit_timing(0x0069000f) // Bit rate: 125kbps, sample point 61.1%
            .set_loopback(false)
            .set_silent(false)
            .leave_disabled();
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        let filter_id = StandardId::new(ID_FLOOR_1).unwrap();
        let filter_mask = StandardId::new(0x7FC).unwrap(); // Accept only 0x201, 0x202, 0x203

        let mut filters = can.modify_filters();
        filters.enable_bank(
            0,
            bxcan::Fifo::Fifo0,
            Mask32::frames_with_std_id(filter_id, filter_mask),
        );

        // Enable filters
        drop(filters);

        // Sync to the bus and start normal operation
        block!(can.enable_non_blocking()).ok();

        // Setup status LEDs
        let mut led_onboard = gpioa
            .pa5
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        led_onboard.set_low().unwrap();

        let mut led_floor_1 = gpioa
            .pa4
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        led_floor_1.set_low().unwrap();

        let mut led_floor_2 = gpioa
            .pa6
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        led_floor_2.set_low().unwrap();

        let mut led_floor_3 = gpioa
            .pa7
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        led_floor_3.set_low().unwrap();

        // Car current floor LEDs
        let mut led_current_floor_1 = gpioc
            .pc5
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        led_current_floor_1.set_low().unwrap();

        let mut led_current_floor_2 = gpiob
            .pb0
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        led_current_floor_2.set_low().unwrap();

        let mut led_current_floor_3 = gpiob
            .pb1
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        led_current_floor_3.set_low().unwrap();

        // Setup buttons
        let mut btn_onboard = gpioc
            .pc13
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
        syscfg.select_exti_interrupt_source(&btn_onboard);
        btn_onboard.trigger_on_edge(&mut exti, Edge::Falling);
        btn_onboard.enable_interrupt(&mut exti);
        let btn_onboard_interrupt_num = btn_onboard.interrupt();

        let mut btn_floor_1 = gpiob
            .pb12
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
        syscfg.select_exti_interrupt_source(&btn_floor_1);
        btn_floor_1.trigger_on_edge(&mut exti, Edge::Falling);
        btn_floor_1.enable_interrupt(&mut exti);
        let btn_floor_1_interrupt_num = btn_floor_1.interrupt();

        let mut btn_floor_2 = gpiob
            .pb15
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
        syscfg.select_exti_interrupt_source(&btn_floor_2);
        btn_floor_2.trigger_on_edge(&mut exti, Edge::Falling);
        btn_floor_2.enable_interrupt(&mut exti);
        let btn_floor_2_interrupt_num = btn_floor_2.interrupt();

        let mut btn_floor_3 = gpiob
            .pb14
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
        syscfg.select_exti_interrupt_source(&btn_floor_3);
        btn_floor_3.trigger_on_edge(&mut exti, Edge::Falling);
        btn_floor_3.enable_interrupt(&mut exti);
        let btn_floor_3_interrupt_num = btn_floor_3.interrupt();

        unsafe {
            NVIC::unmask(btn_onboard_interrupt_num);
            NVIC::unmask(btn_floor_1_interrupt_num);
            NVIC::unmask(btn_floor_2_interrupt_num);
            NVIC::unmask(btn_floor_3_interrupt_num);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        };

        defmt::debug!("Hardware configuration complete");

        // Turn on floor 1 LEDs at the begining of the test
        led_floor_1.set_high().unwrap();
        led_current_floor_1.set_high().unwrap();

        // Setup resources
        (
            Shared {
                can,
                led_floor_1,
                led_floor_2,
                led_floor_3,
                led_current_floor_1,
                led_current_floor_2,
                led_current_floor_3,
            },
            Local {
                btn_onboard,
                btn_floor_1,
                btn_floor_2,
                btn_floor_3,
                led_onboard,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(binds = EXTI15_10,
        local = [btn_onboard, btn_floor_1, btn_floor_2, btn_floor_3, led_onboard],
        shared = [can])]
    fn exti15_10(mut cx: exti15_10::Context) {
        defmt::debug!("EXTI15_10 interrupt");

        let data: Option<[u8; 1]>;

        if cx.local.btn_onboard.is_interrupt_pending() {
            cx.local.btn_onboard.clear_interrupt();
            defmt::debug!("Onboard button pressed");

            cx.local.led_onboard.toggle().unwrap();
            data = Some([0x80]);
        } else if cx.local.btn_floor_1.is_interrupt_pending() {
            defmt::debug!("Floor 1 button pressed");
            cx.local.btn_floor_1.clear_interrupt();

            test_floor_1::spawn().unwrap();
            data = Some([TEST_FLOOR_1]);
        } else if cx.local.btn_floor_2.is_interrupt_pending() {
            defmt::debug!("Floor 2 button pressed");
            cx.local.btn_floor_2.clear_interrupt();

            test_floor_2::spawn().unwrap();
            data = Some([TEST_FLOOR_2]);
        } else if cx.local.btn_floor_3.is_interrupt_pending() {
            defmt::debug!("Floor 3 button pressed");
            cx.local.btn_floor_3.clear_interrupt();

            test_floor_3::spawn().unwrap();
            data = Some([TEST_FLOOR_3]);
        } else {
            defmt::panic!("Unhandeled exti15_10 interrupt");
        }

        // Send CAN message
        let frame = Frame::new_data(
            StandardId::new(ID).unwrap(),
            data.expect("Missing CAN data"),
        );
        cx.shared.can.lock(|can| {
            block!(can.transmit(&frame)).expect("Cannot send CAN frame");
        });
        defmt::debug!(
            "[CAN] tx: id = {:#04x} data = {:#02x}",
            ID,
            data.expect("Missing CAN data")[0]
        );
    }

    #[task(binds = USB_LP_CAN_RX0,
        shared = [can])]
    fn can_rx0(mut cx: can_rx0::Context) {
        defmt::debug!("CAN rx interrupt");

        let mut can_frame: Option<Frame> = None;
        cx.shared.can.lock(|can| {
            can_frame = Some(block!(can.receive()).expect("Cannot receive CAN frame"));
        });

        if let Some(frame) = can_frame.as_ref() {
            if let Some(data) = frame.data() {
                defmt::debug!(
                    "[CAN] rx: id = {:#x} data = {:#02x}",
                    frame.id_as_raw(),
                    data[0],
                );

                match data[0] {
                    TEST_FLOOR_1 => {
                        test_floor_1::spawn().unwrap();
                    }
                    TEST_FLOOR_2 => {
                        test_floor_2::spawn().unwrap();
                    }
                    TEST_FLOOR_3 => {
                        test_floor_3::spawn().unwrap();
                    }
                    _ => (),
                }
            }
        }
    }

    #[task(shared = [led_floor_1, led_floor_2, led_floor_3, led_current_floor_1,
    led_current_floor_2, led_current_floor_3])]
    fn test_floor_1(mut cx: test_floor_1::Context) {
        cx.shared.led_floor_1.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_1.lock(|led| {
            led.set_low().unwrap();
        });

        cx.shared.led_floor_2.lock(|led| {
            led.set_high().unwrap();
        });
        cx.shared.led_current_floor_2.lock(|led| {
            led.set_high().unwrap();
        });

        cx.shared.led_floor_3.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_3.lock(|led| {
            led.set_low().unwrap();
        });
    }

    #[task(shared = [led_floor_1, led_floor_2, led_floor_3, led_current_floor_1,
    led_current_floor_2, led_current_floor_3])]
    fn test_floor_2(mut cx: test_floor_2::Context) {
        cx.shared.led_floor_1.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_1.lock(|led| {
            led.set_low().unwrap();
        });

        cx.shared.led_floor_2.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_2.lock(|led| {
            led.set_low().unwrap();
        });

        cx.shared.led_floor_3.lock(|led| {
            led.set_high().unwrap();
        });
        cx.shared.led_current_floor_3.lock(|led| {
            led.set_high().unwrap();
        });
    }

    #[task(shared = [led_floor_1, led_floor_2, led_floor_3, led_current_floor_1,
    led_current_floor_2, led_current_floor_3])]
    fn test_floor_3(mut cx: test_floor_3::Context) {
        cx.shared.led_floor_1.lock(|led| {
            led.set_high().unwrap();
        });
        cx.shared.led_current_floor_1.lock(|led| {
            led.set_high().unwrap();
        });

        cx.shared.led_floor_2.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_2.lock(|led| {
            led.set_low().unwrap();
        });

        cx.shared.led_floor_3.lock(|led| {
            led.set_low().unwrap();
        });
        cx.shared.led_current_floor_3.lock(|led| {
            led.set_low().unwrap();
        });
    }
}

trait GetId {
    /// Returns the raw value of the id. The value is always a u32 since
    /// extended IDs are u32.
    fn id_as_raw(&self) -> u32;
}

impl GetId for bxcan::Frame {
    fn id_as_raw(&self) -> u32 {
        match self.id() {
            bxcan::Id::Standard(id) => id.as_raw().into(),
            bxcan::Id::Extended(id) => id.as_raw(),
        }
    }
}
