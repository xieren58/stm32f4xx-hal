//! # I2S example with rtic
//!
//! This application show how to use I2sDriver with interruption.
//!
//! # Hardware required
//!
//! * a STM32F411 based board
//! * I2S ADC and DAC, eg PCM1808 and PCM5102 from TI
//! * Audio signal at ADC input, and something to ear at DAC output.
//!
//! # Hardware Wiring
//!
//! ## Stm32
//! TODO
//!
//! ## Assuming you use a PCM1808 module
//! TODO
//!
//! ## Asumming you use a PCM5102 module
//! TODO
//!
//! Expected behavior: TODO

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rtt_target::rprintln;

use stm32f4xx_hal as hal;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3 ])]
mod app {
    use core::fmt::Write;

    use super::hal;

    use hal::gpio::gpioa::*;
    use hal::gpio::gpiob::*;
    use hal::gpio::gpioc::*;
    use hal::gpio::NoPin;
    use hal::i2s::I2s;
    use hal::pac::{SPI2, SPI3};
    use hal::prelude::*;

    use rtt_target::{rtt_init, set_print_channel};

    use stm32_i2s_v12x::{Channel, Config, I2sDriver, I2sStandard};

    type I2s2Driver = I2sDriver<I2s<SPI2, (PB12, PB13, PC6, PB15)>>;
    type I2s3Driver = I2sDriver<I2s<SPI3, (PA4, PC10, NoPin, PC12)>>;

    // Part of the frame we curently transmit or receive
    #[derive(Copy, Clone)]
    pub enum FrameState {
        LeftMsb,
        LeftLsb,
        RightMsb,
        RightLsb,
    }

    use FrameState::{LeftLsb, LeftMsb, RightLsb, RightMsb};

    impl Default for FrameState {
        fn default() -> Self {
            Self::LeftMsb
        }
    }

    #[shared]
    struct Shared {
        #[lock_free]
        i2s2_driver: I2s2Driver,
        #[lock_free]
        i2s3_driver: I2s3Driver,
        sample: (u32, u32), // sample to communicate to the i2s transmit task
    }

    #[local]
    struct Local {
        logs_chan: rtt_target::UpChannel,
        i2s2: I2s2Local,
        i2s3: I2s3Local,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 128
                    name: "Logs"
                }
                1: {
                    size: 128
                    name: "Panics"
                }
            }
        };
        let logs_chan = channels.up.0;
        let panics_chan = channels.up.1;
        set_print_channel(panics_chan);
        let device = ctx.device;
        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8u32.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(50.MHz())
            .pclk2(100.MHz())
            .i2s_clk(61440.kHz())
            .freeze();

        // I2S pins: (WS, CK, MCLK, SD) for I2S2
        let i2s2_pins = (
            gpiob.pb12, //WS
            gpiob.pb13, //CK
            gpioc.pc6,  //MCK
            gpiob.pb15, //SD
        );
        let i2s2 = I2s::new(device.SPI2, i2s2_pins, &clocks);
        let i2s2_config = Config::new_master()
            .receive()
            .standard(I2sStandard::Philips)
            .master_clock(true)
            .request_frequency(48_000);
        let mut i2s2_driver = stm32_i2s_v12x::I2sDriver::new(i2s2, i2s2_config);
        i2s2_driver.set_rx_interrupt(true);
        i2s2_driver.set_error_interrupt(true);

        // I2S3 pins: (WS, CK, MCLK, SD) for I2S3
        let i2s3_pins = (gpioa.pa4, gpioc.pc10, NoPin, gpioc.pc12);
        let i2s3 = I2s::new(device.SPI3, i2s3_pins, &clocks);
        let i2s3_config = i2s2_config.to_slave().transmit();
        let mut i2s3_driver = I2sDriver::new(i2s3, i2s3_config);
        i2s3_driver.set_tx_interrupt(true);
        i2s3_driver.set_error_interrupt(true);

        (
            Shared {
                i2s2_driver,
                i2s3_driver,
                sample: Default::default(),
            },
            Local {
                logs_chan,
                i2s2: Default::default(),
                i2s3: Default::default(),
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(_cx: idle::Context) -> ! {
        #[allow(clippy::empty_loop)]
        loop {}
    }

    // Printing message directly in a i2s interrupt can cause timing issues.
    #[task(capacity = 10, local = [logs_chan])]
    fn log(cx: log::Context, message: &'static str) {
        writeln!(cx.local.logs_chan, "{}", message).unwrap();
    }

    // processing audio
    #[task(shared = [sample])]
    fn process(mut cx: process::Context, sample: (u32, u32)) {
        cx.shared.sample.lock(|smpl| *smpl = sample);
    }

    #[derive(Default)]
    pub struct I2s2Local {
        frame_state: FrameState,
    }

    #[task(
        priority = 4,
        binds = SPI2,
        local = [frame_state: FrameState = LeftMsb, frame: (u32,u32) = (0,0), i2s2],
        shared = [i2s2_driver]
    )]
    fn i2s2(cx: i2s2::Context) {
        let frame_state = cx.local.frame_state;
        let frame = cx.local.frame;
        let i2s2_driver = cx.shared.i2s2_driver;
        let status = i2s2_driver.status();
        if status.ovr() {
            log::spawn("i2s2 Overrun").ok();
            // sequence to delete ovr flag
            i2s2_driver.read_data_register();
            i2s2_driver.status();
        } else if status.rxne() {
            let data = i2s2_driver.read_data_register();
            match (*frame_state, status.chside()) {
                (LeftMsb, Channel::Left) => {
                    frame.0 = (data as u32) << 16;
                    *frame_state = LeftLsb;
                }
                (LeftLsb, Channel::Left) => {
                    frame.0 |= data as u32;
                    *frame_state = RightMsb;
                }
                (RightMsb, Channel::Right) => {
                    frame.1 = (data as u32) << 16;
                    *frame_state = RightLsb;
                }
                (RightLsb, Channel::Right) => {
                    frame.1 |= data as u32;
                    // do process here
                    *frame_state = LeftMsb;
                }
                // in case of ovr this resynchronize at start of new frame
                _ => *frame_state = LeftMsb,
            }
        }
    }

    #[derive(Default)]
    pub struct I2s3Local {
        frame_state: FrameState,
    }

    #[task(
        priority = 4,
        binds = SPI3,
        local = [frame_state: FrameState = LeftMsb,frame: (u32,u32) = (0,0), i2s3],
        shared = [sample,i2s3_driver]
    )]
    fn i2s3(cx: i2s3::Context) {
        let frame_state = cx.local.frame_state;
        let frame = cx.local.frame;
        let i2s3_driver = cx.shared.i2s3_driver;
        let mut sample = cx.shared.sample;
        let status = i2s3_driver.status();
        if status.fre() {
            log::spawn("i2s3 Frame error").ok();
            todo!("slave synchronisation");
        } else if status.udr() {
            log::spawn("i2s3 underrun").ok();
        } else if status.rxne() {
            let data;
            match (*frame_state, status.chside()) {
                (LeftMsb, Channel::Left) => {
                    *frame = sample.lock(|smpl| *smpl);
                    data = (frame.0 >> 16) as u16;
                    *frame_state = LeftLsb;
                }
                (LeftLsb, Channel::Left) => {
                    data = (frame.0 & 0xFFFF) as u16;
                    *frame_state = RightMsb;
                }
                (RightMsb, Channel::Right) => {
                    data = (frame.1 >> 16) as u16;
                    *frame_state = RightLsb;
                }
                (RightLsb, Channel::Right) => {
                    data = (frame.1 & 0xFFFF) as u16;
                    *frame_state = LeftMsb;
                }
                // in case of udr this resynchronize tracked and actual channel
                _ => {
                    *frame_state = LeftMsb;
                    data = 0; //garbage data to avoid additional underrrun
                }
            }
            i2s3_driver.write_data_register(data);
        }
    }

    // Look i2s3 WS line for (re) synchronisation
    #[task(priority = 4, binds = EXTI4, shared = [i2s3_driver])]
    fn exti4(cx: exti4::Context) {
        let mut _i2s3_driver = cx.shared.i2s3_driver;
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
