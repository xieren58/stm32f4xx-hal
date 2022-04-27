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
    use super::hal;

    use hal::gpio::gpioa::*;
    use hal::gpio::gpiob::*;
    use hal::gpio::gpioc::*;
    use hal::gpio::NoPin;
    use hal::i2s::I2s;
    use hal::pac::{SPI2, SPI3};
    use hal::prelude::*;

    use stm32_i2s_v12x::{Config, I2sDriver, I2sStandard};

    type I2s2Driver = I2sDriver<I2s<SPI2, (PB12, PB13, PC6, PB15)>>;
    type I2s3Driver = I2sDriver<I2s<SPI3, (PA4, PC10, NoPin, PC12)>>;

    #[shared]
    struct Shared {
        i2s2_driver: I2s2Driver,
        i2s3_driver: I2s3Driver,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(cx: idle::Context) -> ! {
        loop {}
    }

    #[task(priority = 4, binds = SPI2, local = [], shared = [])]
    fn i2s2(cx: i2s2::Context) {}

    #[task(priority = 4, binds = SPI3, local = [], shared = [])]
    fn i2s3(cx: i2s3::Context) {}

    #[task(priority = 5, binds = EXTI15_10, shared = [])]
    fn exti15_10(cx: exti15_10::Context) {}
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
