//! This example demonstrates how to use the RTC.
//! Note that the LSI can be quite inaccurate.
//! The tolerance is up to ±47% (Min 17 kHz, Typ 32 kHz, Max 47 kHz).

#![no_main]
#![no_std]

// Halt on panic
use panic_rtt_target as _;

//use core::fmt::Write;
use rtt_target::{rprintln, rtt_init_print};
use cortex_m_rt::entry;

use stm32f4xx_hal::{
    pac,
    prelude::*,
    rtc::{Rtc, RtcClock},
    rcc::{LSEClockMode, LSEClock},
};
use time::{
    macros::{date, time},
    PrimitiveDateTime,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut p = pac::Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();

    //let clocks = rcc.cfgr.lse(LSEClock::new(LSEClockMode::Oscillator)).freeze();
    let clocks = rcc.cfgr.lsi().freeze();

    let mut rtc = Rtc::new(p.RTC, 249, 127, RtcClock::Lsi, clocks, &mut p.PWR).unwrap();

    rtc.set_datetime(&PrimitiveDateTime::new(date!(2019 - 01 - 01), time!(23:59:50)))
        .unwrap();
    let mut delay = p.TIM5.delay_us(&clocks);
    // Alternatively:
    // rtc.set_date(&date!(2019 - 01 - 01)).unwrap();
    // rtc.set_time(&time!(23:59)).unwrap();
    // Or:
    // rtc.set_year(2019).unwrap();
    // rtc.set_month(12).unwrap();
    // rtc.set_day(31).unwrap();
    // rtc.set_hours(23).unwrap();
    // rtc.set_minutes(59).unwrap();
    // rtc.set_seconds(59).unwrap();
    loop {
        rprintln!("{}", rtc.get_datetime());
        delay.delay(500.millis()).unwrap();
    }
}
