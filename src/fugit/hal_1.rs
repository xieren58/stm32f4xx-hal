use super::{Counter, Delay, Error, Instance, SysCounter};

use embedded_hal_one::delay::blocking::DelayUs;
use embedded_hal_one::timer::{
    nb::{Cancel, CountDown},
    Periodic,
};
use fugit::{ExtU32, MicrosDurationU32, TimerDurationU32};

impl<TIM: Instance, const FREQ: u32> DelayUs for Delay<TIM, FREQ> {
    type Error = Error;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.delay(us.micros())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay(ms.millis())
    }
}

impl<TIM: Instance, const FREQ: u32> Periodic for Counter<TIM, FREQ> {}

impl<TIM: Instance, const FREQ: u32> CountDown for Counter<TIM, FREQ> {
    type Time = TimerDurationU32<FREQ>;
    type Error = Error;

    fn start<T>(&mut self, timeout: T) -> Result<(), Self::Error>
    where
        T: Into<Self::Time>,
    {
        self.start(timeout.into())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        self.wait()
    }
}

impl<TIM: Instance, const FREQ: u32> Cancel for Counter<TIM, FREQ> {
    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.cancel()
    }
}

impl CountDown for SysCounter {
    type Time = MicrosDurationU32;
    type Error = Error;

    fn start<T>(&mut self, timeout: T) -> Result<(), Self::Error>
    where
        T: Into<Self::Time>,
    {
        self.start(timeout.into())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        self.wait()
    }
}

impl Cancel for SysCounter {
    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.cancel()
    }
}
