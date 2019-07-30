use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use core::time::Duration;
use cortex_m::peripheral::syst::RegisterBlock;
use lpc11xx::SYST;

/// Global configuration options for the SysTick driver.
pub struct Options {
    /// Frequency of the system clock in ticks per second.
    ///
    /// # Warning
    ///
    /// If this is not accurate then the SysTick timer will measure time
    /// incorrectly. Set this value to the processor's clock frequency.
    pub clock_frequency: u32,
}

/// Async SysTick driver for LPC11xx microcontrollers.
///
/// This driver is intended for asynchronously representing short delays of a
/// few microseconds to a few milliseconds in a power-efficient way without
/// needing to use a valuable general-purpose timer. A periodic mode is also
/// implemented to support regularly scheduled events without drift (this could
/// be used to implement much longer delays but keep in mind the possibility of
/// missed events; a general purpose timer could be more efficient and safer).
pub struct Driver {
    systick: SYST,
    calculator: TickCalculator,
}

impl Driver {
    /// Initialize the driver. There are no driver prerequisites.
    pub fn initialize(mut systick: SYST, options: Options) -> Self {
        systick.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);

        systick.disable_counter();
        systick.disable_interrupt();

        let calculator = TickCalculator::from_frequency(options.clock_frequency);

        Self {
            systick,
            calculator,
        }
    }

    /// Relinquishes ownership of the underlying peripheral.
    #[inline]
    pub fn into_inner(self) -> SYST {
        self.systick
    }

    /// Pauses asynchronously for approximately the specified amount of time.
    ///
    /// # Panics
    ///
    /// Panics if the duration specified is too long to count down using the
    /// SysTick peripheral. The SysTick should be used for durations shorter
    /// than a handful of milliseconds at most as it is only a 24-bit timer.
    pub async fn pause(&mut self, duration: Duration) {
        assert_eq!(duration.as_secs(), 0, "duration is too large");

        if duration.subsec_micros() != 0 {
            self.pause_impl(duration.subsec_micros()).await;

            self.systick.disable_counter();
        }
    }

    /// Same as [pause](#method.pause) but accepts microseconds directly.
    pub async fn pause_micros(&mut self, microseconds: u32) {
        assert!(microseconds < 1_000_000, "duration is too large");

        if microseconds != 0 {
            self.pause_impl(microseconds).await;

            self.systick.disable_counter();
        }
    }

    /// Configures the SysTick timer as a periodic timed event.
    ///
    /// # Panics
    ///
    /// Panics if the duration specified is too long to count down using the
    /// SysTick peripheral. The SysTick should be used for durations shorter
    /// than a handful of milliseconds at most as it is only a 24-bit timer.
    ///
    /// # Warning
    ///
    /// Be careful as it is very easy to overload the processor with interrupts
    /// by using a very small period such as less than a few dozen microseconds.
    pub fn periodic(&mut self, period: Duration) -> Periodic {
        assert_eq!(period.as_secs(), 0, "duration is too large");

        Periodic::new(&mut self.systick, &self.calculator, period.subsec_micros())
    }

    /// Same as [periodic](#method.periodic) but accepts microseconds directly.
    pub fn periodic_micros(&mut self, microseconds: u32) -> Periodic {
        assert!(microseconds < 1_000_000, "duration is too large");

        Periodic::new(&mut self.systick, &self.calculator, microseconds)
    }

    /// Handles an active peripheral interrupt.
    #[inline]
    pub fn handle_interrupt(&mut self) {
        Self::handle_interrupt_impl(&self.systick)
    }

    /// Equivalent to [handle_interrupt](#method.handle_interrupt) but you must
    /// guarantee that there are currently no live references to the driver.
    #[inline]
    pub unsafe fn handle_interrupt_unchecked() {
        Self::handle_interrupt_impl(&*SYST::ptr())
    }

    #[inline]
    fn handle_interrupt_impl(systick: &RegisterBlock) {
        // No nice register API here, we have to bit-twiddle manually to disable the
        // interrupt. This is to prevent spurious interrupts and PendSV wakeups when
        // using short pause intervals, and makes driver bookkeeping a bit simpler.

        unsafe {
            // ENABLE + COUNTFLAG + use half CPU clock
            systick.csr.write(0b1_0000_0000_0000_0001);
        }

        lpc11xx::SCB::set_pendsv();
    }

    #[inline]
    fn pause_impl<'a>(&'a mut self, microseconds: u32) -> impl Future<Output = ()> + 'a {
        let ticks = self.calculator.tick_count_for_duration(microseconds);

        self.systick.set_reload(ticks - 1);
        self.systick.clear_current();
        self.systick.enable_counter();

        TimerFuture::new(&mut self.systick)
    }
}

/// Wrapper type representing a periodic SysTick timer.
///
/// Using the periodic timer mode is better if you need the events to happen at
/// fixed intervals. Pausing repeatedly in a loop will introduce some drift due
/// to the time taken in the loop doing work. The periodic timer has no drift.
pub struct Periodic<'a> {
    systick: &'a mut SYST,
}

impl<'a> Periodic<'a> {
    fn new(systick: &'a mut SYST, calculator: &TickCalculator, period: u32) -> Self {
        assert_ne!(period, 0);

        systick.set_reload(calculator.tick_count_for_duration(period) - 1);
        systick.clear_current();
        systick.enable_counter();

        Self { systick }
    }

    /// Pauses asynchronously until the next periodic event.
    pub async fn pause(&mut self) {
        TimerFuture::new(&mut self.systick).await
    }
}

impl Drop for Periodic<'_> {
    fn drop(&mut self) {
        self.systick.disable_counter();
    }
}

struct TimerFuture<'a> {
    systick: &'a mut SYST,
}

impl<'a> TimerFuture<'a> {
    pub fn new(systick: &'a mut SYST) -> Self {
        systick.enable_interrupt();

        Self { systick }
    }
}

impl Future for TimerFuture<'_> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        if self.systick.has_wrapped() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

struct TickCalculator {
    num_one_us: u16,
    den_one_us: u16,
    num_256_us: u16,
    den_256_us: u16,
}

impl TickCalculator {
    pub fn from_frequency(freq: u32) -> Self {
        assert!(freq <= 50_000_000);

        // On the LPC11xx, the reference clock for the SysTick timer is defined to be
        // exactly half of the system clock; there is no dedicated SysTick reference.

        let d1 = (64 * freq + 15625) / 31250;
        let d2 = (freq + 1) / 2 + (freq * 6 + 125) / 250 + (freq * 9 + 15625) / 31250;

        Self {
            num_one_us: (d1 / 4096) as u16,
            den_one_us: (d1 % 4096) as u16,
            num_256_us: (d2 / 4096) as u16,
            den_256_us: (d2 % 4096) as u16,
        }
    }

    fn approximate_tick_count_micros(&self, us: u32) -> u32 {
        debug_assert!(us < 1_000_000);

        // The fixed-point approximation below has been exhaustively tested for all
        // frequencies up to 50MHz (the maximum device processor frequency) and all
        // microsecond durations from 0us through to 999999us.
        //
        // The approximation will always return a tick count within +/- 2.425 ticks
        // of the expected tick count for any given frequency and duration. However
        // the approximation is exact for frequencies that are a multiple of 2MHz.
        //
        // In exchange it requires no long multiplication and no division of any
        // kind and as such is suitable when durations are not known in advance.

        let q = us / 256;
        let r = us % 256;

        let tq = q * u32::from(self.num_256_us) + (q * u32::from(self.den_256_us) + 2048) / 4096;
        let tr = r * u32::from(self.num_one_us) + (r * u32::from(self.den_one_us) + 2048) / 4096;

        tq + tr
    }

    pub fn tick_count_for_duration(&self, us: u32) -> u32 {
        let ticks = self.approximate_tick_count_micros(us);
        assert!(ticks <= 16_777_216, "duration too large");

        ticks
    }
}

#[cfg(test)]
mod tests {
    use super::TickCalculator;

    #[test]
    fn tick_calculator_correctness() {
        // The accuracy of the tick count approximation has already been verified; this
        // should just be a check to verify that the logic was not accidentally broken.

        let frequency_50mhz = TickCalculator::from_frequency(50_000_000);
        let frequency_29mhz = TickCalculator::from_frequency(29_000_000);
        let frequency_weird = TickCalculator::from_frequency(34_193_617);

        assert_eq!(frequency_50mhz.tick_count_for_duration(0), 0);
        assert_eq!(frequency_50mhz.tick_count_for_duration(1), 25);
        assert_eq!(frequency_50mhz.tick_count_for_duration(128), 3200);
        assert_eq!(frequency_50mhz.tick_count_for_duration(1000), 25000);
        assert_eq!(frequency_50mhz.tick_count_for_duration(181847), 4546175);

        assert_eq!(frequency_29mhz.tick_count_for_duration(0), 0);
        assert_eq!(frequency_29mhz.tick_count_for_duration(1), 15);
        assert_eq!(frequency_29mhz.tick_count_for_duration(2), 29);
        assert_eq!(frequency_29mhz.tick_count_for_duration(128), 1856);
        assert_eq!(frequency_29mhz.tick_count_for_duration(1000), 14500);
        assert_eq!(frequency_29mhz.tick_count_for_duration(97841), 1418695);

        assert_eq!(frequency_weird.tick_count_for_duration(0), 0);
        assert_eq!(frequency_weird.tick_count_for_duration(1), 17);
        assert_eq!(frequency_weird.tick_count_for_duration(6), 103);
        assert_eq!(frequency_weird.tick_count_for_duration(10), 171);
        assert_eq!(frequency_weird.tick_count_for_duration(40), 684);
        assert_eq!(frequency_weird.tick_count_for_duration(128), 2188);
        assert_eq!(frequency_weird.tick_count_for_duration(1234), 21097);
        assert_eq!(frequency_weird.tick_count_for_duration(81849), 1399357);
    }

    #[test]
    fn tick_calculator_within_range() {
        let ticks_50mhz = TickCalculator::from_frequency(50_000_000);

        // This is the largest duration that still fits the timer at 50MHz
        assert_eq!(ticks_50mhz.tick_count_for_duration(671088), 16777200);
    }

    #[test]
    #[should_panic(expected = "duration too large")]
    fn tick_calculator_out_of_range() {
        let ticks_50mhz = TickCalculator::from_frequency(50_000_000);

        let _ = ticks_50mhz.tick_count_for_duration(671089);
    }
}
