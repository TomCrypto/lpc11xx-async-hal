use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use lpc11xx::adc::RegisterBlock;
use lpc11xx::ADC as ADCDevice;

// TODO: add support for start-on-timer-match for software sampling
// don't include PIO0_2/PIO1_5 as these are known to be buggy, see
// errata sheet: https://www.nxp.com/docs/en/errata/ES_LPC1114.pdf

/// Global configuration options for the ADC driver.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Divisor such that PCLK / (divisor + 1) <= 4.5MHz.
    pub clock_divisor: u8,
}

pub enum Accuracy {
    ThreeBits,
    FourBits,
    FiveBits,
    SixBits,
    SevenBits,
    EightBits,
    NineBits,
    TenBits,
}

/// An analog channel corresponding to an analog function pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Channel {
    AD0 = 0,
    AD1 = 1,
    AD2 = 2,
    AD3 = 3,
    AD4 = 4,
    AD5 = 5,
    AD6 = 6,
    AD7 = 7,
}

impl Channel {
    fn as_channel_mask(self) -> u8 {
        1u8 << self as u8
    }
}

/// Async ADC driver for LPC11xx microcontrollers.
pub struct Driver {
    adc: ADCDevice,
    clock_divisor: u8,
}

impl Driver {
    /// Initialize the driver. See UM10398 25.2 for prerequisites.
    pub fn initialize(adc: ADCDevice, options: Options) -> Self {
        Self {
            adc,
            clock_divisor: options.clock_divisor,
        }
    }

    /// Relinquishes ownership of the underlying peripheral.
    pub fn into_inner(self) -> ADCDevice {
        self.adc
    }

    /// Samples an analog channel in software mode to 10-bit accuracy.
    pub async fn sample(&mut self, channel: Channel) -> VoltageRange {
        self.adc.inten.reset();

        unsafe {
            self.adc.cr.write(|w| {
                w.sel()
                    .bits(channel.as_channel_mask())
                    .burst()
                    .swmode()
                    .clkdiv()
                    .bits(self.clock_divisor)
                    .start()
                    .start()
            });
        }

        unsafe {
            ADC_CTX.voltages_measured = false;
            ADC_CTX.channel = Some(channel);
        }

        VoltageRange::from_adc((MeasurementFuture.await)[channel as usize])
    }

    /// Handles an active peripheral interrupt.
    pub fn handle_interrupt(&mut self) {
        Self::handle_interrupt_impl(&self.adc)
    }

    /// Equivalent to [handle_interrupt](#method.handle_interrupt) but you must
    /// guarantee that there are currently no live references to the driver.
    pub unsafe fn handle_interrupt_unchecked() {
        Self::handle_interrupt_impl(&*ADCDevice::ptr())
    }

    fn handle_interrupt_impl(adc: &RegisterBlock) {
        let ctx: &mut AdcContext = unsafe { &mut ADC_CTX };

        let stat = adc.stat.read();

        if let Some(channel) = ctx.channel {
            let register = &adc.dr[channel as usize].read();

            if register.done().bit_is_set() {
                ctx.voltages[channel as usize] = register.v_vref().bits();
                ctx.voltages_measured = true;

                adc.cr.modify(|_, w| w.start().stop());

                lpc11xx::SCB::set_pendsv();
            }
        } else {
            // get the channel mask first based on the interrupt mask
            let mask = adc.inten.read().adintenn().bits();

            let done = stat.done().bits();
            let overrun = stat.overrun().bits();

            // TODO: race condition here??

            if done & mask == mask && overrun & mask == 0 {
                ctx.voltages[0] = adc.dr[0].read().v_vref().bits();
                ctx.voltages[1] = adc.dr[1].read().v_vref().bits();
                ctx.voltages[2] = adc.dr[2].read().v_vref().bits();
                ctx.voltages[3] = adc.dr[3].read().v_vref().bits();
                ctx.voltages[4] = adc.dr[4].read().v_vref().bits();
                ctx.voltages[5] = adc.dr[5].read().v_vref().bits();
                ctx.voltages[6] = adc.dr[6].read().v_vref().bits();
                ctx.voltages[7] = adc.dr[7].read().v_vref().bits();

                ctx.voltages_measured = true;

                lpc11xx::SCB::set_pendsv();
            }
        }
    }
}

impl core::fmt::Debug for Driver {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Driver {{ /* device ADC peripheral */ }}")
    }
}

/// A referred voltage range measured from the ADC peripheral.
///
/// This type represents a range of voltages of the form:
///
/// ```text
/// [Vdda * (N / B), Vdda * ((N + 1) / B)]
/// ```
///
/// where B = 1024 for this ADC peripheral and N is the 10-bit
/// ADC value measured. This type exposes N / B as a fraction.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct VoltageRange(u16);

impl VoltageRange {
    /// The minimum measurable voltage range.
    #[inline]
    pub const fn minimum() -> Self {
        Self(0)
    }

    /// The maximum measurable voltage range.
    #[inline]
    pub const fn maximum() -> Self {
        Self(1023)
    }

    /// Returns the lower bound for the measured Vdda-referred voltage.
    #[inline]
    pub const fn as_referred_voltage_lower_bound(self) -> (u16, u16) {
        (self.0, 1024)
    }

    /// Returns the upper bound for the measured Vdda-referred voltage.
    #[inline]
    pub const fn as_referred_voltage_upper_bound(self) -> (u16, u16) {
        (self.0 + 1, 1024)
    }

    fn from_adc(value: u16) -> Self {
        Self(value)
    }
}

struct MeasurementFuture;

impl Drop for MeasurementFuture {
    fn drop(&mut self) {
        let adc = unsafe { &*ADCDevice::ptr() };

        adc.inten.write(|w| w.adginten().clear_bit());
        adc.cr.modify(|_, w| w.start().stop());
    }
}

impl Future for MeasurementFuture {
    type Output = &'static [u16; 8];

    fn poll(self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        let ctx = unsafe { &mut ADC_CTX };

        if ctx.voltages_measured {
            Poll::Ready(&ctx.voltages)
        } else {
            Poll::Pending
        }
    }
}

struct AdcContext {
    channel: Option<Channel>,
    voltages_measured: bool,
    voltages: [u16; 8],
}

static mut ADC_CTX: AdcContext = AdcContext {
    voltages: [0; 8],
    voltages_measured: false,
    channel: None,
};
