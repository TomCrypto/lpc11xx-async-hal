use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use lpc11xx::adc::RegisterBlock;
use lpc11xx::ADC as ADCDevice;

// ERRATA: https://www.nxp.com/docs/en/errata/ES_LPC111X.pdf
//
// The ADC peripheral has a few problems. In particular, hardware triggering on
// the PIO0_2 and the PIO1_5 pins is unreliable with no workaround, so just
// don't support them. Also, the Global Data register is unreliable in most ADC
// modes, so just don't use it (we don't really need it in this implementation).

/// Global configuration options for the ADC driver.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Clock divisor to the ADC input clock (the APB clock).
    ///
    /// # Note
    ///
    /// It is required that `PCLK / (divisor + 1) <= 4.5MHz`.
    pub clock_divisor: u8,
}

/// A configurable hardware scan accuracy for the ADC.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Accuracy {
    TenBit = 0,
    NineBit = 1,
    EightBit = 2,
    SevenBit = 3,
    SixBit = 4,
    FiveBit = 5,
    FourBit = 6,
    ThreeBit = 7,
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
}

impl Driver {
    /// Initialize the driver. See UM10398 25.2 for prerequisites.
    pub fn initialize(adc: ADCDevice, options: Options) -> Self {
        adc.cr.write(|w| w.clkdiv().bits(options.clock_divisor));
        adc.inten.reset(); // get interrupts into a known state

        Self { adc }
    }

    /// Relinquishes ownership of the underlying peripheral.
    #[inline]
    pub fn into_inner(self) -> ADCDevice {
        self.adc
    }

    /// Samples an analog channel in software mode to 10-bit accuracy.
    pub async fn sample(&mut self, channel: Channel) -> VoltageRange {
        let mask = channel.as_channel_mask();

        self.adc
            .inten
            .write(|w| w.adginten().clear_bit().adinten().bits(mask));

        self.adc.cr.modify(|_, w| {
            w.sel()
                .bits(mask)
                .burst()
                .swmode()
                .clks()
                .ten_bit()
                .start()
                .start()
        });

        unsafe {
            ADC_CTX.status = Status::Software(channel);
        }

        (MeasurementFuture.await)[channel as usize]
    }

    /// Begins a continuous hardware scan on an arbitrary set of ADC channels.
    #[inline]
    pub fn scan(&mut self, channels: u8, accuracy: Accuracy) -> HardwareScan {
        HardwareScan::new(&self.adc, channels, accuracy)
    }

    /// Handles an active peripheral interrupt.
    #[inline]
    pub fn handle_interrupt(&mut self) {
        Self::handle_interrupt_impl(&self.adc)
    }

    /// Equivalent to [handle_interrupt](#method.handle_interrupt) but you must
    /// guarantee that there are currently no live references to the driver.
    #[inline]
    pub unsafe fn handle_interrupt_unchecked() {
        Self::handle_interrupt_impl(&*ADCDevice::ptr())
    }

    fn handle_interrupt_impl(adc: &RegisterBlock) {
        let ctx: &mut AdcContext = unsafe { &mut ADC_CTX };

        match ctx.status {
            Status::Software(channel) => {
                let register = adc.dr[channel as usize].read();

                if register.done().bit_is_clear() {
                    return; // data not available
                }

                ctx.voltages[channel as usize] =
                    VoltageRange::from_raw(register.v_vref().bits(), Accuracy::TenBit);

                // Make sure to stop the ADC conversion
                adc.cr.modify(|_, w| w.start().stop());
            }
            Status::Hardware(mask, accuracy) => {
                if adc.stat.read().done().bits() & mask != mask {
                    return; // we don't have every channel yet
                }

                if mask & 0b0000_0001 != 0 {
                    ctx.voltages[0] =
                        VoltageRange::from_raw(adc.dr[0].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0000_0010 != 0 {
                    ctx.voltages[1] =
                        VoltageRange::from_raw(adc.dr[1].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0000_0100 != 0 {
                    ctx.voltages[2] =
                        VoltageRange::from_raw(adc.dr[2].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0000_1000 != 0 {
                    ctx.voltages[3] =
                        VoltageRange::from_raw(adc.dr[3].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0001_0000 != 0 {
                    ctx.voltages[4] =
                        VoltageRange::from_raw(adc.dr[4].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0010_0000 != 0 {
                    ctx.voltages[5] =
                        VoltageRange::from_raw(adc.dr[5].read().v_vref().bits(), accuracy);
                }

                if mask & 0b0100_0000 != 0 {
                    ctx.voltages[6] =
                        VoltageRange::from_raw(adc.dr[6].read().v_vref().bits(), accuracy);
                }

                if mask & 0b1000_0000 != 0 {
                    ctx.voltages[7] =
                        VoltageRange::from_raw(adc.dr[7].read().v_vref().bits(), accuracy);
                }
            }
            Status::Complete => {}
        }

        // Stop all ADC interrupts if we get down here
        adc.inten.write(|w| w.adginten().clear_bit());

        if let Status::Complete = ctx.status {
            // do nothing (reverse pattern)
        } else {
            ctx.status = Status::Complete;
            lpc11xx::SCB::set_pendsv();
        }
    }
}

impl core::fmt::Debug for Driver {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Driver {{ /* device ADC peripheral */ }}")
    }
}

/// Wrapper type representing continous scanning mode.
///
/// The ADC peripheral can continously scan a set of ADC channels in hardware
/// without disturbing the processor. The ADC channels may be sampled on the fly
/// by the processor or interrupts can be selectively enabled for some channels.
pub struct HardwareScan<'a> {
    adc: &'a ADCDevice,
    accuracy: Accuracy,
    channels: u8,
}

impl<'a> HardwareScan<'a> {
    fn new(adc: &'a ADCDevice, channels: u8, accuracy: Accuracy) -> Self {
        adc.inten.write(|w| w.adginten().clear_bit());

        // Note: the Accuracy enum already has compatible values for the CLKS field so
        // that we do not rely on the optimizer to have to figure this out for itself.

        adc.cr.modify(|_, w| {
            w.sel()
                .bits(channels)
                .burst()
                .hwmode()
                .clks()
                .bits(accuracy as u8)
                .start()
                .stop()
        });

        Self {
            adc,
            accuracy,
            channels,
        }
    }

    /// Samples a set of analog channels simultaneously.
    ///
    /// # Return value
    ///
    /// Returns measurements for all analog channels. Measurements will be valid
    /// for those channels that are being continuously monitored *and* are in
    /// the channel mask passed to this method, otherwise they will be zero.
    ///
    /// # Panics
    ///
    /// This method will panic if the channels provided aren't contained in the
    /// set of channels being scanned in this ADC hardware scan.
    pub async fn sample(&mut self, channels: u8) -> &[VoltageRange; 8] {
        assert_eq!(channels & self.channels, channels);

        let ctx = unsafe { &mut ADC_CTX };

        ctx.voltages = [VoltageRange::ground(self.accuracy); 8];
        ctx.status = Status::Hardware(channels, self.accuracy);

        self.adc
            .inten
            .write(|w| w.adginten().clear_bit().adinten().bits(channels));

        MeasurementFuture.await
    }

    /// Samples an analog channel synchronously in a non-blocking fashion.
    ///
    /// # Return value
    ///
    /// If a measurement is available for the selected channel, returns that
    /// measurement and clears it from the ADC peripheral. If no measurement
    /// is available, this method will do nothing and return `None`.
    ///
    /// # Panics
    ///
    /// This method will panic if the channel provided isn't contained in the
    /// set of channels being scanned in this ADC hardware scan.
    pub fn sample_now(&mut self, channel: Channel) -> Option<VoltageRange> {
        assert_ne!(channel.as_channel_mask() & self.channels, 0);

        let register = self.adc.dr[channel as usize].read();

        if register.done().bit_is_set() {
            Some(VoltageRange::from_raw(
                register.v_vref().bits(),
                self.accuracy,
            ))
        } else {
            None
        }
    }
}

impl Drop for HardwareScan<'_> {
    fn drop(&mut self) {
        self.adc.cr.modify(|_, w| w.burst().swmode().start().stop());
    }
}

impl core::fmt::Debug for HardwareScan<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "HardwareScan {{ /* device ADC peripheral */ }}")
    }
}

/// A referred voltage range measured from the ADC peripheral.
///
/// This type represents a range of voltages quantized by the ADC according to
/// its maximum and configured accuracy. The minimum and maximum voltages are
/// related to the ADC measurement as:
///
/// `Vmin / Vref = Nmin / denominator`
///
/// `Vmax / Vref = Nmax / denominator`
///
/// where Vref is the analog reference Vdd = Vdda. This type provides access to
/// `Nmin`, `Nmax` and `denominator` in such a way that these values are simple
/// to interpret. All measurements are proportional to `Vref` as shown above.
///
/// The lower the ADC accuracy setting, the larger `Nmax - Nmin` will be.
#[derive(Clone, Copy, Debug)]
pub struct VoltageRange(u16);

impl VoltageRange {
    /// The voltage range corresponding to "near analog ground".
    #[inline]
    pub const fn ground(accuracy: Accuracy) -> Self {
        Self::from_raw(0, accuracy)
    }

    /// Returns the raw measurement as obtained from the ADC.
    #[inline]
    pub const fn into_raw(self) -> u16 {
        self.0 & 0x3ff
    }

    /// Returns the lower bound `Nmin` for this voltage range.
    #[inline]
    pub const fn lower_bound(self) -> u16 {
        self.0 & 0x3ff
    }

    /// Returns the upper bound `Nmax` for this voltage range.
    #[inline]
    pub const fn upper_bound(self) -> u16 {
        (self.0 & 0x3ff) + (1 << (self.0 >> 10))
    }

    /// Returns the midpoint of this voltage range.
    ///
    /// # Note
    ///
    /// For a 10-bit measurement, the midpoint is always rounded down to the
    /// lower bound as the measured range is quantized to two 10-bit values.
    #[inline]
    pub const fn midpoint(self) -> u16 {
        (self.0 & 0x3ff) + (1 << (self.0 >> 10)) / 2
    }

    /// Returns the denominator used to calculate referred voltages.
    #[inline]
    pub const fn denominator(self) -> u16 {
        1024
    }

    #[inline]
    const fn from_raw(value: u16, accuracy: Accuracy) -> Self {
        let accuracy_bits = accuracy as u16; // ten bits == 0, three bits == 7
        Self((value >> accuracy_bits << accuracy_bits) | (accuracy_bits << 10))
    }
}

struct MeasurementFuture;

impl Future for MeasurementFuture {
    type Output = &'static [VoltageRange; 8];

    fn poll(self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        let ctx = unsafe { &mut ADC_CTX };

        if let Status::Complete = ctx.status {
            Poll::Ready(&ctx.voltages)
        } else {
            Poll::Pending
        }
    }
}

enum Status {
    Software(Channel),
    Hardware(u8, Accuracy),
    Complete,
}

struct AdcContext {
    voltages: [VoltageRange; 8],
    status: Status,
}

static mut ADC_CTX: AdcContext = AdcContext {
    voltages: [VoltageRange::ground(Accuracy::TenBit); 8],
    status: Status::Complete,
};

#[cfg(test)]
mod tests {
    use super::{Accuracy, VoltageRange};

    #[test]
    fn voltage_range_methods() {
        assert_eq!(VoltageRange::ground(Accuracy::ThreeBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::ThreeBit).upper_bound(), 128);
        assert_eq!(VoltageRange::ground(Accuracy::ThreeBit).midpoint(), 64);

        assert_eq!(VoltageRange::ground(Accuracy::FourBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::FourBit).upper_bound(), 64);
        assert_eq!(VoltageRange::ground(Accuracy::FourBit).midpoint(), 32);

        assert_eq!(VoltageRange::ground(Accuracy::FiveBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::FiveBit).upper_bound(), 32);
        assert_eq!(VoltageRange::ground(Accuracy::FiveBit).midpoint(), 16);

        assert_eq!(VoltageRange::ground(Accuracy::SixBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::SixBit).upper_bound(), 16);
        assert_eq!(VoltageRange::ground(Accuracy::SixBit).midpoint(), 8);

        assert_eq!(VoltageRange::ground(Accuracy::SevenBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::SevenBit).upper_bound(), 8);
        assert_eq!(VoltageRange::ground(Accuracy::SevenBit).midpoint(), 4);

        assert_eq!(VoltageRange::ground(Accuracy::EightBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::EightBit).upper_bound(), 4);
        assert_eq!(VoltageRange::ground(Accuracy::EightBit).midpoint(), 2);

        assert_eq!(VoltageRange::ground(Accuracy::NineBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::NineBit).upper_bound(), 2);
        assert_eq!(VoltageRange::ground(Accuracy::NineBit).midpoint(), 1);

        assert_eq!(VoltageRange::ground(Accuracy::TenBit).lower_bound(), 0);
        assert_eq!(VoltageRange::ground(Accuracy::TenBit).upper_bound(), 1);
        assert_eq!(VoltageRange::ground(Accuracy::TenBit).midpoint(), 0);
    }

    #[test]
    fn voltage_range_from_raw_into_raw() {
        for raw in 0..1024 {
            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::TenBit).into_raw(),
                raw
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::NineBit).into_raw(),
                raw & 0b11_1111_1110
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::EightBit).into_raw(),
                raw & 0b11_1111_1100
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::SevenBit).into_raw(),
                raw & 0b11_1111_1000
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::SixBit).into_raw(),
                raw & 0b11_1111_0000
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::FiveBit).into_raw(),
                raw & 0b11_1110_0000
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::FourBit).into_raw(),
                raw & 0b11_1100_0000
            );

            assert_eq!(
                VoltageRange::from_raw(raw, Accuracy::ThreeBit).into_raw(),
                raw & 0b11_1000_0000
            );
        }
    }
}
