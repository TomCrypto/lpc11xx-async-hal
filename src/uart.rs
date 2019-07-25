use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use lpc11xx::uart::RegisterBlock;
use lpc11xx::UART as UARTDevice;

/// Global configuration options for the UART driver.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// The divisor to the UART clock source used to generate all timings.
    pub clock_divisor: u16,
    /// Divisor value for the fractional divider, set to zero to disable.
    pub fractional_div: u8,
    /// Multiplier for the fractional divider, must never be set to zero.
    pub fractional_mul: u8,
    /// The RX FIFO interrupt threshold, allowed values are 1, 4, 8, 14.
    pub rx_threshold: RxThreshold,
    /// Enable hardware flow control. If false, no flow control is used.
    pub flow_control: bool,
}

/// The RX threshold to use for the UART driver.
///
/// This threshold (or "trigger level") determines how many bytes must be in the
/// RX FIFO before a read interrupt is triggered. Normally this should be set to
/// the maximum (14 bytes) unless no flow control is being used and characters
/// are being lost. Lowering this threshold increases UART interrupt pressure.
#[derive(Clone, Copy, Debug)]
pub enum RxThreshold {
    OneByte,
    FourBytes,
    EightBytes,
    FourteenBytes,
}

impl Default for RxThreshold {
    fn default() -> Self {
        Self::FourteenBytes
    }
}

/// Async UART driver for LPC11xx microcontrollers.
///
/// Since UART is a bidirectional protocol, this driver allows sending and
/// receiving at the same time by splitting itself into sending and receiving
/// parts with the `split` method.
///
/// # Baud Rate
///
/// This UART driver does not calculate baud rates directly; you must calculate
/// the correct clock/fractional divider values based on the UART clock
/// frequency; see UM10398 13.5.15.
///
/// # Flow Control
///
/// Only hardware flow control (or no flow control) is supported. If enabled,
/// the modem CTS and RTS pins must have been configured and should be
/// physically connected to something.
///
/// # Error Handling
///
/// No UART errors are handled as they are unreliable, and bytes are read
/// exactly as received. If you need a reliable channel, you must implement your
/// own framing and error correction protocol on top of the UART channel.
///
/// # Frame Format
///
/// The frame format is fixed at 8N1 (8-bit characters, 1 stop bit, no parity)
/// for ease of use and to avoid byte truncation in read or write operations.
pub struct Driver {
    uart: UARTDevice,
}

impl Driver {
    /// Initialize the driver. See UM10398 13.2 for prerequisites.
    pub fn initialize(uart: UARTDevice, options: Options) -> Self {
        uart.lcr.write(|w| w.dlab().enable());

        // This assert is fine in release; the manual specifically states that a zero
        // divisor is treated as if it was 1 since division by zero is not permitted.

        debug_assert!(options.clock_divisor != 0);

        unsafe { &uart.dlm.dlm }.write(|w| w.dlmsb().bits((options.clock_divisor / 256) as u8));
        unsafe { &uart.dll.dll }.write(|w| w.dllsb().bits((options.clock_divisor % 256) as u8));

        // These asserts are safe in release, if the values are out of range then the
        // fractional divider will not work and the baud rate will probably be wrong.

        debug_assert!(options.fractional_mul < 16);
        debug_assert!(options.fractional_div < options.fractional_mul);

        uart.fdr.write(|w| {
            w.divaddval()
                .bits(options.fractional_div)
                .mulval()
                .bits(options.fractional_mul)
        });

        uart.lcr.write(|w| w.dlab().disable().wls().eight());

        uart.mcr.write(|w| {
            w.rtsen()
                .bit(options.flow_control)
                .ctsen()
                .bit(options.flow_control)
        });

        unsafe { &uart.fcr.fcr }.write(|w| match options.rx_threshold {
            RxThreshold::OneByte => w.rxtl().one_word(),
            RxThreshold::FourBytes => w.rxtl().four_words(),
            RxThreshold::EightBytes => w.rxtl().eight_words(),
            RxThreshold::FourteenBytes => w.rxtl().fourteen_words(),
        });

        Driver { uart }
    }

    /// Provides access to send and receive over UART.
    pub fn split(&mut self) -> (Sender, Receiver) {
        unsafe { &self.uart.dlm.ier }.reset();

        unsafe { &self.uart.fcr.fcr }.write(|w| {
            // in theory FIFOEN should never be zero but you never know
            w.fifoen().enable().txfifores().clear().rxfifores().clear()
        });

        (Sender { uart: &self.uart }, Receiver { uart: &self.uart })
    }

    /// Relinquishes ownership of the underlying peripheral.
    pub fn into_inner(self) -> UARTDevice {
        self.uart
    }

    /// Handles an active peripheral interrupt.
    pub fn handle_interrupt(&mut self) {
        Self::handle_interrupt_impl(&self.uart)
    }

    /// Equivalent to [handle_interrupt](#method.handle_interrupt) but you must
    /// guarantee that there are currently no live references to the driver.
    pub unsafe fn handle_interrupt_unchecked() {
        Self::handle_interrupt_impl(&*UARTDevice::ptr())
    }

    fn handle_interrupt_impl(uart: &RegisterBlock) {
        match unsafe { &uart.fcr.iir.read().intid() } {
            lpc11xx::uart::iir::INTIDR::THRE => {
                Self::handle_thre_interrupt(uart, unsafe { &mut TX_CTX })
            }
            lpc11xx::uart::iir::INTIDR::RDA | lpc11xx::uart::iir::INTIDR::CTI => {
                Self::handle_rda_cti_interrupts(uart, unsafe { &mut RX_CTX })
            }
            _ => debug_assert!(false, "unexpected interrupt flag"),
        }
    }

    fn handle_thre_interrupt(uart: &RegisterBlock, ctx: &mut TxContext) {
        if ctx.buf_ptr == ctx.buf_end {
            // This branch should only be hit for "flush" write operations. In that
            // case we set buf_len to a nonzero value to signal the flush was done.

            debug_assert!(ctx.buf_ptr.is_null());

            ctx.buf_len = 1; // TX flush completed
            return Self::notify_tx_complete(uart);
        }

        for _ in 0..16 {
            unsafe { &uart.dll.thr }.write(|w| w.thr().bits(unsafe { *ctx.buf_ptr }));

            ctx.buf_ptr = unsafe { ctx.buf_ptr.add(1) };

            if ctx.buf_ptr == ctx.buf_end {
                return Self::notify_tx_complete(uart);
            }
        }
    }

    fn handle_rda_cti_interrupts(uart: &RegisterBlock, ctx: &mut RxContext) {
        debug_assert!(ctx.buf_ptr != ctx.buf_end);

        while uart.lsr.read().rdr().is_valid() {
            let byte = unsafe { &uart.dll.rbr }.read().rbr().bits();

            unsafe { ctx.buf_ptr.write(byte) };
            ctx.buf_len += 1;

            ctx.buf_ptr = if ctx.options.stop == Some(byte) {
                ctx.buf_end // the stop byte was encountered
            } else {
                unsafe { ctx.buf_ptr.add(1) }
            };

            if ctx.buf_ptr == ctx.buf_end {
                return Self::notify_rx_complete(uart);
            }
        }

        // We must have read between 1 and 16 bytes above, depending on trigger level
        // and interrupt latency. If the caller did not request an exact read, we are
        // allowed to complete the RX operation and let the caller decide what to do.

        if !ctx.options.read_exact {
            ctx.buf_ptr = ctx.buf_end; // finished
            return Self::notify_rx_complete(uart);
        }
    }

    fn notify_tx_complete(uart: &RegisterBlock) {
        Self::disable_tx_interrupts(uart);
        lpc11xx::SCB::set_pendsv();
    }

    fn notify_rx_complete(uart: &RegisterBlock) {
        Self::disable_rx_interrupts(uart);
        lpc11xx::SCB::set_pendsv();
    }

    fn enable_tx_interrupts(uart: &RegisterBlock) {
        unsafe { &uart.dlm.ier }.modify(|_, w| w.threie().enable());
    }

    fn disable_tx_interrupts(uart: &RegisterBlock) {
        unsafe { &uart.dlm.ier }.modify(|_, w| w.threie().disable());
    }

    fn enable_rx_interrupts(uart: &RegisterBlock) {
        unsafe { &uart.dlm.ier }.modify(|_, w| w.rbrie().enable());
    }

    fn disable_rx_interrupts(uart: &RegisterBlock) {
        unsafe { &uart.dlm.ier }.modify(|_, w| w.rbrie().disable());
    }
}

impl core::fmt::Debug for Driver {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Driver {{ /* device UART peripheral */ }}")
    }
}

/// Sending side of the UART driver.
pub struct Sender<'a> {
    uart: &'a UARTDevice,
}

impl<'a> Sender<'a> {
    /// Writes a number of bytes to the UART TX FIFO.
    pub async fn write(&mut self, buffer: &[u8]) -> usize {
        // Fill up the TX FIFO. This is required by the UART hardware to actually
        // trigger THRE interrupts (need at least two bytes in the FIFO). If the
        // write is small enough then it might actually be written synchronously
        // into the FIFO without needing to await.

        let (buf_len, written) = (buffer.len(), self.fill_tx_queue(buffer));

        if written == buf_len {
            return written;
        }

        unsafe {
            TX_CTX.buf_ptr = buffer.as_ptr().add(written);
            TX_CTX.buf_end = buffer.as_ptr().add(buf_len);
            TX_CTX.buf_len = buf_len;
        }

        TxFuture::new(&self.uart).await
    }

    /// Flushes the UART TX FIFO.
    pub async fn flush(&mut self) {
        if self.uart.lsr.read().temt().is_empty_() {
            return; // the TX FIFO is already empty
        }

        unsafe {
            TX_CTX.buf_ptr = core::ptr::null();
            TX_CTX.buf_end = core::ptr::null();
            TX_CTX.buf_len = 0; // sentinel len
        }

        TxFuture::new(&self.uart).await;
    }

    fn fill_tx_queue(&mut self, buffer: &[u8]) -> usize {
        for (written, &byte) in buffer.iter().take(16).enumerate() {
            if self.uart.lsr.read().thre().is_valid() {
                return written; // TX FIFO is full
            }

            unsafe {
                self.uart.dll.thr.write(|w| w.thr().bits(byte));
            }
        }

        buffer.len()
    }
}

impl core::fmt::Debug for Sender<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Sender {{ /* device UART peripheral */ }}")
    }
}

/// Receiving side of the UART driver.
pub struct Receiver<'a> {
    uart: &'a UARTDevice,
}

impl<'a> Receiver<'a> {
    /// Reads any number of bytes from the UART RX FIFO. At least one byte will
    /// be read, so this method will not return if the RX FIFO is empty and no
    /// bytes are being sent by the remote end.
    ///
    /// # Return value
    ///
    /// Returns the number of bytes read into the buffer.
    pub async fn read(&mut self, buffer: &mut [u8]) -> usize {
        let options = RxOptions {
            read_exact: false,
            stop: None,
        };

        self.read_with_options(buffer, options).await
    }

    /// Reads an exact number of bytes from the UART RX FIFO.
    ///
    /// # Return value
    ///
    /// Returns the number of bytes read into the buffer, which will always be
    /// equal to `buffer.len()` for this method.
    pub async fn read_exact(&mut self, buffer: &mut [u8]) -> usize {
        let options = RxOptions {
            read_exact: true,
            stop: None,
        };

        self.read_with_options(buffer, options).await
    }

    /// Reads from the UART RX FIFO until the buffer is filled or until a
    /// specific stop character is encountered.
    ///
    /// The stop character will be included as part of the bytes read, if it
    /// was encountered.
    ///
    /// This method is provided to efficiently implement simple line-oriented
    /// protocols without needing to implement a buffered reader on top of the
    /// UART driver.
    ///
    /// # Return value
    ///
    /// Returns the number of bytes read into the buffer.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #![feature(async_await)]
    /// # async fn example() {
    /// # let receiver: lpc11xx_async_hal::uart::Receiver = panic!();
    /// let mut buffer = [0; 64];
    ///
    /// let read_bytes = receiver.read_until(&mut buffer, b'\n').await;
    ///
    /// // possible outcome:
    /// //  - buffer contains b"string\n"
    /// //  - read_bytes equals 7
    /// # }
    /// ```
    pub async fn read_until(&mut self, buffer: &mut [u8], stop: u8) -> usize {
        let options = RxOptions {
            read_exact: true,
            stop: Some(stop),
        };

        self.read_with_options(buffer, options).await
    }

    async fn read_with_options(&mut self, buffer: &mut [u8], options: RxOptions) -> usize {
        // We don't need to read the RX FIFO queue because we will get a CTI interrupt
        // if it is not empty. We do not want to process any reads here to avoid
        // duplicating read logic.

        let buf_len = buffer.len();

        if buf_len == 0 {
            return 0;
        }

        unsafe {
            RX_CTX.buf_ptr = buffer.as_mut_ptr();
            RX_CTX.buf_end = buffer.as_mut_ptr().add(buf_len);
            RX_CTX.options = options;
            RX_CTX.buf_len = 0;
        }

        RxFuture::new(&self.uart).await
    }
}

impl core::fmt::Debug for Receiver<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Receiver {{ /* device UART peripheral */ }}")
    }
}

struct TxFuture;
struct RxFuture;

impl TxFuture {
    pub fn new(uart: &UARTDevice) -> Self {
        Driver::enable_tx_interrupts(uart);

        Self
    }
}

impl Drop for TxFuture {
    fn drop(&mut self) {
        Driver::disable_tx_interrupts(unsafe { &*UARTDevice::ptr() });
    }
}

impl Future for TxFuture {
    type Output = usize;

    fn poll(self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        let ctx = unsafe { &mut TX_CTX };

        if ctx.buf_ptr == ctx.buf_end {
            match ctx.buf_len {
                0 => Poll::Pending,
                n => Poll::Ready(n),
            }
        } else {
            Poll::Pending
        }
    }
}

impl RxFuture {
    pub fn new(uart: &UARTDevice) -> Self {
        Driver::enable_rx_interrupts(uart);

        Self
    }
}

impl Drop for RxFuture {
    fn drop(&mut self) {
        Driver::disable_rx_interrupts(unsafe { &*UARTDevice::ptr() });
    }
}

impl Future for RxFuture {
    type Output = usize;

    fn poll(self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        let ctx = unsafe { &mut RX_CTX };

        if ctx.buf_ptr == ctx.buf_end {
            Poll::Ready(ctx.buf_len)
        } else {
            Poll::Pending
        }
    }
}

struct RxOptions {
    read_exact: bool,
    stop: Option<u8>,
}

struct TxContext {
    buf_ptr: *const u8,
    buf_end: *const u8,
    buf_len: usize,
}

struct RxContext {
    buf_ptr: *mut u8,
    buf_end: *mut u8,
    options: RxOptions,
    buf_len: usize,
}

static mut TX_CTX: TxContext = TxContext {
    buf_ptr: core::ptr::null(),
    buf_end: core::ptr::null(),
    buf_len: 0,
};

static mut RX_CTX: RxContext = RxContext {
    buf_ptr: core::ptr::null_mut(),
    buf_end: core::ptr::null_mut(),
    options: RxOptions {
        read_exact: false,
        stop: None,
    },
    buf_len: 0,
};
