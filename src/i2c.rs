use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use lpc11xx::i2c::RegisterBlock;
use lpc11xx::I2C as I2CDevice;

/// A possible error condition on the I2C bus.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// No slave on the bus acknowledged this address.
    AddressNotAcknowledged(Address),
    /// A data byte was not acknowledged by the slave.
    DataByteNotAcknowledged,
    /// The driver lost arbitration to another master.
    ArbitrationLost,
}

/// Global configuration options for the I2C driver.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Clock divisor for I2C_PCLK for SCL high.
    pub clock_divisor_hi: u16,
    /// Clock divisor for I2C_PCLK for SCL low.
    pub clock_divisor_lo: u16,
}

/// A command on the I2C bus.
#[derive(Debug)]
pub enum Command<'a> {
    /// Reads some bytes from an I2C address.
    Read(Address, &'a mut [u8]),
    /// Write some bytes to an I2C address.
    Write(Address, &'a [u8]),
}

/// Async I2C driver for LPC11xx microcontrollers.
///
/// This driver allows taking control of the I2C bus as a master and executing
/// read/write commands on the bus to arbitrary device addresses. Repeated start
/// is supported in the form of transactions, which are a sequence of commands
/// separated by repeated start conditions.
pub struct Driver {
    i2c: I2CDevice,
}

impl Driver {
    /// Initialize the driver. See UM10398 15.2 for prerequisites.
    pub fn initialize(i2c: I2CDevice, options: Options) -> Self {
        // Start the I2C controller in a disabled state. We will only enable the
        // controller when we are executing a (master only) transaction.

        i2c.conclr.write(|w| w.i2en().disable());

        i2c.sclh.write(|w| w.sclh().bits(options.clock_divisor_hi));
        i2c.scll.write(|w| w.scll().bits(options.clock_divisor_lo));

        Self { i2c }
    }

    /// Relinquishes ownership of the underlying peripheral.
    #[inline]
    pub fn into_inner(self) -> I2CDevice {
        self.i2c
    }

    /// Probes a particular I2C address to determine if a slave is present.
    ///
    /// This is equivalent to writing a zero-length message to that slave. This
    /// method will automatically retry indefinitely in the case of arbitration
    /// loss for convenience.
    pub async fn probe(&mut self, address: Address) -> bool {
        let commands = [Command::Write(address, &[])];

        while let Err(err) = self.exec_transaction(&commands).await {
            if let Error::ArbitrationLost = err {
                continue;
            } else {
                return false;
            }
        }

        true
    }

    /// Executes a single command on the bus.
    ///
    /// This is implemented as a transaction containing that single command.
    #[inline]
    pub async fn exec_command(&mut self, command: Command<'_>) -> Result<(), Error> {
        self.exec_transaction(core::slice::from_ref(&command)).await
    }

    /// Executes a transaction on the bus as an sequence of commands.
    ///
    /// # I2C semantics
    ///
    /// Each command will be separated by a repeated start condition on the bus.
    /// If you want a start/stop condition instead, use multiple transactions,
    /// which will also give other masters a chance to acquire the bus.
    ///
    /// The transaction fails with an error as soon as an address or data byte
    /// in any command is not acknowledged by a slave. The transaction completes
    /// when all commands complete.
    ///
    /// **WARNING**: many simple I2C slave devices will not respond to write
    /// commands until a stop condition (not a repeated start) is observed on
    /// the bus. Read your I2C device's datasheet to find out what it expects.
    ///
    /// # Example
    ///
    /// Sending some bytes and receiving back from a device:
    ///
    /// ```no_run
    /// # #![feature(async_await)]
    /// # async fn example() {
    /// # use lpc11xx_async_hal::i2c::*;
    /// # let i2c: Driver = panic!();
    /// let address = Address::from_right_justified_byte(0xa);
    ///
    /// let mut response = [0; 4];
    ///
    /// let commands = [
    ///     // send some bytes to the device
    ///     Command::Write(address, &[0x12, 0x34, 0x56, 0x78]),
    ///     // receive some response bytes back from the device
    ///     Command::Read(address, &mut response)
    /// ];
    ///
    /// if let Err(error) = i2c.exec_transaction(&commands).await {
    ///     // handle error (e.g. address not acknowledged)
    /// } else {
    ///     // read contents of the response buffer
    /// }
    /// # }
    /// ```
    pub async fn exec_transaction(&mut self, commands: &[Command<'_>]) -> Result<(), Error> {
        if let Some(command) = commands.first() {
            let static_command: &Command<'static> = unsafe { core::mem::transmute(command) };

            let ptr = static_command as *const Command<'static>;
            let end = unsafe { ptr.add(commands.len()) };

            let (buf_ptr, buf_len) = match command {
                Command::Read(_, buf) => (buf.as_ptr() as *mut u8, buf.len()),
                // note that we promise never to write into the Write buffers
                Command::Write(_, buf) => (buf.as_ptr() as *mut u8, buf.len()),
            };

            let buf_end = unsafe { buf_ptr.add(buf_len) };

            let ctx = unsafe { &mut TXN_CTX };

            ctx.command_ptr = ptr;
            ctx.command_end = end;
            ctx.buf_ptr = buf_ptr;
            ctx.buf_end = buf_end;
            ctx.error = Ok(());
        } else {
            return Ok(());
        }

        // Configure controller for START_CONDITION_TRANSMITTED
        self.i2c.conclr.write(|w| w.aa().clear().si().clear());
        self.i2c.conset.write(|w| w.sta().set());

        TransactionFuture::new(&self.i2c).await
    }

    /// Handles an active peripheral interrupt.
    #[inline]
    pub fn handle_interrupt(&mut self) {
        Self::handle_interrupt_impl(&self.i2c)
    }

    /// Equivalent to [handle_interrupt](#method.handle_interrupt) but you must
    /// guarantee that there are currently no live references to the driver.
    #[inline]
    pub unsafe fn handle_interrupt_unchecked() {
        Self::handle_interrupt_impl(&*I2CDevice::ptr())
    }

    fn handle_interrupt_impl(i2c: &RegisterBlock) {
        let ctx = unsafe { &mut TXN_CTX };

        let status = i2c.stat.read().status();

        match status {
            lpc11xx::i2c::stat::STATUSR::START_CONDITION_TRANSMITTED
            | lpc11xx::i2c::stat::STATUSR::REPEATED_START_CONDITION_TRANSMITTED => {
                debug_assert_ne!(ctx.command_ptr, ctx.command_end);

                i2c.dat.write(|w| {
                    // Load the address + R/W for the command
                    w.data().bits(ctx.command_address_byte())
                });

                i2c.conclr.write(|w| w.sta().clear().si().clear());
            }
            lpc11xx::i2c::stat::STATUSR::WRITE_ADDRESS_TRANSMITTED_ACK
            | lpc11xx::i2c::stat::STATUSR::DATA_BYTE_TRANSMITTED_ACK => {
                match ctx.read_next_byte() {
                    TransactionStatus::NextByte(byte) => {
                        i2c.dat.write(|w| w.data().bits(byte));

                        i2c.conclr.write(|w| w.sta().clear().si().clear());
                    }
                    TransactionStatus::MoreCommands => {
                        i2c.conclr.write(|w| w.si().clear());
                        i2c.conset.write(|w| w.sta().set());
                    }
                    TransactionStatus::LastCommand => {
                        i2c.conclr.write(|w| w.sta().clear().si().clear());
                        i2c.conset.write(|w| w.sto().set());

                        lpc11xx::SCB::set_pendsv();
                    }
                }
            }
            lpc11xx::i2c::stat::STATUSR::READ_ADDRESS_TRANSMITTED_ACK
            | lpc11xx::i2c::stat::STATUSR::DATA_BYTE_RECEIVED_ACK => {
                // These two states are actually identical except that READ_ADDRESS does not
                // have a data byte yet; if we allow the read then a data byte will be read.

                if status == lpc11xx::i2c::stat::STATUSR::DATA_BYTE_RECEIVED_ACK {
                    ctx.write_byte(i2c.dat.read().data().bits());
                }

                if ctx.is_command_complete() {
                    i2c.conclr.write(|w| w.sta().clear().si().clear());
                    i2c.conset.write(|w| w.aa().set());
                } else {
                    i2c.conclr
                        .write(|w| w.sta().clear().si().clear().aa().clear());
                }
            }
            lpc11xx::i2c::stat::STATUSR::READ_ADDRESS_TRANSMITTED_NOT_ACK
            | lpc11xx::i2c::stat::STATUSR::DATA_BYTE_RECEIVED_NOT_ACK => {
                // These are not error states; we sent the NOT ACK in the states above because
                // we need the slave to stop sending; the command is complete.

                if ctx.advance_to_next_command().is_some() {
                    i2c.conclr.write(|w| w.si().clear());
                    i2c.conset.write(|w| w.sta().set());
                } else {
                    i2c.conclr
                        .write(|w| w.sta().clear().si().clear().aa().clear());
                    i2c.conset.write(|w| w.sto().set());

                    lpc11xx::SCB::set_pendsv();
                }
            }
            lpc11xx::i2c::stat::STATUSR::WRITE_ADDRESS_TRANSMITTED_NOT_ACK
            | lpc11xx::i2c::stat::STATUSR::DATA_BYTE_TRANSMITTED_NOT_ACK
            | lpc11xx::i2c::stat::STATUSR::ARBITRATION_LOST => {
                match status {
                    lpc11xx::i2c::stat::STATUSR::WRITE_ADDRESS_TRANSMITTED_NOT_ACK => {
                        ctx.set_error(Error::AddressNotAcknowledged(ctx.command_address()));
                    }
                    lpc11xx::i2c::stat::STATUSR::DATA_BYTE_TRANSMITTED_NOT_ACK => {
                        ctx.set_error(Error::DataByteNotAcknowledged);
                    }
                    lpc11xx::i2c::stat::STATUSR::ARBITRATION_LOST => {
                        ctx.set_error(Error::ArbitrationLost);
                    }
                    _ => unsafe { core::hint::unreachable_unchecked() },
                }

                i2c.conclr.write(|w| w.sta().clear().si().clear());
                i2c.conset.write(|w| w.sto().set());

                lpc11xx::SCB::set_pendsv();
            }
            _ => debug_assert!(false, "unexpected interrupt flag"),
        }
    }
}

impl core::fmt::Debug for Driver {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Driver {{ /* device I2C peripheral */ }}")
    }
}

/// A 7-bit slave address to use in I2C transactions.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct Address(u8);

impl Address {
    /// Returns the General Call address.
    #[inline]
    pub const fn general_call() -> Self {
        Self(0b0000_0000)
    }

    /// Converts a byte in the form `0bAAAAAAAX` into an I2C address.
    #[inline]
    pub const fn from_left_justified_byte(value: u8) -> Self {
        Self(value & 0b1111_1110)
    }

    /// Converts a byte in the form `0bXAAAAAAA` into an I2C address.
    #[inline]
    pub const fn from_right_justified_byte(value: u8) -> Self {
        Self(value << 1)
    }

    /// Returns whether this address represents the General Call address.
    ///
    /// This address may not be used for receiving data since it addresses
    /// multiple slaves simultaneously. Attempts to read using this address will
    /// result in NOT ACK errors.
    #[inline]
    pub const fn is_general_call(self) -> bool {
        self.0 == 0b0000_0000
    }

    /// Returns whether this address is in the I2C reserved address space.
    ///
    /// Reserved addresses may be used freely, but note that some devices on the
    /// bus may malfunction. Only use reserved addresses in controlled
    /// environments. The General Call address is not considered a reserved
    /// address by this method.
    #[inline]
    pub const fn is_reserved(self) -> bool {
        let masked = self.0 & 0b1111_0000;

        (self.0 != 0b0000_0000) & ((masked == 0b0000_0000) | (masked == 0b1111_0000))
    }

    /// Returns this address as a `0bAAAAAAA0` byte.
    #[inline]
    pub const fn as_left_justified_byte(self) -> u8 {
        self.0
    }

    /// Returns this address as a `0b0AAAAAAA` byte.
    #[inline]
    pub const fn as_right_justified_byte(self) -> u8 {
        self.0 >> 1
    }

    /// Returns the byte representing a read command for this address.
    #[inline]
    const fn as_read_byte(self) -> u8 {
        self.0 | 0b0000_0001
    }

    /// Returns the byte representing a write command for this address.
    #[inline]
    const fn as_write_byte(self) -> u8 {
        self.0
    }
}

impl<'a> Command<'a> {
    fn address(&self) -> Address {
        match self {
            Command::Read(address, _) => *address,
            Command::Write(address, _) => *address,
        }
    }

    fn address_byte(&self) -> u8 {
        match self {
            Command::Read(address, _) => address.as_read_byte(),
            Command::Write(address, _) => address.as_write_byte(),
        }
    }

    unsafe fn as_ptr_range(&self) -> (*mut u8, *mut u8) {
        match self {
            Command::Read(_, buf) => {
                let ptr = buf.as_ptr() as *mut u8;
                (ptr, ptr.add(buf.len()))
            }
            Command::Write(_, buf) => {
                let ptr = buf.as_ptr() as *mut u8;
                (ptr, ptr.add(buf.len()))
            }
        }
    }
}

struct TransactionFuture;

impl TransactionFuture {
    pub fn new(i2c: &I2CDevice) -> Self {
        i2c.conset.write(|w| w.i2en().enable());

        Self
    }
}

impl Drop for TransactionFuture {
    fn drop(&mut self) {
        let i2c = unsafe { &*I2CDevice::ptr() };
        i2c.conclr.write(|w| w.i2en().disable());
    }
}

impl Future for TransactionFuture {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, _: &mut Context) -> Poll<Self::Output> {
        let ctx = unsafe { &mut TXN_CTX };

        if ctx.command_ptr == ctx.command_end {
            Poll::Ready(ctx.error)
        } else {
            Poll::Pending
        }
    }
}

enum TransactionStatus {
    NextByte(u8),
    MoreCommands,
    LastCommand,
}

struct TransactionContext {
    command_ptr: *const Command<'static>,
    command_end: *const Command<'static>,

    buf_ptr: *mut u8,
    buf_end: *mut u8,

    error: Result<(), Error>,
}

impl TransactionContext {
    pub fn read_next_byte(&mut self) -> TransactionStatus {
        debug_assert!(self.command_ptr != self.command_end);

        if self.buf_ptr == self.buf_end {
            if let Some(command) = self.advance_to_next_command() {
                let (ptr, end) = unsafe { command.as_ptr_range() };

                self.buf_ptr = ptr;
                self.buf_end = end;

                TransactionStatus::MoreCommands
            } else {
                TransactionStatus::LastCommand
            }
        } else {
            let next_byte = unsafe { self.buf_ptr.read() };
            self.buf_ptr = unsafe { self.buf_ptr.add(1) };

            TransactionStatus::NextByte(next_byte)
        }
    }

    pub fn advance_to_next_command(&mut self) -> Option<&Command<'static>> {
        self.command_ptr = unsafe { self.command_ptr.add(1) };

        if self.command_ptr != self.command_end {
            Some(unsafe { &*self.command_ptr })
        } else {
            None
        }
    }

    pub fn is_command_complete(&self) -> bool {
        self.buf_ptr != self.buf_end
    }

    pub fn write_byte(&mut self, byte: u8) {
        unsafe {
            *self.buf_ptr = byte;
            self.buf_ptr = self.buf_ptr.add(1);
        }
    }

    pub fn command_address_byte(&self) -> u8 {
        unsafe { &*self.command_ptr }.address_byte()
    }

    pub fn command_address(&self) -> Address {
        unsafe { &*self.command_ptr }.address()
    }

    pub fn set_error(&mut self, error: Error) {
        self.command_ptr = self.command_end;
        self.error = Err(error);
    }
}

static mut TXN_CTX: TransactionContext = TransactionContext {
    command_ptr: core::ptr::null(),
    command_end: core::ptr::null(),
    buf_ptr: core::ptr::null_mut(),
    buf_end: core::ptr::null_mut(),
    error: Ok(()),
};
