# LPC11xx Async HAL

[![Documentation](https://docs.rs/lpc11xx-async-hal/badge.svg)](https://docs.rs/lpc11xx-async-hal)
[![Crates.io](https://img.shields.io/crates/v/lpc11xx-async-hal.svg)](https://crates.io/crates/lpc11xx-async-hal)

Asynchronous HAL for the NXP LPC111x/LPC11Cxx/LPC11xxL/LPC11xxXL family of Cortex-M0 microcontrollers using async-await syntax.

## Example

Below is a minimal echo server using the UART driver. The microcontroller will wake up from sleep only to send/receive data in the UART interrupt and to issue additional UART commands from the `async fn` inside `PendSV` and will be sleeping the rest of the time.

```rust
use lpc11xx_async_hal::uart;

// Initialize the UART driver ahead of time, e.g. 9600 baud at 12MHz
// The UART peripheral must have power, a clock, configured pins etc

let uart = uart::Driver::initialize(
    device.UART, // < lpc11xx::UART
    uart::Options {
        clock_divisor: 71,
        fractional_div: 0,
        fractional_mul: 1,
        flow_control: false,
        rx_threshold: uart::RxThreshold::default(),
    },
);

// The actual logic, to be polled from the PendSV interrupt handler

async fn echo_server(mut uart: uart::Driver) -> ! {
    let (mut sender, mut receiver) = uart.split();

    sender.write(b"Type some text: ").await;

    loop {
        let mut buffer = [0; 20];

        let bytes_read = receiver.read(&mut buffer).await;

        sender.write(&buffer[..bytes_read]).await;
    }
}
```

## Status

Async drivers for the following peripherals are currently available:

| **Peripheral**     | **Missing features**         | **Won't implement**    |
|:------------------:|:----------------------------:|:----------------------:|
| UART               |                              | Autobaud, RS-485       |
| I2C                | Slave Mode                   | Monitor Mode           |
| ADC                | Timer Match                  |                        |

More are in development, contributions and suggestions welcome.

## Caveats

The Rust async-await feature is still under heavy development to say nothing of its `#![no_std]` integration so this crate should be viewed as a proof of concept and ongoing experiment in writing automatically generated asynchronous state machines on LPC11xx devices. It _works_ fine and does useful things though.

### No-std support

An additional problem is brought about when using async-await in `#![no_std]` crates; essentially, it doesn't work because the compiler machinery to generate the resulting futures depends on thread-local storage which is in `std`. You can work around this by using the `core-futures-stateless` helper crate which will patch libcore to make `#![feature(async_await)]` work:

```toml
[dependencies.core]
package = "core-futures-stateless"
version = "0.1.0"
```

This can be removed once fixed in libcore. Note that this will (unfortunately) change the crate name of all types in the `core` crate in compiler messages.

### Futures behaviour

This crate is built exclusively around the `PendSV` facility in Cortex-M processors. That is, all futures depend on being polled inside the `PendSV` exception handler, and all peripheral interrupt handlers will raise `PendSV` to progress the futures. As a result the `task::Context` associated with your main async task is irrelevant and you should create a dummy `Waker` that does absolutely nothing. The `core-futures-stateless` crate provides a convenience method `task::stateless_waker()` which does just that.

```rust
use core::future::Future;
use core::task::{stateless_waker, Context};

// your executor somewhere inside the PendSV exception handler

match Future::poll(your_pinned_future, &mut Context::from_waker(stateless_waker())) {
    Poll::Ready(x) => { /* future resolved */ },
    Poll::Pending => { /* go back to sleep */ },
}
```

In the future the crate might become context-aware but in the meantime this is the way it works for simplicity and because using `PendSV` is quite a natural approach to polling futures.

### Memory safety

There is currently a minor soundness hole in this crate in that interrupt handlers reference stack-like memory from currently running futures, which means that `Drop` impls for Future types must run to ensure memory safety. I am not sure how to fix this without significant compromises in performance or ergonomics and don't really consider this a major issue from a pragmatic standpoint. Leak futures at your own peril.
