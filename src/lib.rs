//! # HAL for the STM32MP1 family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32MP1 family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//!
//! # Usage
//!
//! ## Building an application (binary crate)
//!
//! A detailed usage guide can be found in the [README]
//!
//! supported microcontrollers are:
//!
//! - stm32mp157
//!
//! ## Usage
//!
//! This crate supports multiple microcontrollers in the
//! stm32mp1 family. Which specific microcontroller you want to build for has to be
//! specified with a feature, for example `stm32mp157`.
//!
//! If no microcontroller is specified, the crate will not compile.
//!
//! The currently supported variants are
//!
//! - `stm32mp157`
//!
//! ## Commonly used setup
//! Almost all peripherals require references to some registers in `RCC` and `AFIO`. The following
//! code shows how to set up those registers
//!
//! ```rust
//! // Get access to the device specific peripherals from the peripheral access crate
//! let dp = pac::Peripherals::take().unwrap();
//!
//! // Take ownership over the raw flash and rcc devices and convert them into the corresponding
//! // HAL structs
//! let mut flash = dp.FLASH.constrain();
//! let mut rcc = dp.RCC.constrain();
//!
//! // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
//! // `clocks`
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//!
//! // Prepare the alternate function I/O registers
//! let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
//! ```
//!
//! ## Usage examples
//!
//! See the [examples] folder.
//!
//! Most of the examples require the following additional dependencies
//! ```toml
//! [dependencies]
//! embedded-hal = "0.2.3"
//! nb = "0.1.2"
//! cortex-m = "0.6.2"
//! cortex-m-rt = "0.6.11"
//! # Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
//! panic-halt = "0.2.0"
//! ```
//!
//! [examples]: https://github.com/stm32-rs/stm32mp1xx-hal/tree/v0.7.0/examples
//! [README]: https://github.com/stm32-rs/stm32mp1xx-hal/tree/v0.7.0

#![no_std]

// If no target specified, print error message.
#[cfg(not(any(
    feature = "stm32mp157",
)))]
compile_error!("Target not found. A `--features <target-name>` is required.");

#[cfg(feature = "device-selected")]
use embedded_hal as hal;

#[cfg(feature = "stm32mp157")]
pub use stm32mp1::stm32mp157 as pac;

#[cfg(feature = "device-selected")]
#[deprecated(since = "0.6.0", note = "please use `pac` instead")]
#[doc(hidden)]
pub use crate::pac as device;

#[cfg(feature = "device-selected")]
#[deprecated(since = "0.6.0", note = "please use `pac` instead")]
#[doc(hidden)]
pub use crate::pac as stm32;

// #[cfg(feature = "device-selected")]
// pub mod adc;
// #[cfg(feature = "device-selected")]
// pub mod bb;
// #[cfg(feature = "device-selected")]
// pub mod crc;
// #[cfg(feature = "device-selected")]
// pub mod delay;
// #[cfg(feature = "device-selected")]
// pub mod dma;
// #[cfg(feature = "device-selected")]
// pub mod flash;
#[cfg(feature = "device-selected")]
pub mod gpio;
// #[cfg(feature = "device-selected")]
// pub mod i2c;
// #[cfg(feature = "device-selected")]
// pub mod ipcc;
#[cfg(feature = "device-selected")]
pub mod prelude;
// #[cfg(feature = "device-selected")]
// pub mod pwm;
// #[cfg(feature = "device-selected")]
// pub mod pwm_input;
// #[cfg(feature = "device-selected")]
// pub mod qei;
// #[cfg(feature = "device-selected")]
// pub mod rcc;
// #[cfg(feature = "device-selected")]
// pub mod rtc;
// #[cfg(feature = "device-selected")]
// pub mod serial;
// #[cfg(feature = "device-selected")]
// pub mod spi;
// #[cfg(feature = "device-selected")]
// pub mod time;
// #[cfg(feature = "device-selected")]
// pub mod timer;
// #[cfg(all(
//     feature = "stm32-usbd",
//     any(feature = "stm32f102", feature = "stm32f103")
// ))]
// pub mod usb;
// #[cfg(feature = "device-selected")]
// pub mod watchdog;
