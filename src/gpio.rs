//! # General Purpose I/Os
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOA peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split();
//! ```
//!
//! This gives you a struct containing all the pins `px0..px15`. These structs are what you use to
//! interract with the pins to change their modes, or their inputs or outputs.
//! For example, to set `pa5` high, you would call
//!
//! ```rust
//! let output = gpioa.pa5.into_push_pull_output();
//! output.set_high();
//! ```
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull up resistor. Will be high when nothing
//!     is connected
//!     - **PullDown**: Input connected to high with a weak pull up resistor. Will be low when nothing
//!     is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it do ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Debugger**: Some pins start out being used by the debugger. A pin in this mode can only be
//! used if the [JTAG peripheral has been turned off](#accessing-pa15-pb3-and-pb14).
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the `as_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode
//!
//! # Interfacing with v1 traits
//!
//! `embedded-hal` has two versions of the digital traits, `v2` which is used by this crate and
//! `v1` which is deprecated but still used by a lot of drivers.  If you want to use such a driver
//! with this crate, you need to convert the digital pins to the `v1` type.
//!
//! This is done using `embedded-hal::digital::v1_compat::OldOutputPin`. For example:
//!
//! ```rust
//! use mfrc522::Mfrc522;
//! let nss = gpioa.pa4.into_push_pull_output();
//! let mut mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();
//! ```

use core::marker::PhantomData;

use crate::pac::EXTI;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self) -> Self::Parts;
}

/// Marker trait for pin mode detection.
pub trait Mode<MODE> {}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Input<MODE> {}

/// Used by the debugger (type state)
pub struct Debugger;
/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Output<MODE> {}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;
impl Active for Analog {}

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Alternate<MODE> {}

pub enum State {
    High,
    Low,
}

/// GPIO Pin speed selection
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

/// Allow setting of the slew rate of an IO pin
///
/// Initially all pins are set to the maximum slew rate
pub trait OutputSpeed {
    fn set_speed(&mut self, speed: Speed);
}

#[derive(Debug, PartialEq)]
#[allow(non_camel_case_types)]
pub enum Edge {
    RISING,
    FALLING,
    RISING_FALLING,
}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, exti: &mut EXTI);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self, edge: Edge);
    fn check_interrupt(&mut self, edge: Edge) -> bool;
}

/// Tracks the current pin state for dynamic pins
pub enum Dynamic {
    InputFloating,
    InputPullUp,
    InputPullDown,
    OutputPushPull,
    OutputOpenDrain,
}

impl Active for Dynamic {}

#[derive(Debug, PartialEq)]
pub enum PinModeError {
    IncorrectMode,
}

impl Dynamic {
    fn is_input(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown | OutputOpenDrain => true,
            OutputPushPull => false,
        }
    }
    fn is_output(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown => false,
            OutputPushPull | OutputOpenDrain => true,
        }
    }
}

/// NOTE: This trait should ideally be private but must be pub in order to avoid
/// complaints from the compiler.
pub trait PinMode {
    unsafe fn set_mode() -> Self;
}

// These impls are needed because a macro can not brace initialise a ty token
impl<MODE> Input<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Output<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Alternate<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl Debugger {
    const fn _new() -> Self {
        Self {}
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $rcc_bit:expr, $PXx:ident, $extigpionr:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $exticri:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;
            use core::convert::Infallible;

            use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use crate::pac::$GPIOX;

            // use crate::{pac::{RCC, EXTI, SYSCFG}, bb};
            use crate::pac::{RCC, EXTI};
            use super::{
                Alternate, Floating, GpioExt, Input,
                OpenDrain,
                Output,
                PullDown,
                PullUp,
                PushPull,
                Analog,
                State,
                Active,
                Debugger,
                Pxx,
                Mode,
                Edge,
                ExtiPin,
                PinMode,
                Dynamic,
                PinModeError,
                OutputSpeed,
                Speed,
            };

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    unsafe {
                        // Enable the clock for the peripheral
                        // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
                        (*RCC::ptr()).mc_ahb4ensetr.write(|w| w.bits(1 << $rcc_bit));
                    }
                    Parts {
                        $(
                            $pxi: $PXi { mode: <$MODE>::_new() },
                        )+
                    }
                }
            }

            /// Partially erased pin. Only used in the Pxx enum
            pub struct Generic<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> Generic<MODE> {
                pub fn downgrade(self) -> Pxx<MODE> {
                    Pxx::$PXx(self)
                }
            }

            impl<MODE> OutputPin for Generic<Output<MODE>> {
                type Error = Infallible;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) })
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) })
                }
            }

            impl<MODE> InputPin for Generic<Input<MODE>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl<MODE> ExtiPin for Generic<Input<MODE>> {
                /// Make corresponding EXTI line sensitive to this pin
                fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                    let offset = 4 * (self.i % 4);
                    match self.i {
                        0..=3 => {
                            exti.exticr1.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                            })
                        },
                        4..=7 => {
                            exti.exticr2.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                            })
                        },
                        8..=11 => {
                            exti.exticr3.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                            })
                        },
                        12..=15 => {
                            exti.exticr4.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                            })
                        },
                        _ => unreachable!(),
                    }
                }

                /// Generate interrupt on rising edge, falling edge or both
                fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
                    match edge {
                        Edge::RISING => {
                            exti.rtsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() | (1 << self.i))
                            });
                            exti.ftsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() & !(1 << self.i))
                            });
                        },
                        Edge::FALLING => {
                            exti.ftsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() | (1 << self.i))
                            });
                            exti.rtsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() & !(1 << self.i))
                            });
                        },
                        Edge::RISING_FALLING => {
                            exti.rtsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() | (1 << self.i))
                            });
                            exti.ftsr1.modify(|r, w| unsafe {
                                w.bits(r.bits() | (1 << self.i))
                            });
                        }
                    }
                }

                /// Enable external interrupts from this pin.
                fn enable_interrupt(&mut self, exti: &mut EXTI) {
                    exti.c2imr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                }

                /// Disable external interrupts from this pin
                fn disable_interrupt(&mut self, exti: &mut EXTI) {
                    exti.c2imr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                }

                /// Clear the interrupt pending bit for this pin
                fn clear_interrupt_pending_bit(&mut self, edge: Edge) {
                    unsafe {
                        match edge {
                            Edge::RISING => {
                                 ((*EXTI::ptr()).rpr1.write(|w| w.bits(1 << self.i)));
                            }
                            Edge::FALLING => {
                                ((*EXTI::ptr()).fpr1.write(|w| w.bits(1 << self.i)));
                            }
                            Edge::RISING_FALLING => {
                                ((*EXTI::ptr()).rpr1.write(|w| w.bits(1 << self.i)));
                                ((*EXTI::ptr()).fpr1.write(|w| w.bits(1 << self.i)));
                            }
                        }
                    }
                }

                /// Reads the interrupt pending bit for this pin
                fn check_interrupt(&mut self, edge: Edge) -> bool {
                    let rising_int = unsafe { (*EXTI::ptr()).rpr1.read().bits() & (1 << self.i) };
                    let falling_int = unsafe { (*EXTI::ptr()).fpr1.read().bits() & (1 << self.i) };
                    match edge {
                        Edge::RISING => {
                            rising_int != 0
                        }
                        Edge::FALLING => {
                            falling_int != 0
                        }
                        Edge::RISING_FALLING => {
                            rising_int != 0 || falling_int != 0
                        }
                    }
                }
            }

            impl <MODE> StatefulOutputPin for Generic<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl <MODE> toggleable::Default for Generic<Output<MODE>> {}

            impl InputPin for Generic<Output<OpenDrain>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            pub type $PXx<MODE> = Pxx<MODE>;


            impl<MODE> Mode<MODE> for Generic<MODE> {}

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    mode: MODE,
                }

                impl<MODE> Mode<MODE> for $PXi<MODE> {}

                impl $PXi<Debugger> {
                    /// Put the pin in an active state. The caller
                    /// must enforce that the pin is really in this
                    /// state in the hardware.
                    #[allow(dead_code)]
                    pub(crate) unsafe fn activate(self) -> $PXi<Input<Floating>> {
                        $PXi { mode: Input::_new() }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Configures the pin to operate as an alternate function push-pull output
                    /// pin.
                    #[inline]
                    pub fn into_alternate_push_pull(self) -> $PXi<Alternate<PushPull>> {
                        let offset = 2 * $i;

                        unsafe {
                            // Set no pull-up/pull-down
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            // Set push-pull
                            (*$GPIOX::ptr()).otyper.modify(|r, w| {
                                w.bits(r.bits() & !(0b1 << $i))
                            });
                            // Set alternate function mode
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                            })
                        };

                        $PXi { mode: Alternate::_new() }
                    }

                    /// Configures the pin to operate as an alternate function open-drain output
                    /// pin.
                    #[inline]
                    pub fn into_alternate_open_drain(self) -> $PXi<Alternate<OpenDrain>> {
                        let offset = 2 * $i;

                        unsafe {
                            // Set no pull-up/pull-down
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            // Set open-drain
                            (*$GPIOX::ptr()).otyper.modify(|r, w| {
                                w.bits(r.bits() | (0b1 << $i))
                            });
                            // Set alternate function mode
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                            })
                        };

                        $PXi { mode: Alternate::_new() }
                    }

                    /// Configures the pin to operate as a floating input pin
                    #[inline]
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        unsafe {
                            $PXi::<Input<Floating>>::set_mode()
                        }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    #[inline]
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        unsafe {
                            $PXi::<Input<PullDown>>::set_mode()
                        }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    #[inline]
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        unsafe {
                            $PXi::<Input<PullUp>>::set_mode()
                        }
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// Initial state will be low.
                    #[inline]
                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output_with_state(State::Low)
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    #[inline]
                    pub fn into_open_drain_output_with_state(
                        mut self,
                        initial_state: State,
                    ) -> $PXi<Output<OpenDrain>> {
                        self.set_state(initial_state);
                        unsafe {
                            $PXi::<Output<OpenDrain>>::set_mode()
                        }
                    }
                    /// Configures the pin to operate as an push-pull output pin.
                    /// Initial state will be low.
                    #[inline]
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output_with_state(State::Low)
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    #[inline]
                    pub fn into_push_pull_output_with_state(
                        mut self,
                        initial_state: State,
                    ) -> $PXi<Output<PushPull>> {
                        self.set_state(initial_state);
                        unsafe {
                            $PXi::<Output<PushPull>>::set_mode()
                        }
                    }

                    /// Configures the pin to operate as an analog input pin
                    #[inline]
                    pub fn into_analog(self) -> $PXi<Analog> {
                        unsafe {
                            $PXi::<Analog>::set_mode()
                        }
                    }

                    /// Configures the pin as a pin that can change between input
                    /// and output without changing the type. It starts out
                    /// as a floating input
                    #[inline]
                    pub fn into_dynamic(self) -> $PXi<Dynamic> {
                        self.into_floating_input();
                        $PXi::<Dynamic>{mode: Dynamic::InputFloating}
                    }
                }

                // These macros are defined here instead of at the top level in order
                // to be able to refer to macro variables from the outer layers.
                macro_rules! impl_temp_output {
                    (
                        $fn_name:ident,
                        $stateful_fn_name:ident,
                        $mode:ty
                    ) => {
                        /**
                          Temporarily change the mode of the pin.

                          The value of the pin after conversion is undefined. If you
                          want to control it, use `$stateful_fn_name`
                        */
                        #[inline]
                        pub fn $fn_name(
                            &mut self,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            let mut temp = unsafe { $PXi::<$mode>::set_mode() };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode();
                            }
                        }

                        /**
                          Temporarily change the mode of the pin.

                          Note that the new state is set slightly before conversion
                          happens. This can cause a short output glitch if switching
                          between output modes
                        */
                        #[inline]
                        pub fn $stateful_fn_name(
                            &mut self,
                            state: State,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            self.set_state(state);
                            let mut temp = unsafe { $PXi::<$mode>::set_mode() };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode();
                            }
                        }
                    }
                }
                macro_rules! impl_temp_input {
                    (
                        $fn_name:ident,
                        $mode:ty
                    ) => {
                        /**
                          Temporarily change the mode of the pin.
                        */
                        #[inline]
                        pub fn $fn_name(
                            &mut self,
                            mut f: impl FnMut(&mut $PXi<$mode>)
                        ) {
                            let mut temp = unsafe { $PXi::<$mode>::set_mode() };
                            f(&mut temp);
                            unsafe {
                                Self::set_mode();
                            }
                        }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active, $PXi<MODE>: PinMode {
                    impl_temp_output!(
                        as_push_pull_output,
                        as_push_pull_output_with_state,
                        Output<PushPull>
                    );
                    impl_temp_output!(
                        as_open_drain_output,
                        as_open_drain_output_with_state,
                        Output<OpenDrain>
                    );
                    impl_temp_input!(
                        as_floating_input,
                        Input<Floating>
                    );
                    impl_temp_input!(
                        as_pull_up_input,
                        Input<PullUp>
                    );
                    impl_temp_input!(
                        as_pull_down_input,
                        Input<PullDown>
                    );
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Erases the pin number from the type
                    #[inline]
                    fn into_generic(self) -> Generic<MODE> {
                        Generic {
                            i: $i,
                            _mode: PhantomData,
                        }
                    }

                    /// Erases the pin number and port from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pxx<MODE> {
                        self.into_generic().downgrade()
                    }
                }

                // embedded_hal impls

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Infallible;
                    #[inline]
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(self.set_state(State::High))
                    }

                    #[inline]
                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(self.set_state(State::Low))
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    #[inline]
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Ok(self._is_set_low())
                    }
                }

                impl<MODE> OutputSpeed for $PXi<Output<MODE>> {
                    fn set_speed(&mut self, speed: Speed) {
                        let offset = $i * 2;

                        unsafe {
                            (*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                            });
                        }
                    }
                }

                impl OutputSpeed for $PXi<Alternate<PushPull>> {
                    fn set_speed(&mut self, speed: Speed){
                        let offset = $i * 2;

                        unsafe {
                            (*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                            });
                        }
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Infallible;
                    #[inline]
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(self._is_low())
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Infallible;
                    #[inline]
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    #[inline]
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Ok(self._is_low())
                    }
                }


                // Dynamic pin

                impl $PXi<Dynamic> {
                    #[inline]
                    pub fn make_pull_up_input(&mut self) {
                        // NOTE(unsafe), we have a mutable reference to the current pin
                        unsafe { $PXi::<Input<PullUp>>::set_mode() };
                        self.mode = Dynamic::InputPullUp;
                    }
                    #[inline]
                    pub fn make_pull_down_input(&mut self) {
                        // NOTE(unsafe), we have a mutable reference to the current pin
                        unsafe { $PXi::<Input<PullDown>>::set_mode() };
                        self.mode = Dynamic::InputPullDown;
                    }
                    #[inline]
                    pub fn make_floating_input(&mut self) {
                        // NOTE(unsafe), we have a mutable reference to the current pin
                        unsafe { $PXi::<Input<Floating>>::set_mode() };
                        self.mode = Dynamic::InputFloating;
                    }
                    #[inline]
                    pub fn make_push_pull_output(&mut self) {
                        // NOTE(unsafe), we have a mutable reference to the current pin
                        unsafe { $PXi::<Output<PushPull>>::set_mode() };
                        self.mode = Dynamic::OutputPushPull;
                    }
                    #[inline]
                    pub fn make_open_drain_output(&mut self) {
                        // NOTE(unsafe), we have a mutable reference to the current pin
                        unsafe { $PXi::<Output<OpenDrain>>::set_mode() };
                        self.mode = Dynamic::OutputOpenDrain;
                    }
                }

                impl OutputPin for $PXi<Dynamic> {
                    type Error = PinModeError;
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        if self.mode.is_output() {
                            self.set_state(State::High);
                            Ok(())
                        }
                        else {
                            Err(PinModeError::IncorrectMode)
                        }
                    }
                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        if self.mode.is_output() {
                            self.set_state(State::Low);
                            Ok(())
                        }
                        else {
                            Err(PinModeError::IncorrectMode)
                        }
                    }
                }

                impl InputPin for $PXi<Dynamic> {
                    type Error = PinModeError;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        if self.mode.is_input() {
                            Ok(self._is_low())
                        }
                        else {
                            Err(PinModeError::IncorrectMode)
                        }
                    }
                }


                // Exti pin impls

                impl<MODE> ExtiPin for $PXi<Input<MODE>> {
                    /// Configure EXTI Line $i to trigger from this pin.
                    fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                        let offset = 4 * ($i % 4);
                        match $i {
                            0..=3 => {
                                exti.exticr1.modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                                })
                            },
                            4..=7 => {
                                exti.exticr2.modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                                })
                            },
                            8..=11 => {
                                exti.exticr3.modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                                })
                            },
                            12..=15 => {
                                exti.exticr4.modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0xff << offset)) | ($extigpionr << offset))
                                })
                            },
                            _ => unreachable!(),
                        }
                    }

                    /// Generate interrupt on rising edge, falling edge or both
                    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
                        match edge {
                            Edge::RISING => {
                                exti.rtsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() | (1 << $i))
                                });
                                exti.ftsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() & !(1 << $i))
                                });
                            },
                            Edge::FALLING => {
                                exti.ftsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() | (1 << $i))
                                });
                                exti.rtsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() & !(1 << $i))
                                });
                            },
                            Edge::RISING_FALLING => {
                                exti.rtsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() | (1 << $i))
                                });
                                exti.ftsr1.modify(|r, w| unsafe {
                                    w.bits(r.bits() | (1 << $i))
                                });
                            }
                        }
                    }

                    /// Enable external interrupts from this pin.
                    #[inline]
                    fn enable_interrupt(&mut self, exti: &mut EXTI) {
                        exti.c2imr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                    }

                    /// Disable external interrupts from this pin
                    #[inline]
                    fn disable_interrupt(&mut self, exti: &mut EXTI) {
                        exti.c2imr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    }

                    /// Clear the interrupt pending bit for this pin
                    #[inline]
                    fn clear_interrupt_pending_bit(&mut self, edge: Edge) {
                        unsafe {
                            match edge {
                                Edge::RISING => {
                                     ((*EXTI::ptr()).rpr1.write(|w| w.bits(1 << $i)));
                                }
                                Edge::FALLING => {
                                    ((*EXTI::ptr()).fpr1.write(|w| w.bits(1 << $i)));
                                }
                                Edge::RISING_FALLING => {
                                    ((*EXTI::ptr()).rpr1.write(|w| w.bits(1 << $i)));
                                    ((*EXTI::ptr()).fpr1.write(|w| w.bits(1 << $i)));
                                }
                            }
                        }
                    }

                    /// Reads the interrupt pending bit for this pin
                    #[inline]
                    fn check_interrupt(&mut self, edge: Edge) -> bool {
                        match edge {
                            Edge::RISING => {
                                let rising_int = unsafe {
                                    (*EXTI::ptr()).rpr1.read().bits() & (1 << $i)
                                };
                                rising_int != 0
                            }
                            Edge::FALLING => {
                                let falling_int = unsafe {
                                    (*EXTI::ptr()).fpr1.read().bits() & (1 << $i)
                                };
                                falling_int != 0
                            }
                            Edge::RISING_FALLING => {
                                let rising_int = unsafe {
                                    (*EXTI::ptr()).rpr1.read().bits() & (1 << $i)
                                };
                                let falling_int = unsafe {
                                    (*EXTI::ptr()).fpr1.read().bits() & (1 << $i)
                                };
                                rising_int != 0 || falling_int != 0
                            }
                        }
                    }
                }


                // Internal helper functions

                // NOTE: The functions in this impl block are "safe", but they
                // are callable when the pin is in modes where they don't make
                // sense.
                impl<MODE> $PXi<MODE> {
                    /**
                      Set the output of the pin regardless of its mode.
                      Primarily used to set the output value of the pin
                      before changing its mode to an output to avoid
                      a short spike of an incorrect value
                    */
                    fn set_state(&mut self, state: State) {
                        match state {
                            State::High => unsafe {
                                (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i))
                            }
                            State::Low => unsafe {
                                (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i)))
                            }
                        }
                    }

                    fn _is_set_low(&self) -> bool {
                        unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 }
                    }

                    fn _is_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 }
                    }
                }

                impl PinMode for $PXi<Input<Floating>> {
                    unsafe fn set_mode() -> Self {
                        let offset = 2 * $i;

                        // Set to floating
                        (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)))
                        });
                        // Set to input
                        (*$GPIOX::ptr()).moder.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)))
                        });

                        $PXi { mode: Input::_new() }
                    }
                }

                impl PinMode for $PXi<Input<PullDown>> {
                    unsafe fn set_mode() -> Self {
                        let offset = $i * 2;

                        // Set to input
                        (*$GPIOX::ptr()).moder.modify(|r, w|
                            w.bits(r.bits() & !(0b11 << offset))
                        );
                        // Set to pull-down
                        (*$GPIOX::ptr()).pupdr.modify(|r, w|
                            w.bits(r.bits() & !(0b11 << offset) | (0b10 << offset))
                        );

                        $PXi { mode: Input::_new() }
                    }
                }

                impl PinMode for $PXi<Input<PullUp>> {
                    unsafe fn set_mode() -> Self {
                        let offset = $i * 2;

                        // Set to input
                        (*$GPIOX::ptr()).moder.modify(|r, w|
                            w.bits(r.bits() & !(0b11 << offset))
                        );
                        // Set to pull-up
                        (*$GPIOX::ptr()).pupdr.modify(|r, w|
                            w.bits(r.bits() & !(0b11 << offset) | (0b01 << offset))
                        );

                        $PXi { mode: Input::_new() }
                    }
                }

                impl PinMode for $PXi<Output<OpenDrain>> {
                    unsafe fn set_mode() -> Self {
                        let offset = $i * 2;

                        // Set no pull-up/pull-down
                        (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                        });
                        // Set open-drain
                        (*$GPIOX::ptr()).otyper.modify(|r, w| {
                            w.bits(r.bits() | (0b1 << $i))
                        });
                        // Set output mode
                        (*$GPIOX::ptr()).moder.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                        });

                        $PXi { mode: Output::_new() }
                    }
                }

                impl PinMode for $PXi<Output<PushPull>> {
                    unsafe fn set_mode() -> Self {
                        let offset = $i * 2;

                        // Set no pull-up/pull-down
                        (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                        });
                        // Set push-pull
                        (*$GPIOX::ptr()).otyper.modify(|r, w| {
                            w.bits(r.bits() & !(0b1 << $i))
                        });
                        // Set output mode
                        (*$GPIOX::ptr()).moder.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                        });

                        $PXi { mode: Output::_new() }
                    }
                }

                impl PinMode for $PXi<Analog> {
                    unsafe fn set_mode() -> Self {
                        let offset = $i * 2;

                        // Set no pull-up/pull-down
                        (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                        });
                        // Set output mode
                        (*$GPIOX::ptr()).moder.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset))
                        });

                        $PXi { mode: Analog{} }
                    }
                }
            )+
        }
    }
}

macro_rules! impl_pxx {
    ($(($port:ident :: $pin:ident)),*) => {
        use embedded_hal::digital::v2::{InputPin, StatefulOutputPin, OutputPin};
        use core::convert::Infallible;

        pub enum Pxx<MODE> {
            $(
                $pin($port::Generic<MODE>)
            ),*
        }

        impl<MODE> OutputPin for Pxx<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_high()),*
                }
            }

            fn set_low(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_low()),*
                }
            }
        }

        impl<MODE> StatefulOutputPin for Pxx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_high()),*
                }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> InputPin for Pxx<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl InputPin for Pxx<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl<MODE> ExtiPin for Pxx<Input<MODE>> {
            fn make_interrupt_source(&mut self, exti: &mut EXTI) {
                match self {
                    $(Pxx::$pin(pin) => pin.make_interrupt_source(exti)),*
                }
            }

            fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge) {
                match self {
                    $(Pxx::$pin(pin) => pin.trigger_on_edge(exti, level)),*
                }
            }

            fn enable_interrupt(&mut self, exti: &mut EXTI) {
                match self {
                    $(Pxx::$pin(pin) => pin.enable_interrupt(exti)),*
                }
            }

            fn disable_interrupt(&mut self, exti: &mut EXTI) {
                match self {
                    $(Pxx::$pin(pin) => pin.disable_interrupt(exti)),*
                }
            }

            fn clear_interrupt_pending_bit(&mut self, edge: Edge) {
                match self {
                    $(Pxx::$pin(pin) => pin.clear_interrupt_pending_bit(edge)),*
                }
            }

            fn check_interrupt(&mut self, edge: Edge) -> bool {
                match self {
                    $(Pxx::$pin(pin) => pin.check_interrupt(edge)),*
                }
            }
        }
    }
}

impl_pxx! {
    (gpioa::PAx),
    (gpiob::PBx),
    (gpioc::PCx),
    (gpiod::PDx),
    (gpioe::PEx),
    (gpiof::PFx),
    (gpiog::PGx),
    (gpioh::PHx),
    (gpioi::PIx),
    (gpioj::PJx),
    (gpiok::PKx)
}

gpio!(GPIOA, gpioa, 0, PAx, 0, [
    PA0: (pa0, 0, Input<Floating>, exticr1),
    PA1: (pa1, 1, Input<Floating>, exticr1),
    PA2: (pa2, 2, Input<Floating>, exticr1),
    PA3: (pa3, 3, Input<Floating>, exticr1),
    PA4: (pa4, 4, Input<Floating>, exticr2),
    PA5: (pa5, 5, Input<Floating>, exticr2),
    PA6: (pa6, 6, Input<Floating>, exticr2),
    PA7: (pa7, 7, Input<Floating>, exticr2),
    PA8: (pa8, 8, Input<Floating>, exticr3),
    PA9: (pa9, 9, Input<Floating>, exticr3),
    PA10: (pa10, 10, Input<Floating>, exticr3),
    PA11: (pa11, 11, Input<Floating>, exticr3),
    PA12: (pa12, 12, Input<Floating>, exticr4),
    PA13: (pa13, 13, Input<Floating>, exticr4),
    PA14: (pa14, 14, Input<Floating>, exticr4),
    PA15: (pa15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOB, gpiob, 1, PBx, 1, [
    PB0: (pb0, 0, Input<Floating>, exticr1),
    PB1: (pb1, 1, Input<Floating>, exticr1),
    PB2: (pb2, 2, Input<Floating>, exticr1),
    PB3: (pb3, 3, Input<Floating>, exticr1),
    PB4: (pb4, 4, Input<Floating>, exticr2),
    PB5: (pb5, 5, Input<Floating>, exticr2),
    PB6: (pb6, 6, Input<Floating>, exticr2),
    PB7: (pb7, 7, Input<Floating>, exticr2),
    PB8: (pb8, 8, Input<Floating>, exticr3),
    PB9: (pb9, 9, Input<Floating>, exticr3),
    PB10: (pb10, 10, Input<Floating>, exticr3),
    PB11: (pb11, 11, Input<Floating>, exticr3),
    PB12: (pb12, 12, Input<Floating>, exticr4),
    PB13: (pb13, 13, Input<Floating>, exticr4),
    PB14: (pb14, 14, Input<Floating>, exticr4),
    PB15: (pb15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOC, gpioc, 2, PCx, 2, [
    PC0: (pc0, 0, Input<Floating>, exticr1),
    PC1: (pc1, 1, Input<Floating>, exticr1),
    PC2: (pc2, 2, Input<Floating>, exticr1),
    PC3: (pc3, 3, Input<Floating>, exticr1),
    PC4: (pc4, 4, Input<Floating>, exticr2),
    PC5: (pc5, 5, Input<Floating>, exticr2),
    PC6: (pc6, 6, Input<Floating>, exticr2),
    PC7: (pc7, 7, Input<Floating>, exticr2),
    PC8: (pc8, 8, Input<Floating>, exticr3),
    PC9: (pc9, 9, Input<Floating>, exticr3),
    PC10: (pc10, 10, Input<Floating>, exticr3),
    PC11: (pc11, 11, Input<Floating>, exticr3),
    PC12: (pc12, 12, Input<Floating>, exticr4),
    PC13: (pc13, 13, Input<Floating>, exticr4),
    PC14: (pc14, 14, Input<Floating>, exticr4),
    PC15: (pc15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOD, gpiod, 3, PDx, 3, [
    PD0: (pd0, 0, Input<Floating>, exticr1),
    PD1: (pd1, 1, Input<Floating>, exticr1),
    PD2: (pd2, 2, Input<Floating>, exticr1),
    PD3: (pd3, 3, Input<Floating>, exticr1),
    PD4: (pd4, 4, Input<Floating>, exticr2),
    PD5: (pd5, 5, Input<Floating>, exticr2),
    PD6: (pd6, 6, Input<Floating>, exticr2),
    PD7: (pd7, 7, Input<Floating>, exticr2),
    PD8: (pd8, 8, Input<Floating>, exticr3),
    PD9: (pd9, 9, Input<Floating>, exticr3),
    PD10: (pd10, 10, Input<Floating>, exticr3),
    PD11: (pd11, 11, Input<Floating>, exticr3),
    PD12: (pd12, 12, Input<Floating>, exticr4),
    PD13: (pd13, 13, Input<Floating>, exticr4),
    PD14: (pd14, 14, Input<Floating>, exticr4),
    PD15: (pd15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOE, gpioe, 4, PEx, 4, [
    PE0: (pe0, 0, Input<Floating>, exticr1),
    PE1: (pe1, 1, Input<Floating>, exticr1),
    PE2: (pe2, 2, Input<Floating>, exticr1),
    PE3: (pe3, 3, Input<Floating>, exticr1),
    PE4: (pe4, 4, Input<Floating>, exticr2),
    PE5: (pe5, 5, Input<Floating>, exticr2),
    PE6: (pe6, 6, Input<Floating>, exticr2),
    PE7: (pe7, 7, Input<Floating>, exticr2),
    PE8: (pe8, 8, Input<Floating>, exticr3),
    PE9: (pe9, 9, Input<Floating>, exticr3),
    PE10: (pe10, 10, Input<Floating>, exticr3),
    PE11: (pe11, 11, Input<Floating>, exticr3),
    PE12: (pe12, 12, Input<Floating>, exticr4),
    PE13: (pe13, 13, Input<Floating>, exticr4),
    PE14: (pe14, 14, Input<Floating>, exticr4),
    PE15: (pe15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOF, gpiof, 5, PFx, 5, [
    PF0: (pf0, 0, Input<Floating>, exticr1),
    PF1: (pf1, 1, Input<Floating>, exticr1),
    PF2: (pf2, 2, Input<Floating>, exticr1),
    PF3: (pf3, 3, Input<Floating>, exticr1),
    PF4: (pf4, 4, Input<Floating>, exticr2),
    PF5: (pf5, 5, Input<Floating>, exticr2),
    PF6: (pf6, 6, Input<Floating>, exticr2),
    PF7: (pf7, 7, Input<Floating>, exticr2),
    PF8: (pf8, 8, Input<Floating>, exticr3),
    PF9: (pf9, 9, Input<Floating>, exticr3),
    PF10: (pf10, 10, Input<Floating>, exticr3),
    PF11: (pf11, 11, Input<Floating>, exticr3),
    PF12: (pf12, 12, Input<Floating>, exticr4),
    PF13: (pf13, 13, Input<Floating>, exticr4),
    PF14: (pf14, 14, Input<Floating>, exticr4),
    PF15: (pf15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOG, gpiog, 6, PGx, 6, [
    PG0: (pg0, 0, Input<Floating>, exticr1),
    PG1: (pg1, 1, Input<Floating>, exticr1),
    PG2: (pg2, 2, Input<Floating>, exticr1),
    PG3: (pg3, 3, Input<Floating>, exticr1),
    PG4: (pg4, 4, Input<Floating>, exticr2),
    PG5: (pg5, 5, Input<Floating>, exticr2),
    PG6: (pg6, 6, Input<Floating>, exticr2),
    PG7: (pg7, 7, Input<Floating>, exticr2),
    PG8: (pg8, 8, Input<Floating>, exticr3),
    PG9: (pg9, 9, Input<Floating>, exticr3),
    PG10: (pg10, 10, Input<Floating>, exticr3),
    PG11: (pg11, 11, Input<Floating>, exticr3),
    PG12: (pg12, 12, Input<Floating>, exticr4),
    PG13: (pg13, 13, Input<Floating>, exticr4),
    PG14: (pg14, 14, Input<Floating>, exticr4),
    PG15: (pg15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOH, gpioh, 7, PHx, 7, [
    PH0: (ph0, 0, Input<Floating>, exticr1),
    PH1: (ph1, 1, Input<Floating>, exticr1),
    PH2: (ph2, 2, Input<Floating>, exticr1),
    PH3: (ph3, 3, Input<Floating>, exticr1),
    PH4: (ph4, 4, Input<Floating>, exticr2),
    PH5: (ph5, 5, Input<Floating>, exticr2),
    PH6: (ph6, 6, Input<Floating>, exticr2),
    PH7: (ph7, 7, Input<Floating>, exticr2),
    PH8: (ph8, 8, Input<Floating>, exticr3),
    PH9: (ph9, 9, Input<Floating>, exticr3),
    PH10: (ph10, 10, Input<Floating>, exticr3),
    PH11: (ph11, 11, Input<Floating>, exticr3),
    PH12: (ph12, 12, Input<Floating>, exticr4),
    PH13: (ph13, 13, Input<Floating>, exticr4),
    PH14: (ph14, 14, Input<Floating>, exticr4),
    PH15: (ph15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOI, gpioi, 8, PIx, 8, [
    PI0: (pi0, 0, Input<Floating>, exticr1),
    PI1: (pi1, 1, Input<Floating>, exticr1),
    PI2: (pi2, 2, Input<Floating>, exticr1),
    PI3: (pi3, 3, Input<Floating>, exticr1),
    PI4: (pi4, 4, Input<Floating>, exticr2),
    PI5: (pi5, 5, Input<Floating>, exticr2),
    PI6: (pi6, 6, Input<Floating>, exticr2),
    PI7: (pi7, 7, Input<Floating>, exticr2),
    PI8: (pi8, 8, Input<Floating>, exticr3),
    PI9: (pi9, 9, Input<Floating>, exticr3),
    PI10: (pi10, 10, Input<Floating>, exticr3),
    PI11: (pi11, 11, Input<Floating>, exticr3),
    PI12: (pi12, 12, Input<Floating>, exticr4),
    PI13: (pi13, 13, Input<Floating>, exticr4),
    PI14: (pi14, 14, Input<Floating>, exticr4),
    PI15: (pi15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOJ, gpioj, 9, PJx, 9, [
    PJ0: (pj0, 0, Input<Floating>, exticr1),
    PJ1: (pj1, 1, Input<Floating>, exticr1),
    PJ2: (pj2, 2, Input<Floating>, exticr1),
    PJ3: (pj3, 3, Input<Floating>, exticr1),
    PJ4: (pj4, 4, Input<Floating>, exticr2),
    PJ5: (pj5, 5, Input<Floating>, exticr2),
    PJ6: (pj6, 6, Input<Floating>, exticr2),
    PJ7: (pj7, 7, Input<Floating>, exticr2),
    PJ8: (pj8, 8, Input<Floating>, exticr3),
    PJ9: (pj9, 9, Input<Floating>, exticr3),
    PJ10: (pj10, 10, Input<Floating>, exticr3),
    PJ11: (pj11, 11, Input<Floating>, exticr3),
    PJ12: (pj12, 12, Input<Floating>, exticr4),
    PJ13: (pj13, 13, Input<Floating>, exticr4),
    PJ14: (pj14, 14, Input<Floating>, exticr4),
    PJ15: (pj15, 15, Input<Floating>, exticr4),
]);

gpio!(GPIOK, gpiok, 10, PKx, 10, [
    PK0: (pk0, 0, Input<Floating>, exticr1),
    PK1: (pk1, 1, Input<Floating>, exticr1),
    PK2: (pk2, 2, Input<Floating>, exticr1),
    PK3: (pk3, 3, Input<Floating>, exticr1),
    PK4: (pk4, 4, Input<Floating>, exticr2),
    PK5: (pk5, 5, Input<Floating>, exticr2),
    PK6: (pk6, 6, Input<Floating>, exticr2),
    PK7: (pk7, 7, Input<Floating>, exticr2),
]);
