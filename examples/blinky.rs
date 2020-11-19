#![feature(asm)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32mp1xx_hal as hal;
use pac::interrupt;

use hal::{prelude::*, gpio::*, pac};
use core::mem::MaybeUninit;
use heapless::Vec;
use heapless::consts::*;
use core::borrow::BorrowMut;
use usb_device::endpoint::EndpointType::Interrupt;

static mut LED_RED: MaybeUninit<crate::hal::gpio::Pxx<Output<PushPull>>> = MaybeUninit::uninit();
static mut BTN: MaybeUninit<crate::hal::gpio::Pxx<Input<PullUp>>> = MaybeUninit::uninit();

#[interrupt]
fn EXTI1() {
    let led_red = unsafe { &mut *LED_RED.as_mut_ptr() };
    led_red.set_high().unwrap();

    let btn = unsafe { &mut *BTN.as_mut_ptr() };
    if btn.check_interrupt(Edge::FALLING) {
        btn.clear_interrupt_pending_bit(Edge::FALLING);
    }
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    // let mut vec = Vec::<_, U3>::new();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpiog = dp.GPIOG.split();

    let mut led_red = unsafe { &mut *LED_RED.as_mut_ptr() };
    *led_red = gpiog.pg13.into_push_pull_output().downgrade();
    // led_red = gpiog.pg13.into_push_pull_output().downgrade();
    let mut led_green = gpiob.pb13.into_push_pull_output().downgrade();
    let mut led_blue = gpioa.pa3.into_push_pull_output().downgrade();

    // vec.push(led_red).unwrap_or(());
    // vec.push(&mut led_green).unwrap_or(());
    // vec.push(&mut led_blue).unwrap_or(());

    // let btn = unsafe { &mut *BTN.as_mut_ptr() };
    let mut btn = gpiog.pg1.into_floating_input();
    btn.make_interrupt_source(&mut dp.EXTI);
    btn.trigger_on_edge(&mut dp.EXTI, Edge::RISING_FALLING);
    btn.enable_interrupt(&mut dp.EXTI);

    unsafe {
        dp.RCC.mc_ahb3ensetr
            .write(|w|
                w
                    .hsemen().set_bit()
            );
        cp.NVIC.set_priority(pac::Interrupt::EXTI1, 6);
        pac::NVIC::unmask(pac::Interrupt::EXTI1);
    };

    // Set all LEDs low
    // for led in &mut vec {
    //     led.set_low().unwrap();
    // }

    fn wait() {
        for _ in 0..48_000 {
            unsafe { asm!("nop;"); }
        }
    }

    loop {
        if btn.is_low().unwrap() {
            led_green.set_high().unwrap();
            wait();
            led_green.set_low().unwrap();
            wait();
        }
    }
}


