#![no_std]
#![no_main]
#![feature(asm)]
#![feature(generators, generator_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(const_fn)]
#![feature(never_type)]
#![feature(unsize)]
#![allow(unused_imports)]
#![allow(incomplete_features)]
#![allow(dead_code)]

extern crate jlink_rtt;

#[cfg(debug_assertions)]
extern crate panic_rtt;

#[macro_use]
mod macros;
mod dynbox;
//mod adc;
//mod beeper;
//mod button;
//mod cmd;
//mod debug;
//mod embbox;
//mod lcd;
//mod mega_adc;
//mod pause;
//mod pld;
//mod port;
//mod sche;
//mod splash;
//mod tps;
mod usb;
//mod cb;

use core::fmt::Binary;
use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m::asm::{bkpt, delay, wfi};
use cortex_m::peripheral;
use cortex_m::interrupt::{disable as int_disable, enable as int_enable};
use cortex_m_rt::{entry, exception};
//use embbox::EmbBox;
use embedded_hal::digital::v2::OutputPin;
pub(crate) use jlink_rtt::rtt_print;
//use lcd::{color::*, Lcd, Rect, FULL_SCREEN_RECT, LCD_HEIGHT, LCD_WIDTH};
//use mega_adc::{ MegaAdc, AdcFrame, AfeFrame};
use nb::block;
//use pause::pause;
//use port::*;
use stm32l4xx_hal::{
    //afio, 
    i2c, pac,
    prelude::*,
    rtc::Rtc,
    stm32,
    stm32::interrupt,
    stm32::Interrupt,
    time,
    timer::{Event, Timer},
};
//use tps::Tps;
//use cb::CircularBuffer;
use dynbox::DynBox;
use core::ops::{Generator, GeneratorState};
use core::mem;

#[cfg(not(debug_assertions))]
#[panic_handler]
#[inline(never)]
fn panic(_info: &PanicInfo) -> ! {
    int_disable();
    rtt_print!("Panic handler! Reseting...");
    
    cortex_m::peripheral::SCB::sys_reset()
}

use core::sync::atomic::{AtomicI32, AtomicIsize, Ordering};

#[rtfm::app(device = pac)]
const APP: () = {
    struct Resources {
        // A resource
        #[init(AtomicI32::new(0))]
        shared: AtomicI32,
        //rtc : stm32l4xx_hal::rtc::Rtc,
        g_task : DynBox<dyn Generator<Yield = u32, Return = !>, [usize; 0x10]>,
    }

    #[init]
    fn init(_: init::Context) -> init::LateResources {

        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc.cfgr
            .sysclk(64.mhz())
            .hsi48(true)
            .freeze(&mut flash.acr);

        usb::usb_init(&clocks);

        ////let mut nvic = cp.NVIC;

        //let _mono_timer = time::MonoTimer::new(cp.DWT, clocks);

        //let mut pwr = dp.PWR;
        //let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);

        //let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);
        //rtc.listen_seconds();
        ////unsafe { peripheral::NVIC::unmask(Interrupt::RTC) };

        rtfm::pend(Interrupt::USART1);
        rtfm::pend(Interrupt::USART2);

        let gen = || {
            loop {
                yield 0u32;
                yield 1;
                yield 2;
            }
        };
        let g_task = DynBox::new(gen,[0usize;0x10]);

        init::LateResources {
            //rtc,
            g_task,
        }
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {

        loop {
            //rtt_print!("IDLE");
            //use crate::pac::DWT;
            //busy_wait_cycles!(72_000_000);
            usb::usb_interrupt();
        }
    }

    // `shared` can be accessed from this context
    #[task(binds = USART1, resources = [&shared], priority = 3)]
    fn uart0(cx: uart0::Context) {
        cx.resources.shared.fetch_add(1, Ordering::Relaxed); 
 
        rtt_print!("USART1: {}", cx.resources.shared.load(Ordering::Relaxed));
    }

    // `shared` can be accessed from this context
    #[task(binds = USART2, resources = [&shared], priority = 2)]
    fn uart1(cx: uart1::Context) {
        cx.resources.shared.fetch_add(1, Ordering::Relaxed); 
 
        rtt_print!("USART2: {}", cx.resources.shared.load(Ordering::Relaxed));
    }

    #[task(binds = RTC_WKUP, resources = [&shared/*, rtc*/], priority = 1)]
    fn rtc(cx: rtc::Context) {
        static mut G_TASK : DynBox<dyn Generator<Yield = u32, Return = !>, [usize; 0x10]> = DynBox::empty([0usize;0x10]);
        static mut G_RESET : bool = true;

        //cx.resources.rtc.clear_second_flag();
        cx.resources.shared.fetch_add(1, Ordering::Relaxed); 

        if *G_RESET {
            G_TASK.occupy( || {
                loop {
                    rtt_print!("Yield 0");
                    yield 0u32;
                    rtt_print!("Yield 1");
                    yield 1;
                    rtt_print!("Yield 2");
                    yield 2;
                }
            });
            *G_RESET = false;
        }

        use core::ops::DerefMut;
        use core::pin::Pin;

        //rtt_print!("RTC: {}", cx.resources.shared.load(Ordering::Relaxed));
        //rtt_print!("cb: {:?}", &G_TASK);

        let gen = G_TASK.deref_mut();
        unsafe {
            match Pin::new_unchecked(gen).resume() {
                    GeneratorState::Yielded(num) => { rtt_print!("Step : {}", num); }
                    GeneratorState::Complete(_)  => { rtt_print!("Finish step!"); }
            };
        }
    }
};

//static mut LCD: Lcd = Lcd::new();
//static mut RTC: Option<Rtc> = None;
//static mut TIM2: Option<Timer<pac::TIM2>> = None;
//static mut TIMER_PAUSE: Option<Timer<pac::TIM1>> = None;
//static mut MAGIC_NUM: Option<u32> = None;
//static mut MONO_TIMER: Option<time::MonoTimer> = None;

//type I2C1Driver = stm32f1xx_hal::i2c::BlockingI2c<
//    pac::I2C1,
//    (
//        stm32f1xx_hal::gpio::gpiob::PB8<
//            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
//        >,
//        stm32f1xx_hal::gpio::gpiob::PB9<
//            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
//        >,
//    ),
//>;

//static mut I2C1_DRIVER: Option<I2C1Driver> = None;
//static mut TPS: Option<Tps<I2C1Driver>> = None;
//static mut MEGA_ADC: MegaAdc = MegaAdc::new();
//static mut VB : CircularBuffer = CircularBuffer::new();

static mut DBG_TICK: u32 = 0;
//INTERRUPTS
#[interrupt]
fn TIM2() {
    //unsafe { TIM2.as_mut().unwrap().clear_update_interrupt_flag() };

    unsafe {
        DBG_TICK += 1;
    }
}

//#[interrupt]
//fn RTC() {
//    //let rtc = unsafe { &mut RTC.as_mut().unwrap() };
//    //rtc.clear_second_flag();
//
//    let _res = unsafe { DBG_TICK };
//    unsafe {
//        DBG_TICK = 0;
//    }
//
//    //let _ = rtt_print!("TIM2 hz: {}", res);
//}

#[exception]
fn SysTick() {
    static mut DBG_SYSTICK: u32 = 0;
    *DBG_SYSTICK += 1;

    unsafe {
        //button::BUTTON_LEFT.update_state_100hz();
        //button::BUTTON_RIGHT.update_state_100hz();
    }
}
