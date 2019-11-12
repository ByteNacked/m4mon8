use core::sync::atomic::{AtomicIsize, Ordering};
use cortex_m::{peripheral::NVIC, singleton};
use stm32f1xx_hal::pac::{ADC3, RCC};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};

pub static mut ACCEL_ADC: Option<Adc> = None;

#[derive(Debug)]
pub struct Adc {
    x: AtomicIsize,
    y: AtomicIsize,
    z: AtomicIsize,
}

impl Adc {
    pub fn new() -> Self {
        Adc {
            x: AtomicIsize::new(0),
            y: AtomicIsize::new(0),
            z: AtomicIsize::new(0),
        }
    }

    pub fn init(&mut self) {
        let rcc = unsafe { &*RCC::ptr() };

        // ADC Prescaler - PLCK2 divided by 8
        rcc.cfgr.modify(|_, w| w.adcpre().div8());
        // ADC 1 interface clock enable
        rcc.apb2enr.modify(|_, w| w.adc3en().enabled());

        let adc3 = unsafe { &*ADC3::ptr() };

        adc3.cr1.modify(|_, w| {
            w.scan()
                .enabled() // Режим сканирования
                .jauto()
                .enabled() // Автоматическое измерение добавленных каналов
                .jeocie()
                .enabled() // Прервание после окончания измерения добавленных каналов
        });

        adc3.cr2.modify(|_, w| w.cont().single());

        adc3.sqr3.modify(|_, w| unsafe {
            w.sq1().bits(4) // ACCZ    pin 30
        });

        adc3.jsqr.modify(|_, w| unsafe {
            w.jl()
                .bits(3) // Количество измерений добавленных каналов = 4
                .jsq1()
                .bits(7) // PWOUT   pin pf8
                .jsq2()
                .bits(5) // ACCY    pin 31
                .jsq3()
                .bits(6) // ACCX    pin 32
                .jsq4()
                .bits(8) // Current pin G1
        });

        // Interrupt
        unsafe { NVIC::unmask(Interrupt::ADC3) };
    }

    pub fn start_conversion(&self) {
        let adc3 = unsafe { &*ADC3::ptr() };
        adc3.cr2.modify(|_, w| {
            w.adon().enabled() // Включаем АЦП
        });
    }

    pub fn callback_conv_ready(&self) {
        let adc3 = unsafe { &*ADC3::ptr() };

        adc3.sr.modify(|_, w| {
            w.jeoc().clear() // Clear Injected channel end of conversion flag
        });

        let z = adc3.dr.read().bits() as i16 as isize - 2000; // ACC Z
        let y = adc3.jdr2.read().bits() as i16 as isize - 2000; // ACC Y
        let x = adc3.jdr3.read().bits() as i16 as isize - 2000; // ACC X
        let _c = adc3.jdr4.read().bits() as i32; // Current
        let _v = adc3.jdr1.read().bits() as i32; // Voltage

        self.x.store(x, Ordering::Relaxed);
        self.y.store(y, Ordering::Relaxed);
        self.z.store(z, Ordering::Relaxed);
    }

    pub fn get_axes(&self) -> (isize, isize, isize) {
        (
            self.x.load(Ordering::Relaxed),
            self.y.load(Ordering::Relaxed),
            self.z.load(Ordering::Relaxed),
        )
    }
}

#[interrupt]
fn ADC3() {
    unsafe {
        ACCEL_ADC.as_ref().unwrap().callback_conv_ready();
    }
}
