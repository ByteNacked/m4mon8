use super::rtt_print;
use core::ops::{Generator, GeneratorState};
use core::pin::Pin;
use cortex_m::asm;
use cortex_m::asm::{bkpt, delay, wfi};
use cortex_m_rt::{entry, exception};

fn func1() -> u32 {
    1
}

fn func2() -> u32 {
    2
}

fn func3() -> u32 {
    3
}

const fn up_to(
    limit: u32,
) -> impl Generator<Yield = u32, Return = u32> + core::marker::Unpin + core::marker::Sized {
    move || {
        yield func1();
        yield func2();
        yield func3();

        return limit;
    }
}

fn test() {
    unsafe {
        asm!("NOP" : : : : "volatile");
        asm!("NOP");
        asm!("NOP");
    }
}

static mut MY_GEN: impl Generator<Yield = u32, Return = u32> + core::marker::Unpin = up_to(2);

#[allow(dead_code)]
pub fn schedule() {
    rtt_print!("Gen size {}", core::mem::size_of_val(unsafe { &MY_GEN }));

    //let b : SmallBox<dyn Generator<Yield=u32, Return = u32>, S4>= SmallBox::new(
    //    || {
    //        yield 0;
    //        yield 1;
    //        yield 2;
    //        return 3;
    //    }
    //);

    match Pin::new(unsafe { &mut MY_GEN }).resume() {
        GeneratorState::Yielded(num) => {
            rtt_print!("Step : {}", num);
        }
        GeneratorState::Complete(_) => {
            rtt_print!("Finish step!");
        }
    }
}

#[exception]
fn SVCall() {
    //asm!("
    //    tst lr, #4
    //    ite eq
    //    mrseq r0, msp
    //    mrsne r0, psp
    //    ldr r1, [r0, #24]
    //    ldrb r1, [r1, #-2]
    //    ldr pc, [r2, r1, lsl #2]
    //"   :
    //    : "{r2}"(T::first())
    //    : "r0", "r1", "cc"
    //    : "volatile"
    //);
}

#[exception]
fn PendSV() {
    test();
}

/////////////////////////////////////////////////////////////////////////////////////////
