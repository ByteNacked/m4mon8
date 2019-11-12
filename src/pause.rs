use crate::TIMER_PAUSE;
use nb::block;
use stm32f1xx_hal::{prelude::*, time::MilliSeconds};

pub fn pause(mili: MilliSeconds) {
    unsafe {
        if let Some(t) = TIMER_PAUSE.as_mut() {
            t.start(mili);
            let _ = block!(t.wait());
        } else {
            panic!("No timer for pause!");
        }
    }
}
