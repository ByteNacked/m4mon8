#![allow(dead_code)]

struct Beeper {}

#[derive(Copy, Clone)]
struct Sound {
    sound: u32,
    count: u32,
}

impl Beeper {
    pub fn new() -> Self {
        Beeper {}
    }

    pub fn init(&mut self) {
        //apb1.enr().modify(|_, w| w.$timXen().set_bit());
        //apb1.rstr().modify(|_, w| w.$timXrst().set_bit());
        //apb1.rstr().modify(|_, w| w.$timXrst().clear_bit());
    }
}
