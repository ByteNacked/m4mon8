#![allow(dead_code)]

use super::rtt_print;
use stm32f1xx_hal::pac::GPIOG;

pub static mut BUTTON_LEFT: Button<Left> = Button::new(Left {});
pub static mut BUTTON_RIGHT: Button<Right> = Button::new(Right {});

pub trait ButtonPin {
    fn state(&self) -> State;
}

pub struct Left {}
impl ButtonPin for Left {
    fn state(&self) -> State {
        let gpiog = unsafe { &*GPIOG::ptr() };
        gpiog.idr.read().idr8().bit().into()
    }
}

pub struct Right {}
impl ButtonPin for Right {
    fn state(&self) -> State {
        let gpiog = unsafe { &*GPIOG::ptr() };
        gpiog.idr.read().idr9().bit().into()
    }
}

pub enum State {
    Pressed,
    Released,
}

impl Into<State> for bool {
    fn into(self) -> State {
        match self {
            false => State::Pressed,
            true => State::Released,
        }
    }
}

enum Event {
    KeyDown,
    KeyUp,
    None,
}

#[derive(Debug, Copy, Clone)]
pub enum Press {
    Short,
    Long,
    None,
}

pub struct Button<P> {
    prev: State,
    p: Press,
    press_len: u32,
    pin: P,
}

impl<P: ButtonPin> Button<P> {
    const fn new(pin: P) -> Self {
        Button {
            prev: State::Released,
            p: Press::None,
            press_len: 0u32,
            pin,
        }
    }

    pub fn update_state_100hz(&mut self) {
        let pin_state = self.pin.state();

        match self.prev {
            State::Pressed => {
                if let State::Pressed = pin_state {
                    self.press_len += 1;
                } else {
                    self.prev = pin_state;
                    /*определение длины нажатия или его остутствия (джиттер)*/
                    match self.press_len {
                        0..=5 => (),
                        6..=255 => {
                            self.p = Press::Short;
                            rtt_print!("Button press: {:?} in ticks {}", self.p, self.press_len);
                        }
                        _ => {
                            self.p = Press::Long;
                            rtt_print!("Button press: {:?} in ticks {}", self.p, self.press_len);
                        }
                    }
                    self.press_len = 0;
                }
            }
            State::Released => {
                if let State::Pressed = pin_state {
                    self.press_len += 1;
                    self.prev = pin_state;
                }
            }
        }
    }

    /// Гонка данных если вызывать в прерывании более высокого приоритета, чем то что вызывает update
    pub unsafe fn is_pressed(&mut self) -> Press {
        let p = self.p;
        self.p = Press::None;
        p
    }
}
