#![allow(dead_code)]

use super::lcd::*;
use super::pause;
use stm32f1xx_hal::time::U32Ext;

//const RUST_LOGO : &'static[u8] = include_bytes!("../pic/rust-logo-white_t.bmp");
//const RUST_EVA : &'static[u8] = include_bytes!("../pic/rust_eva_logo-t.bmp");
//const RUST_EMB_240X289 : &'static[u8] = include_bytes!("../pic/rust_emb_240x289_t.bmp");
//const _RUST_BROWN : &'static[u8] = include_bytes!("../pic/rust_rust_200x200_t.bmp");
//

#[allow(unused_variables)]
pub fn draw_sreens(lcd: &mut Lcd) {
    //    let rust_emb : &'static[u16] = from_u8_slice(RUST_EMB_240X289);
    //    lcd.fill_rect_with_bitmap(Rect{x : 0, y : 0, w : 289, h : 240}, rust_emb);
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    let rust_eva16 : &'static[u16] = from_u8_slice(RUST_EVA);
    //    lcd.fill_rect_with_bitmap(FULL_SCREEN_RECT, rust_eva16);
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());

    //    lcd.clear();
    //    let rust_logo16 : &'static[u16] = from_u8_slice(RUST_LOGO);
    //    lcd.fill_rect_with_bitmap(Rect { x : 75, y : 20, w : 200, h : 200}, rust_logo16);
    //    pause(1000.ms());

    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
    //    pause(1000.ms());
}

pub fn from_u8_slice(slice: &[u8]) -> &[u16] {
    use core::mem::{size_of, transmute};
    use core::slice::from_raw_parts;

    let ptr: *const u16 = unsafe { transmute(slice.as_ptr()) };
    let len: usize = slice.len() / (size_of::<u16>() / size_of::<u8>());

    unsafe { from_raw_parts(ptr, len) }
}
