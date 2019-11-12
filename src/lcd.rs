#![allow(dead_code)]

use super::pause;
use color::*;
use stm32f1xx_hal::prelude::*;

//   Ориентация:
//
//    x
//    o---------------------> width
//  y |
//    |
//    |
//    |
//    |
//    |  height
//    Y
//

pub const LCD_WIDTH: usize = 320;
pub const LCD_HEIGHT: usize = 240;
pub const FULL_SCREEN_RECT: Rect = Rect {
    x: 0,
    y: 0,
    w: LCD_WIDTH,
    h: LCD_HEIGHT,
};

#[derive(Copy, Clone, Debug)]
pub struct Rect {
    pub x: usize,
    pub y: usize,
    pub w: usize,
    pub h: usize,
}

impl Rect {
    pub fn new(x: usize, y: usize, w: usize, h: usize) -> Self {
        Rect { x, y, w, h }
    }
}

pub struct Lcd {}

impl Lcd {
    pub const fn new() -> Self {
        Lcd {}
    }

    pub fn init(&mut self) {
        use io::*;
        use ports::*;

        // Проверка тактирования GPIO
        lcd_init_check();
        // Снимаем сигнал RES на LCD
        lcd_reset(true);
        // Выбор типа интерфейса 16 бит
        AdrIndex::get().write(0x23);
        // Инициализая контроллера дисплея
        Self::init_disp_controller();
        // Очистка памяти дисплея
        self.clear();
        // Пауза 1 сек
        pause(1000.ms());
        // Включаем питание подсветки
        lcd_pwr(true);
        // Включаем видимость дисплея
        Self::display_enable(true);
    }

    pub fn set_pixel(&mut self, _x: usize, _y: usize) {}

    pub fn clear(&mut self) {
        let rect = FULL_SCREEN_RECT;
        self.fill_rect_with_color(rect, Color::Black);
    }

    /// Заливка области установленным цветом
    pub fn fill_rect_with_color<C>(&mut self, rect: Rect, color: C)
    where
        C: Into<u16>,
    {
        Self::set_rect(rect);
        Self::fill_with_color(rect.w * rect.h, color.into());
    }

    /// Заливка области установленным битмапом
    pub fn fill_rect_with_bitmap(&mut self, rect: Rect, bitmap: &[u16]) {
        Self::set_rect(rect);
        Self::fill_with_bitmap(rect.w * rect.h, bitmap);
    }

    /// Инициализация контроллера дисплея
    fn init_disp_controller() {
        use io::*;

        let adr_index = AdrIndex::get();
        let adr_parmt = AdrParmt::get();

        // disp off
        adr_index.write(0x05);
        adr_parmt.write(0x00);

        // RGB interface control
        adr_index.write(0x02);
        adr_parmt.write(0x00);

        // standby_Off
        adr_index.write(0x10);
        adr_parmt.write(0x00);

        adr_index.write(0x30);
        adr_parmt.write(0x00);

        adr_index.write(0x31);
        adr_parmt.write(0x00);

        adr_index.write(0x32);
        adr_parmt.write(0x00);

        Self::set_entry_mode();
    }

    fn set_entry_mode() {
        use io::*;
        let adr_index = AdrIndex::get();
        let adr_parmt = AdrParmt::get();

        adr_index.write(0x03);
        adr_parmt.write(0x30);
    }

    /// команда включения/выключения отображения
    fn display_enable(on: bool) {
        use io::*;
        if on {
            // disp on
            AdrIndex::get().write(0x05);
            AdrParmt::get().write(0x01);
        } else {
            // disp off
            AdrIndex::get().write(0x05);
            AdrParmt::get().write(0x00);
        }
    }

    /// Установка области развертки
    fn set_rect(rect: Rect) {
        let x1 = rect.x;
        let x2 = rect.x + rect.w - 1;
        let y1 = rect.y;
        let y2 = rect.y + rect.h - 1;

        use io::*;
        let adr_index = AdrIndex::get();
        let adr_parmt = AdrParmt::get();

        adr_index.write(0x35);
        adr_parmt.write(x1 as u16);
        adr_index.write(0x36);
        adr_parmt.write(x2 as u16);
        let yy = ((y1 << 8) | y2) as u16;
        adr_index.write(0x37);
        adr_parmt.write(yy);

        adr_index.write(0x20);
        adr_parmt.write(y1 as u16);
        adr_index.write(0x21);
        adr_parmt.write(x1 as u16);
    }

    /// Залитие области развертки цветом
    fn fill_with_color(mut ln: usize, color: u16) {
        use io::*;
        AdrIndex::get().write(0x22);
        while ln != 0 {
            AdrParmt::get().write(color);
            ln -= 1;
        }
    }
    /// Залитие области развертки битмапом
    fn fill_with_bitmap(ln: usize, bitmap: &[u16]) {
        use io::*;
        AdrIndex::get().write(0x22);
        for i in 0..ln {
            AdrParmt::get().write(bitmap[i]);
        }
    }
}

mod ports {
    use stm32f1xx_hal::pac::{GPIOC, GPIOF, RCC};

    /// Проверяем что тактирование подано
    pub fn lcd_init_check() {
        let rcc = unsafe { &*RCC::ptr() };
        let mut p_rdy = rcc.apb2enr.read().iopcen().is_enabled();
        p_rdy &= rcc.apb2enr.read().iopfen().is_enabled();

        assert!(p_rdy, "Peripheral is not enabled")
    }
    /// Логический сброс
    pub fn lcd_reset(on: bool) {
        let gpioc = unsafe { &*GPIOC::ptr() };
        gpioc.odr.modify(|_, w| w.odr0().bit(on));
    }

    /// Вкл/Выкл всей схемы OLED (питание подсветки)
    pub fn lcd_pwr(on: bool) {
        let gpiof = unsafe { &*GPIOF::ptr() };
        gpiof.odr.modify(|_, w| w.odr15().bit(on));
    }
}

mod io {
    use volatile_register::{RW, WO};

    #[repr(C)]
    pub struct AdrIndex {
        r: RW<u16>,
    }

    #[repr(C)]
    pub struct AdrParmt {
        r: WO<u16>,
    }

    impl AdrIndex {
        pub fn get() -> &'static mut AdrIndex {
            unsafe { &mut *(0x6000_0000 as *mut AdrIndex) }
        }
        pub fn read(&mut self) -> u16 {
            self.r.read()
        }
        pub fn write(&mut self, bb: u16) {
            unsafe { self.r.write(bb) };
        }
    }

    impl AdrParmt {
        pub fn get() -> &'static mut AdrParmt {
            unsafe { &mut *(0x6008_0000 as *mut AdrParmt) }
        }
        pub fn write(&mut self, bb: u16) {
            unsafe { self.r.write(bb) };
        }
    }
}

pub mod color {
    pub enum Color {
        White = 0b11111_111111_11111,
        Black = 0b00000_000000_00000,
        Green = 0b00000_111111_00000,
        Blue = 0b00000_000000_11111,
        Red = 0b11111_000000_00000,
    }

    impl Into<u16> for Color {
        fn into(self) -> u16 {
            self as u16
        }
    }

    pub struct RGB(pub u8, pub u8, pub u8);

    /// Переводим RGB в формат 5-6-5
    impl Into<u16> for RGB {
        fn into(self) -> u16 {
            let r: u16 = (self.0 / 8) as u16;
            let g: u16 = (self.1 / 4) as u16;
            let b: u16 = (self.2 / 8) as u16;

            (r << 11) & 0b11111_000000_00000
                | (g << 6) & 0b00000_111111_00000
                | b & 0b00000_000000_11111
        }
    }
}
