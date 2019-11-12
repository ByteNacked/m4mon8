#![allow(dead_code)]

pub mod frame;

use crate::pac::DWT;
use crate::pld::{PldAdcMode, PldAfeFifoOut as PldAfeFifo, PldCfg, PldFifo, PldGie, PldIe, PldRdSpi, PldSpi};
use crate::rtt_print;
//use cortex_m::asm::{dsb, dmb, isb, nop};
pub use frame::{AdcFrame, AfeFrame};

pub struct MegaAdc {
    adc_frame: AdcFrame,
    afe_frame: AfeFrame,
    cnt: u16,
    afe_cnt: u16,
    change: &'static tables::Change,
}


impl MegaAdc {
    pub const fn new() -> Self {
        MegaAdc {
            adc_frame: AdcFrame::new(),
            afe_frame: AfeFrame::new(),
            cnt: 0,
            afe_cnt: 0,
            change: &tables::AD1298_CHANGE_12X10,
        }
    }

    pub fn init(&mut self) {
        cmd::adc_stop();
        busy_wait_cycles!(72000 * 40);

        cmd::write(&tables::ADC1298_STOP_CMD);

        PldRdSpi::get().write(0x0018); // Установка 32000 Гц реограф
        PldFifo::get().write(0x0018);

        PldAdcMode::get().write(0x0002); // Режим программирования АЦП
        busy_wait_cycles!(72000 * 5);

        PldCfg::get().write(0x0051); // Сброс FIFO и блоков АЦП
        busy_wait_cycles!(72000 * 1);

        PldCfg::get().write(0x0032); // Снять сброс с FIFO + режим программного опроса
        busy_wait_cycles!(72000 * 1);

        // Сброс ADC
        {
            cmd::adc_reset_on();
            busy_wait_cycles!(72000 * 10);
            cmd::adc_reset_off();
        }

        cmd::write(&tables::ADC1298_STOP_CMD); // Опять?
        cmd::write(&tables::ADC1298_SET1KHZ_CMD);
        cmd::write(&tables::ADC1298_SIGNAL_CMD);
        cmd::write(&tables::ADC1298_START_CMD);

        // TODO: read-out tests

        PldIe::get().write(0x0008); // Разрешение выдачи прерывания FIFO_AEMPTY на ногу RSV1 - нужно читать 41 слово
        PldGie::get().write(0x0001); // Глобальное разрешение прерываний внутри PLD

        self.init_afe();

        PldAdcMode::get().write(0x0000); // Принимаем данные из АЦП в PLD, рабочий режим // 0 - работа с FIFO. 4 - работа с SPI
        busy_wait_cycles!(72000 * 1);

        cmd::adc_start(); // Поехали!
    }

    fn init_afe(&self) {
        afe::reset_on();
        busy_wait_cycles!(72000 * 2);
        afe::reset_off();
        busy_wait_cycles!(72000 * 2);

        // Проводим диагностику, максимальное время - 16 мс
        afe::cmd_write(&afe::AFE4490_DIAG);
        busy_wait_cycles!(72000 * 20);

        afe::cmd_write(&afe::AFE4490_PROG1);
        afe::cmd_write(&afe::AFE4490_PROG2);
        afe::cmd_write(&afe::AFE4490_PROG3);
        afe::cmd_write(&afe::AFE4490_PROG4);
        afe::cmd_write(&afe::AFE4490_PROG5);
        afe::cmd_write(&afe::AFE4490_PROG6);
        afe::cmd_write(&afe::AFE4490_PROG7);
        afe::cmd_write(&afe::AFE4490_PROG8);
        afe::cmd_write(&afe::AFE4490_PROG9);
        afe::cmd_write(&afe::AFE4490_PROGA);
        afe::cmd_write(&afe::AFE4490_PROGB);
        afe::cmd_write(&afe::AFE4490_PROGC);
        afe::cmd_write(&afe::AFE4490_PROGD);
        afe::cmd_write(&afe::AFE4490_PROGE);
        afe::cmd_write(&afe::AFE4490_PROGF);
        afe::cmd_write(&afe::AFE4490_PROG10);
        afe::cmd_write(&afe::AFE4490_PROG11);
        afe::cmd_write(&afe::AFE4490_PROG12);
        afe::cmd_write(&afe::AFE4490_PROG13);
        afe::cmd_write(&afe::AFE4490_PROG14);
        afe::cmd_write(&afe::AFE4490_PROG15);
        afe::cmd_write(&afe::AFE4490_PROG16);
        afe::cmd_write(&afe::AFE4490_PROG17);
        afe::cmd_write(&afe::AFE4490_PROG18);
        afe::cmd_write(&afe::AFE4490_PROG19);
        afe::cmd_write(&afe::AFE4490_PROG1A);
        afe::cmd_write(&afe::AFE4490_PROG1B);
        afe::cmd_write(&afe::AFE4490_PROG1C);
        afe::cmd_write(&afe::AFE4490_PROG1D);
        afe::cmd_write(&afe::AFE4490_PROG1E);
        afe::cmd_write(&afe::AFE4490_PROG20);
        afe::cmd_write(&afe::AFE4490_PROG21);
        afe::cmd_write(&afe::AFE4490_PROG22);
        afe::cmd_write(&afe::AFE4490_PROG23);
        afe::cmd_write(&afe::AFE4490_PROG29);

        afe::cmd_write(&afe::AFE4490_SPIREADEN);

        // TODO: read-out tests
    }

    pub fn try_samples(&mut self, cb : &mut impl FnMut(&AdcFrame, &AfeFrame) ) {
        while cmd::is_adc_rdy() {
            match (self.try_read_adc1298_fifo(), self.try_read_afe_fifo()) {
                (Ok(_), Ok(_)) => cb(&self.adc_frame, &self.afe_frame),
                _ => self.fifo_reset(),
            }
        }
    }

    pub fn try_read_adc1298_fifo(&mut self) -> Result<(), ()> {
        // Дети, не повторяейте это дома
        let buf : &mut [u32; 16 + 1] = unsafe { core::mem::transmute(&mut self.adc_frame) };

        // Вычитываем заголовок
        let head = PldFifo::get().read(); // голова проверка ниже по коду
        let _dummy = PldFifo::get().read(); // пустые вычитывания статусов
        let _dummy = PldFifo::get().read();
        let _dummy = PldFifo::get().read();
        let status_l: u32 = PldFifo::get().read() as u32; // младшая часть статусного рега
        let status_h: u32 = PldFifo::get().read() as u32; // старшая
        let _status: u32 = (status_h << 16) | status_l;

        // тест корректности головы
        if (head ^ 0xFEFE) != 0 {
            rtt_print!("head");
            Err(())?
        }

        // Вычитываем точки
        for i in 0..16 {
            let bb_l = PldFifo::get().read() as u32;
            let bb_h = PldFifo::get().read() as u32;
            let mut bb: u32 = (bb_h << 16) | bb_l;

            // расширение знака
            unsafe {
                asm!( "sbfx $0, $1, #0, #24" : "=r"(bb) : "r"(bb) :: "volatile" );
            }

            // исключение ???
            if bb == 0xFF80_0000 {
                bb += 1 << tables::AD1298_SHIFT[i];
            }

            // оставляем нужное количество разрядов и переставляем каналы
            buf[self.change[i]] = bb >> tables::AD1298_SHIFT[i];
        }

        // Вычитываем хвост
        let _dummy = PldFifo::get().read(); // чтение CRC
        let cnt = PldFifo::get().read(); // счётчик
        let _tail = PldFifo::get().read(); // Хвост 0xBDBD читаем но не проверяем

        // Дописываем счетчик в кадр
        buf[16] = cnt as u32;

        // контроль счетчика
        if cnt != self.cnt {
            rtt_print!("{} != {}", cnt, self.cnt);
            self.cnt = cnt.wrapping_add(1);
            Err(())?
        }

        self.cnt = cnt.wrapping_add(1);
        //busy_wait_cycles!(100);

        Ok(())
    }

    fn fifo_reset(&mut self) {
        self.cnt = 0;
        self.afe_cnt = 0;

        cmd::adc_stop();

        PldAdcMode::get().write(0x0002);
        busy_wait_cycles!(72000 * 5);

        PldCfg::get().write(0x0073);
        busy_wait_cycles!(72000 * 1);

        PldCfg::get().write(0x0072);
        busy_wait_cycles!(72000 * 1);

        PldIe::get().write(0x0008); // Разрешение выдачи прерывания FIFO_AEMPTY на ногу RSV1 - нужно читать 41 слово
        PldGie::get().write(0x0001); // Глобальное разрешение прерываний внутри PLD

        self.init_afe();

        PldAdcMode::get().write(0x0000);
        busy_wait_cycles!(72000 * 6); // Принимаем данные из АЦП в PLD, рабочий режим // 0 - работа с FIFO. 4 - работа с SPI

        cmd::adc_start();
    }

    pub fn try_read_afe_fifo(&mut self) -> Result<(), ()> {
        // ------------Здесь закончился опрос ADS1298---------------------
        // ------------Теперь опрашиваем AFE4490-оксиметр-----------------
        //-------------Нужно вычитать 12 слов-----------------------------
        //-------------22 разряда со знаком-------------------------------
        //
        // FEFD		Маркер начала кадра								0
        // XXXX		Номер измерения в двоичном коде		1
        // ------------------------------------------------------------------------------------------
        // XXXX		LED2VAL 			Low word						2  ir
        // 00XX 	LED2VAL				High word						3
        // XXXX		ALED2VAL	 		Low word						4  bg-ir
        // 01XX 	ALED2VAL			High word						5
        // XXXX		LED1VAL 			Low word						6  red
        // 02XX 	LED1VAL				High word						7
        // XXXX		ALED1VAL			Low word						8  bg-red
        // 03XX 	ALED1VAL			High word						9
        // -------------------------------------------------------------------------------------------
        // XXXX		Номер измерения в двоичном коде		10
        // BCBC		Маркер конца кадра								11

        // Дети, не повторяейте это дома
        let buf : &mut [u32; 4 + 1] = unsafe { core::mem::transmute(&mut self.afe_frame) };

        let head = PldAfeFifo::get().read();
        let cnt = PldAfeFifo::get().read();

        // тест корректности головы
        if (head ^ 0xFEFD) != 0 {
            rtt_print!("afehead");
            Err(())?
        }

        for i in 0..4 {
            let bb_l = PldAfeFifo::get().read() as u32;
            let bb_h = PldAfeFifo::get().read() as u32;
            let mut bb: u32 = (bb_h << 16) | bb_l;

            // расширение знака
            unsafe {
                asm!( "sbfx $0, $1, #0, #22" : "=r"(bb) : "r"(bb) :: "volatile" );
            }

            buf[i] = bb;
        }

        let _cnt = PldAfeFifo::get().read();
        let tail = PldAfeFifo::get().read();

        // Дописываем счетчик в кадр
        buf[4] = cnt as u32;

        // контроль счетчика
        if cnt != self.afe_cnt {
            rtt_print!("afe {} != {}", cnt, self.afe_cnt);
            self.afe_cnt = cnt.wrapping_add(1);
            Err(())?
        }

        self.afe_cnt = cnt.wrapping_add(1);

        Ok(())
    }
}

#[rustfmt::skip]
mod cmd {
    use stm32f1xx_hal::pac::{GPIOC, GPIOB, GPIOE, DWT};
    use crate::pld::PldSpi;

    pub fn is_adc_rdy() -> bool {
        let gpioe = unsafe { &*GPIOE::ptr() };
        gpioe.idr.read().idr0().is_low()
    }

    pub fn adc_reset_on() {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.odr.modify(|_, w| w.odr13().low());
    }

    pub fn adc_reset_off() {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.odr.modify(|_, w| w.odr13().high());
    }

    pub fn adc_start() {
        let gpioc = unsafe { &*GPIOC::ptr() };
        gpioc.odr.modify(|_, w| w.odr7().high());
    }

    pub fn adc_stop() {
        let gpioc = unsafe { &*GPIOC::ptr() };
        gpioc.odr.modify(|_, w| w.odr7().low());
    }

    pub fn write(cmd : &[u16]) {
        for i in cmd { data_send(*i); }
    }

    fn data_send(b : u16) {
        PldSpi::get().write(b);
        busy_wait_cycles!(72000 * 1);
    }
}

#[rustfmt::skip]
mod tables {
    pub const ADC1298_START_CMD : [u16;3]  = [ 0x2200,0x1110,0x6600 ]; // Команда Enable Data Continious Mode - 0x10
    pub const ADC1298_STOP_CMD : [u16;3]   = [ 0x2200,0x1111,0x6600 ]; // Команда Stop Data Continious Mode - 0x11

    pub const ADC1298_SET1KHZ_CMD : [u16;5]  = [ 0x2200,0x1141,0x1100,0x1104,0x6600 ]; // Запись частот 1 КГц - @(0x4) Никакая другая частота невозможна
    pub const ADC1298_GETKHZ_CMD : [u16;5]   = [ 0x2200,0x1121,0x1100,0x1100,0x6600 ]; // Чтение регистра частот

    // Инициализация для внешнего сигнала
    pub const ADC1298_SIGNAL_CMD : [u16;140] =
    [
        0x0200,0x0142,0x0100,0x0115,0x0600,  // Перевод тестового сигнала на внутренний источник - @(0x10)
        0x0200,0x0143,0x0100,0x01C0,0x0600,  // Использовать внутреннюю опору
        0x0200,0x0145,0x0100,0x0110,0x0600,  // REO
        0x0200,0x0146,0x0100,0x0110,0x0600,  // Gain для канала 1 = 1 для АЦП 0  - @(0x6) <- 0x50
        0x0200,0x0147,0x0100,0x0110,0x0600,  // Gain для канала 2 = 1 для АЦП 0  - @(0x7) <- 0x50
        0x0200,0x0148,0x0100,0x0110,0x0600,  // Gain для канала 3 = 1 для АЦП 0  - @(0x8) <- 0x50
        0x0200,0x0149,0x0100,0x0160,0x0600,  // Gain для канала 4 = 1 для АЦП 0  - @(0x9) <- 0x50 в 12 раз канал Микрофона
        0x0200,0x014A,0x0100,0x0110,0x0600,  // Gain для канала 5 = 1 для АЦП 0  - @(0xa) <- 0x50
        0x0200,0x014B,0x0100,0x0110,0x0600,  // Gain для канала 6 = 1 для АЦП 0  - @(0xb) <- 0x50
        0x0200,0x014C,0x0100,0x0150,0x0600,  // ECG - C6F

        0x2000,0x1042,0x1000,0x1015,0x6000,  // Перевод тестового сигнала на внутренний источник - @(0x10)
        0x2000,0x1043,0x1000,0x10C0,0x6000,  // Использовать внутреннюю опору
        0x2000,0x1045,0x1000,0x1010,0x6000,  // REO
        0x2000,0x1046,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x1047,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x1048,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x1049,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x104A,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x104B,0x1000,0x1050,0x6000,  // ECG
        0x2000,0x104C,0x1000,0x1050,0x6000,  // ECG

        // Отключены рео настройки по просьбе Севы, 03.10.2019
        // 0x0200,0x0156,0x0100,0x01A3,0x0600,  // Включить внутренний демодулятор, использовать внешний модулятор
        // 0x2000,0x1056,0x1000,0x10A3,0x6000,  // Включить внутренний демодулятор, использовать внешний модулятор

        0x2000,0x1044,0x1000,0x100F,0x6000,   // Включить подтяжки
        0x2000,0x104F,0x1000,0x10F6,0x6000, //LOFFP // Это АЦП отвечает за ЭКГ
        0x2000,0x1050,0x1000,0x1080,0x6000, //LOFFN
        0x2000,0x1051,0x1000,0x1034,0x6000, //FLIP

        0x0200,0x0144,0x0100,0x010F,0x0600,   // Включить подтяжки
        0x0200,0x014F,0x0100,0x0180,0x0600, //LOFFP // В этом АЦП только один ЭКГ канал
        0x0200,0x0150,0x0100,0x0180,0x0600, //LOFFN
        0x0200,0x0151,0x0100,0x0180,0x0600, //FLIP
    ];

    // Таблица сдвига для каналов, оставляем только нужное количество разрядов
    pub const AD1298_SHIFT : [usize; 16] =
    [
        (24-22),// int reo0;  // 21 рео
        (24-22),// int reo1;  // 21 рео
        (24-16),// int agnd;
        (24-19),// int ch4;   // 19 экг
        (24-16),// int agnd;
        (24-19),// int ch2;   // 19 экг
        (24-16),// int agnd;
        (24-19),// int chF;   // 19 экг
        (24-16),// int mic1;  // микрофон канала АД
        (24-19),// int ch5;   // 19 экг
        (24-20),// int ad;   // давление
        (24-19),// int ch3;   // 19 экг
        (24-16),// int brth;  // канал APT - дыхание и храп с датчика давления
        (24-19),// int ch1;   // 19 экг
        (24-19),// int ch6;   // 19 экг
        (24-19),// int chL;   // 19 экг
    ];

    // Таблица перестановки каналов, для того, чтобы дальше использовать по порядку
    pub type Change = [usize; 17];
    pub const AD1298_CHANGE_12X10 : Change =
    [
        8,// int reo0;
        9,// int reo1;  // 19 экг
        13,// int agnd;  // Сюда нужно засунуть RED
        5,// int ch4;  // 19 экг
        14,// int agnd;  // Сюда нужно засунуть IR
        3,// int ch2;  // 19 экг
        15,// int agnd;  // 21 рео
        1,// int chF;  // 19 экг
        12,// int mic1;
        6,// int ch5;  // 19 экг
        11,// int ad;
        4,// int ch3;  // 19 экг
        10,// int brth;
        2,// int ch1;  // 19 экг
        7,// int ch6;
        0,// int chL;  // 19 экг
        16,// int Count
    ];
}

#[rustfmt::skip]
mod afe {
    // ----------Эти регистры задают времена диаграммы оксиметра AFE4490--------------------------
    // Программируются через автомат PLD, для этого нужны дополнительные байты 03 и 06 перед данными
    // 03 - опсутить CS, 06 - поднять CS
    // Значения в регистрах представляют собой количество тактов 4Мгц
    // Можно провести эксперименты и проверить, является ли эта диаграмма идеальной
    // Но эти значения рекомендованы TI
    // Частота съема данных - 1 КГц
    pub const AFE4490_DIAG      : [u16;5]     = [ 0x0300,0x0300,0x0300,0x0304,0x0600 ];         // Команда начать диагностику, включить чтение по SPI
    pub const AFE4490_SPIREADEN : [u16;5]     = [ 0x0300,0x0300,0x0300,0x0301,0x0600 ];         // Команда разрешить чтение по SPI
    pub const AFE4490_PROG1     : [u16;5]     = [ 0x0301,0x0300,0x030C,0x0319,0x0600 ];         // Программировать регистр по адресу 1
    pub const AFE4490_PROG2     : [u16;5]     = [ 0x0302,0x0300,0x030F,0x03FE,0x0600 ];         // Программировать регистр по адресу 2
    pub const AFE4490_PROG3     : [u16;5]     = [ 0x0303,0x0300,0x030C,0x0300,0x0600 ];         // Программировать регистр по адресу 3
    pub const AFE4490_PROG4     : [u16;5]     = [ 0x0304,0x0300,0x030F,0x03FF,0x0600 ];         // Программировать регистр по адресу 4
    pub const AFE4490_PROG5     : [u16;5]     = [ 0x0305,0x0300,0x0300,0x0319,0x0600 ];         // Программировать регистр по адресу 5
    pub const AFE4490_PROG6     : [u16;5]     = [ 0x0306,0x0300,0x0303,0x03FE,0x0600 ];         // Программировать регистр по адресу 6
    pub const AFE4490_PROG7     : [u16;5]     = [ 0x0307,0x0300,0x0304,0x0319,0x0600 ];         // Программировать регистр по адресу 7
    pub const AFE4490_PROG8     : [u16;5]     = [ 0x0308,0x0300,0x0307,0x03FE,0x0600 ];         // Программировать регистр по адресу 8
    pub const AFE4490_PROG9     : [u16;5]     = [ 0x0309,0x0300,0x0304,0x0300,0x0600 ];         // Программировать регистр по адресу 9
    pub const AFE4490_PROGA     : [u16;5]     = [ 0x030A,0x0300,0x0307,0x03FF,0x0600 ];         // Программировать регистр по адресу A
    pub const AFE4490_PROGB     : [u16;5]     = [ 0x030B,0x0300,0x0308,0x0319,0x0600 ];         // Программировать регистр по адресу B
    pub const AFE4490_PROGC     : [u16;5]     = [ 0x030C,0x0300,0x0307,0x03FE,0x0600 ];         // Программировать регистр по адресу C
    pub const AFE4490_PROGD     : [u16;5]     = [ 0x030D,0x0300,0x0300,0x0304,0x0600 ];         // Программировать регистр по адресу D
    pub const AFE4490_PROGE     : [u16;5]     = [ 0x030E,0x0300,0x0303,0x03FF,0x0600 ];         // Программировать регистр по адресу E
    pub const AFE4490_PROGF     : [u16;5]     = [ 0x030F,0x0300,0x0304,0x0304,0x0600 ];         // Программировать регистр по адресу F
    pub const AFE4490_PROG10    : [u16;5]     = [ 0x0310,0x0300,0x0307,0x03FF,0x0600 ];         // Программировать регистр по адресу 10
    pub const AFE4490_PROG11    : [u16;5]     = [ 0x0311,0x0300,0x0308,0x0304,0x0600 ];         // Программировать регистр по адресу 11
    pub const AFE4490_PROG12    : [u16;5]     = [ 0x0312,0x0300,0x030B,0x03FF,0x0600 ];         // Программировать регистр по адресу 12
    pub const AFE4490_PROG13    : [u16;5]     = [ 0x0313,0x0300,0x030C,0x0304,0x0600 ];         // Программировать регистр по адресу 13
    pub const AFE4490_PROG14    : [u16;5]     = [ 0x0314,0x0300,0x030F,0x03FF,0x0600 ];         // Программировать регистр по адресу 14
    pub const AFE4490_PROG15    : [u16;5]     = [ 0x0315,0x0300,0x0300,0x0300,0x0600 ];         // Программировать регистр по адресу 15
    pub const AFE4490_PROG16    : [u16;5]     = [ 0x0316,0x0300,0x0300,0x0303,0x0600 ];         // Программировать регистр по адресу 16
    pub const AFE4490_PROG17    : [u16;5]     = [ 0x0317,0x0300,0x0304,0x0300,0x0600 ];         // Программировать регистр по адресу 17
    pub const AFE4490_PROG18    : [u16;5]     = [ 0x0318,0x0300,0x0304,0x0303,0x0600 ];         // Программировать регистр по адресу 18
    pub const AFE4490_PROG19    : [u16;5]     = [ 0x0319,0x0300,0x0308,0x0300,0x0600 ];         // Программировать регистр по адресу 19
    pub const AFE4490_PROG1A    : [u16;5]     = [ 0x031A,0x0300,0x0308,0x0303,0x0600 ];         // Программировать регистр по адресу 1A
    pub const AFE4490_PROG1B    : [u16;5]     = [ 0x031B,0x0300,0x030C,0x0300,0x0600 ];         // Программировать регистр по адресу 1B
    pub const AFE4490_PROG1C    : [u16;5]     = [ 0x031C,0x0300,0x030C,0x0303,0x0600 ];         // Программировать регистр по адресу 1C
    pub const AFE4490_PROG1D    : [u16;5]     = [ 0x031D,0x0300,0x030F,0x03FF,0x0600 ];         // Программировать регистр по адресу 1D   // 1 КГц
    pub const AFE4490_PROG1E    : [u16;5]     = [ 0x031E,0x0300,0x0301,0x0302,0x0600 ];         // Программировать регистр по адресу 1E   // 3 усреднения (больше не ставить!!!), включить внутреннее тактирование
    pub const AFE4490_PROG20    : [u16;5]     = [ 0x0320,0x0300,0x0300,0x0301,0x0600 ];         // Установки LED1 2-0 бит коэффициент усиления
    pub const AFE4490_PROG21    : [u16;5]     = [ 0x0321,0x0300,0x0300,0x0301,0x0600 ];         // Установки LED2 Ambient DAC current = 0 uA 2-0 бит коэффициент усиления
    pub const AFE4490_PROG22    : [u16;5]     = [ 0x0322,0x0300/*02*/,0x0300,0x0300,0x0600 ];      // Программировать регистр по адресу 22  // Регулировка тока для светодиодов
    pub const AFE4490_PROG23    : [u16;5]     = [ 0x0323,0x0300/*01*/,0x0302,0x0300,0x0600 ];      // Программировать регистр по адресу 23   // Использовать внешний тактовый сигнал на 8.192 МГц, опора - 0,5В
    pub const AFE4490_PROG29    : [u16;5]     = [ 0x0329,0x0300,0x0300,0x0380,0x0600 ];         // Программировать регистр по адресу 29 // Разрешить мониторить внутренние частоты

    //const u32 Afe4490Read[]     = { 0x0301,0x0300,0x0300,0x0300,0x0600 };         // Считать регистр по адресу в первой тетраде

    use stm32f1xx_hal::pac::{GPIOF, DWT};
    use crate::pld::PldAfeSpiMode;

    pub fn reset_on() {
        let gpiof = unsafe { &*GPIOF::ptr() };
        gpiof.odr.modify(|_, w| w.odr13().low());
    }

    pub fn reset_off() {
        let gpiof = unsafe { &*GPIOF::ptr() };
        gpiof.odr.modify(|_, w| w.odr13().high());
    }

    pub fn cmd_write(cmd : &[u16]) {
        for i in cmd { data_send(*i) }
    }

    fn data_send(b : u16) {
        PldAfeSpiMode::get().write(b);
        busy_wait_cycles!(72000 * 1);
    }

}
