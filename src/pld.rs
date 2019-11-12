#![allow(dead_code)]

use crate::pac::DWT;
use crate::rtt_print;
use nb::Error::{Other, WouldBlock};
use nb::{Error as NbError, Result as NbResult};
use volatile_register::{RO, RW, WO};

register_u16_rw! {PldPWMValve,          0x60380000} // ШИММ для клапана, или регистр общего назначения				чтение/запись
register_u16_rw! {PldAfeCmdResp_high_r, 0x60380002} // Чтение старшего слова ответа на команды от AFE440			чтение
register_u16_rw! {PldPWMsel,            0x60380002} // Разрешение работы компрессора и клапана 						запись
register_u16_rw! {PldFdpOut,            0x60380004} // Вывод FIFO от VS1063A (звук)									чтение
register_u16_rw! {PldPWMComp,           0x60380004} // ШИММ для компрессора											запись
register_u16_rw! {PldAfeSpiMode,        0x60380006} // конфигурационный регистр для управления AFE449				чтение/запись
register_u16_rw! {PldAfeCmd_low,        0x60380008} // Запись младшего слова команды для AFE4490					запись
register_u16_rw! {PldAfeFifoOut,        0x60380008} // Отсюда нужно забирать данные из AFE4490						чтение
register_u16_rw! {PldAfeAdcOut,         0x6038000c} // Вспомогательный регистр, чтение пинов периферии АВС			чтение  // 0-BTHOFF, 1-DIAG_END, 2-LED_ALM, 3-PD_ALM, 4-WIFIOFF
register_u16_rw! {PldAfeCmd_high_r,     0x6038000c} // Запись команды ЧТЕНИЯ, которая будет передана в AFE4490		запись
register_u16_rw! {PldAfeCmd_high,       0x6038000e} // Запись команды ЗАПИСИ, которая будет передана в AFE4490		запись
register_u16_rw! {PldAfeCmdResp_low_r,  0x6038000e} // Чтение младшего слова ответа на команды от AFE4490			чтение

register_u16_rw! {PldAdcMode, 0x6030_0000} // Регистр режимов работы автомата PLD
register_u16_rw! {PldCfg,     0x6030_0002} // Конфигурационный регистр PLD
register_u16_rw! {PldId,      0x6030_0004} // Регистр содержит версию прошивки PLD - чтение
register_u16_rw! {PldSpi,     0x6030_0004}
register_u16_rw! {PldGie,     0x6030_0006} // Разрешение глобальных прерваний
register_u16_rw! {PldIe,      0x6030_0008} // Регистр разрешения прерываний
register_u16_rw! {PldCfg5,    0x6030_000A}
register_u16_rw! {PldRdSpi,   0x6030_000C}
register_u16_rw! {PldFifo,    0x6030_000E} // Регистр данных FIFO

register_u16_rw! {PldTestReg, 0x6038_0000} // Внимание! Этим можно пользоваться только на этапе инициализации! АВС

pub fn pld_init() {
    // PllRdy
    // Проверка готовности PLL
    {
        let r = PldId::get();
        for _ in 0..100 {
            let res = r.read();
            rtt_print!("PldIDReg: {:X}", res);
            busy_wait_cycles!(72000 * 10);
            if res & 0x8000 == 0 {
                break;
            }
        }
        rtt_print!("PllRdy test OK");
    }

    // IdTest
    // Проверка кода идентификаци
    {
        let r = PldCfg::get();
        r.write(0x0073); // сброс FIFO
        busy_wait_cycles!(72000 * 5);
        r.write(0x0072); // сброс FIFO
        busy_wait_cycles!(72000 * 5);

        const PLD_ID_CODE: u16 = 0xA5;

        // Hесоответствие кода идентификации
        let v = PldId::get().read();
        if PldId::get().read() != PLD_ID_CODE {
            panic!("Pld ID code does NOT match: {:X}", v);
        }
        rtt_print!("Pld ID code does match: {:X}", v);
    }

    // FsmcTest
    // Проверка шины данных
    {
        let mut map_bus: u16 = 0;
        let mut rg: u16 = 1;
        let test_reg = PldTestReg::get();

        while rg != 0 {
            test_reg.write(!rg);
            map_bus |= !test_reg.read() ^ rg;
            //rtt_print!("map_bus : {:0>16b}", rg);

            test_reg.write(rg);
            map_bus |= test_reg.read() ^ rg;

            rg = rg << 1;
        }
        match map_bus == 0 {
            true => rtt_print!("Fsmc test OK"),
            false => panic!("Fsmc test FAILED"),
        }
    }

    // FIFO Test
    // Проверка работоспособности FIFO в PLL
    {
        let mut fifo_test_buf = [0u16; 0x100];
        PldAdcMode::get().write(0x0001); // Включить режим теста FIFO от STM32
        busy_wait_cycles!(72000 * 5);
        PldCfg::get().write(0x0056); // Снять сброс и включить режим заполнения старшего байта значащим байтом данных
        busy_wait_cycles!(72000 * 5);

        for i in 0..5 {
            for rg in 0u16..0x100 {
                PldCfg5::get().write(rg | !rg << 8);
                fifo_test_buf[rg as usize] = 0;
            }

            for rg in 0u16..0x100 {
                let bb = PldFifo::get().read();
                fifo_test_buf[rg as usize] = bb;

                if bb != rg | !rg << 8 {
                    rtt_print!("{:X?}", &fifo_test_buf[..]);
                    rtt_print!("FIFO test FAILED on {} attempt", i);
                    return;
                }
            }
        }

        rtt_print!("{:X?}", &fifo_test_buf[..]);
        rtt_print!("FIFO test OK");
    }
}

/// Включить ШИММ (0x01 - разрешить работу выхода клапана),
/// 0x02 - разрешить работу компрессора, 
/// 0x03 - разрешить работу обоих выходов, 
/// 0х00 - запретить работу обоих выходов 
pub fn pld_enable_pwm() {
    // Разрешить работу клапана и компрессора
    PldPWMsel::get().write(0x0003);
}

pub fn pld_emergency_block_unlock() {
    // Если был аварийный сброс давления, нужно для снятия флага прочитать этот регистр
    let _ = PldId::get().read();
}