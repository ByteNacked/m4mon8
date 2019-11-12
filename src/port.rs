#![allow(dead_code)]

const INPUT_ANALOG: u8 = 0;
const INPUT_FLOAT: u8 = 1;
const INPUT_PULL_UP_DOWN: u8 = 2;

#[rustfmt::skip]
pub fn port_init() {
    use stm32f1xx_hal::pac::{
        RCC,
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
        GPIOE,
        GPIOF,
        GPIOG
    };

    let rcc = unsafe { &*RCC::ptr() };
    rcc.apb2enr.modify(|_, w| {
        w.iopaen().enabled()
         .iopben().enabled()
         .iopcen().enabled()
         .iopden().enabled()
         .iopeen().enabled()
         .iopfen().enabled()
         .iopgen().enabled()
    });

    {  //GPIO A
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.odr.modify(|_, w| {
            w.odr0().high()
            .odr1().high()
            .odr2().low()
            .odr3().high()
            .odr4().high()
            .odr5().low()
            .odr6().low()
            .odr7().low()
            .odr8().high()
            .odr9().high()
            .odr10().high()
            .odr11().low()
            .odr12().low()
        });

        gpioa.crl.modify(|_, w| {
            w.mode0().input()
            .cnf0().bits(INPUT_FLOAT)
            .mode1().output2()
            .cnf1().push_pull()
            .mode2().output2()
            .cnf2().push_pull()
            .mode3().input()
            .cnf3().bits(INPUT_FLOAT)
            .mode4().output2()
            .cnf4().push_pull()
            .mode5().output2()
            .cnf5().push_pull()
            .mode6().input()
            .cnf6().bits(INPUT_FLOAT)
            .mode7().output2()
            .cnf7().push_pull()
        });

        gpioa.crh.modify(|_, w| {
            w.mode8().input()
            .cnf8().bits(INPUT_FLOAT)
            .mode9().output2()
            .cnf9().push_pull()
            .mode10().input()
            .cnf10().bits(INPUT_PULL_UP_DOWN)
            .mode11().input()
            .cnf11().bits(INPUT_PULL_UP_DOWN)
            .mode12().input()
            .cnf12().bits(INPUT_PULL_UP_DOWN)
        });
    }

    {  //GPIO B
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.odr.modify(|_, w| {
            w
            .odr13().low()
        });

        gpiob.crh.modify(|_, w| {
            w
            .mode13().output2()
            .cnf13().push_pull()
        });
    }

    {  //GPIO C
        let gpioc = unsafe { &*GPIOC::ptr() };
        gpioc.odr.modify(|_, w| {
            w.odr0().low()
            .odr1().high()
            .odr2().high()
            .odr3().high()
            .odr4().high()
            .odr5().low()
            .odr6().low()
            .odr7().low()
            .odr8().high()
            .odr9().high()
            .odr10().high()
            .odr11().high()
            .odr12().low()
            .odr13().high()
        });

        gpioc.crl.modify(|_, w| {
            w.mode0().output2()
            .cnf0().push_pull()
            .mode1().input()
            .cnf1().bits(INPUT_PULL_UP_DOWN)
            .mode2().output2()
            .cnf2().open_drain()
            .mode3().input()
            .cnf3().bits(INPUT_FLOAT)
            .mode4().output2()
            .cnf4().push_pull()
            .mode5().output2()
            .cnf5().push_pull()
            .mode6().output2()
            .cnf6().push_pull()
            .mode7().output2()
            .cnf7().push_pull()
        });

        gpioc.crh.modify(|_, w| {
            w.mode8().input()
            .cnf8().bits(INPUT_FLOAT)
            .mode9().input()
            .cnf9().bits(INPUT_FLOAT)
            .mode10().input()
            .cnf10().bits(INPUT_FLOAT)
            .mode11().input()
            .cnf11().bits(INPUT_FLOAT)
            .mode12().output50()
            .cnf12().push_pull()
            .mode13().input()
            .cnf13().bits(INPUT_PULL_UP_DOWN)
        });
    }

    {  //GPIO E
        let gpioe = unsafe { &*GPIOE::ptr() };
        gpioe.odr.modify(|_, w| {
            w
            .odr0().high()
        });

        gpioe.crl.modify(|_, w| {
            w
            .mode0().input()
            .cnf0().bits(INPUT_FLOAT)
        });
    }

    {  //GPIO F
        let gpiof = unsafe { &*GPIOF::ptr() };
        gpiof.odr.modify(|_, w| {
            w.odr0().high()
             .odr1().high()
             .odr2().high()
             .odr3().high()
             .odr4().high()
             .odr5().low()
             .odr6().low()
             .odr7().low()
             .odr8().low()
             .odr9().low()
             .odr10().low()
             .odr11().high()
             .odr12().high()
             .odr13().low()
             .odr14().high()
             .odr15().low()
        });

        gpiof.crl.modify(|_, w| {
            w.mode0().input()
             .cnf0().bits(INPUT_FLOAT)
             .mode1().input()
             .cnf1().bits(INPUT_FLOAT)
             .mode2().input()
             .cnf2().bits(INPUT_FLOAT)
             .mode3().input()
             .cnf3().bits(INPUT_FLOAT)
             .mode4().input()
             .cnf4().bits(INPUT_FLOAT)
             .mode5().input()
             .cnf5().bits(INPUT_PULL_UP_DOWN)
             .mode6().input()
             .cnf6().bits(INPUT_ANALOG)
             .mode7().input()
             .cnf7().bits(INPUT_ANALOG)
        });

        gpiof.crh.modify(|_, w| {
            w.mode8().input()
             .cnf8().bits(INPUT_ANALOG)
             .mode9().input()
             .cnf9().bits(INPUT_ANALOG)
             .mode10().input()
             .cnf10().bits(INPUT_FLOAT)
             .mode11().input()
             .cnf11().bits(INPUT_PULL_UP_DOWN)
             .mode12().input()
             .cnf12().bits(INPUT_FLOAT)
             .mode13().output2()
             .cnf13().push_pull()
             .mode14().input()
             .cnf14().bits(INPUT_FLOAT)
             .mode15().output2()
             .cnf15().push_pull()
        });
    }


    {  //GPIO G

        let gpiog = unsafe { &*GPIOG::ptr() };
        gpiog.odr.modify(|_, w| {
            w.odr0().high()
             .odr1().high()
             .odr2().high()
             .odr3().high()
             .odr4().high()
             .odr5().high()
             .odr6().low()
             .odr7().high()
             .odr8().high()
             .odr9().high()
             .odr10().high()
             .odr11().high()
             .odr12().high()
             .odr13().low()
             .odr14().high()
             .odr15().high()
        });

        gpiog.crl.modify(|_, w| {
            w.mode0().output2()
             .cnf0().open_drain()
             .mode1().output2()
             .cnf1().open_drain()
             .mode2().input()
             .cnf2().bits(INPUT_FLOAT)
             .mode3().input()
             .cnf3().bits(INPUT_PULL_UP_DOWN)
             .mode4().output2()
             .cnf4().open_drain()
             .mode5().output2()
             .cnf5().open_drain()
             .mode6().output2()
             .cnf6().open_drain()
             .mode7().output2()
             .cnf7().open_drain()
        });

        gpiog.crh.modify(|_, w| {
            w.mode8().input()
             .cnf8().bits(INPUT_PULL_UP_DOWN)
             .mode9().input()
             .cnf9().bits(INPUT_PULL_UP_DOWN)
             .mode10().output2()
             .cnf10().push_pull()
             .mode11().input()
             .cnf11().bits(INPUT_FLOAT)
             .mode12().input()
             .cnf12().bits(INPUT_FLOAT)
             .mode13().input()
             .cnf13().bits(INPUT_FLOAT)
             .mode14().input()
             .cnf14().bits(INPUT_FLOAT)
             .mode15().output2()
             .cnf15().open_drain()
        });

    }

   //TODO: registers 
}

#[rustfmt::skip]
pub fn fsmc_init() {
    use stm32f1xx_hal::pac::{
        RCC,
        FSMC,
        GPIOB,
        GPIOD,
        GPIOE,
    };
    
    let rcc = unsafe { &*RCC::ptr() };

    rcc.ahbenr.modify(|_, w| w.fsmcen().enabled());
    let fsmc = unsafe { &*FSMC::ptr() };
    fsmc.bcr1.modify(|_, w| {
        w.mbken().enabled()
         .muxen().enabled()
         .mtyp().flash()
         .mwid().bits16()
         .wren().enabled()
         .extmod().disabled()
    });
    fsmc.btr1.modify(|_, w| unsafe {
        w.addset().bits(12)
         .datast().bits(15)
         .addhld().bits(12)
         //.clkdiv().bits(1)
         //.busturn().bits(1)
         //.datlat().bits(0)
         .accmod().a()
    });

   //TODO: comments
    let gpiob = unsafe { &*GPIOB::ptr() };
    let gpiod = unsafe { &*GPIOD::ptr() };
    let gpioe = unsafe { &*GPIOE::ptr() };
    gpiob.crl.modify(|_, w| {
        w.mode7().output2()
         .cnf7().alt_push_pull()
    });
    gpiod.crh.modify(|_, w| {
        w.mode14().output()
         .cnf14().alt_push_pull()
         .mode15().output()
         .cnf15().alt_push_pull()
    });
    gpiod.crl.modify(|_, w| {
        w.mode0().output()
         .cnf0().alt_push_pull()
         .mode1().output()
         .cnf1().alt_push_pull()
    });
    gpioe.crl.modify(|_, w| {
        w.mode7().output()
         .cnf7().alt_push_pull()
    });
    gpioe.crh.modify(|_, w| {
        w.mode8().output()
         .cnf8().alt_push_pull()
         .mode9().output()
         .cnf9().alt_push_pull()
         .mode10().output()
         .cnf10().alt_push_pull()
         .mode11().output()
         .cnf11().alt_push_pull()
         .mode12().output()
         .cnf12().alt_push_pull()
         .mode13().output()
         .cnf13().alt_push_pull()
         .mode14().output()
         .cnf14().alt_push_pull()
         .mode15().output()
         .cnf15().alt_push_pull()
    });
    gpiod.crh.modify(|_, w| {
        w.mode8().output()
         .cnf8().alt_push_pull()
         .mode9().output()
         .cnf9().alt_push_pull()
         .mode10().output()
         .cnf10().alt_push_pull()
         .mode11().output()
         .cnf11().alt_push_pull()
         .mode12().output()
         .cnf12().alt_push_pull()
         .mode13().output()
         .cnf13().alt_push_pull()
    });
    gpioe.crl.modify(|_, w| {
        w.mode3().output()
         .cnf3().alt_push_pull()
         .mode4().output()
         .cnf4().alt_push_pull()
    });
    gpiod.crl.modify(|_, w| {
        w.mode4().output2()
         .cnf4().alt_push_pull()
         .mode5().output2()
         .cnf5().alt_push_pull()
         .mode7().output2()
         .cnf7().alt_push_pull()
    });
}
