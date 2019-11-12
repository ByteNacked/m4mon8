mod monitor;

use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use stm32_usbd::{UsbBus, UsbBusType};
use stm32f1xx_hal::pac::Peripherals;
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::{gpio::gpioa, gpio::gpioc, prelude::*, rcc::Clocks};
use usb_device::{bus::UsbBusAllocator, prelude::*};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_MONITOR: Option<monitor::MonitorDev<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;
const VID: u16 = 0x0483;
const PID: u16 = 0x7503;

pub fn usb_init(clocks: &Clocks) -> () {
    assert!(clocks.usbclk_valid());

    let p = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { Peripherals::steal() };

    let mut gpioa = unsafe { dp.GPIOA.steal() };
    let mut gpioc = unsafe { dp.GPIOC.steal() };

    let mut usb_pw = gpioc.pc2.into_push_pull_output(&mut gpioc.crl);
    let usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    let _ = usb_pw.set_high();
    //usb_dp.set_low();
    delay(clocks.sysclk().0 / 100);
    let _ = usb_pw.set_low();

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    unsafe {
        let bus = UsbBus::new(dp.USB, (usb_dm, usb_dp));

        USB_BUS = Some(bus);

        USB_MONITOR = Some(monitor::MonitorDev::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(VID, PID))
            .manufacturer("Incart")
            .product("Monitor")
            .serial_number("TEST")
            .max_packet_size_0(64)
            .build();

        USB_DEVICE = Some(usb_dev);
    }

    let mut nvic = p.NVIC;

    nvic.enable(Interrupt::USB_HP_CAN_TX);
    nvic.enable(Interrupt::USB_LP_CAN_RX0)
}

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

static mut USB_IRQ_CNT: u32 = 0;
static mut CMD_PROC: ellocopo::Processor = ellocopo::Processor::new();
fn usb_interrupt() {
    unsafe {
        USB_IRQ_CNT += 1;
    }
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let monitor = unsafe { USB_MONITOR.as_mut().unwrap() };

    if !usb_dev.poll(&mut [monitor]) {
        return;
    }

    let mut buf = [0u8; 0x40];

    match monitor.read(&mut buf) {
        Ok(count) if count > 0 => {
            let p = unsafe { &mut CMD_PROC };
            let count = match p.process_try_answer(&mut buf, count) {
                Ok(c) => c,
                Err(_) => panic!(),
            };
            monitor.write(&buf[0..count]).ok();
        }
        _ => {}
    }

    use crate::VB;
    match unsafe { VB.dequeue_slice(&mut buf) } {
        n => {
            let _ = monitor.vis(&buf[..n]);
        }
    }

}
