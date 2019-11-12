use crate::rtt_print;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const TPS_ADDR: u8 = 0x90 >> 1;

pub struct Tps<T: 'static> {
    d: &'static mut T,
}

pub enum PowerState {
    LowPower,
    FullPower,
}

impl<T> Tps<T>
where
    T: Read + Write + WriteRead,
{
    pub fn new(d: &'static mut T) -> Self {
        Tps { d }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        let mut buf = [0u8; 1];

        let tr = [0x10, 0x31];
        let _ = self.d.write(TPS_ADDR, &tr);
        let _ = self.d.write_read(TPS_ADDR, &tr[..1], &mut buf);
        rtt_print!("reg {:X} val {:X}", tr[0], buf[0]);
        assert_eq!(tr[1], buf[0]);

        let tr = [0x12, 0x3F];
        let _ = self.d.write(TPS_ADDR, &tr);
        let _ = self.d.write_read(TPS_ADDR, &tr[..1], &mut buf);
        rtt_print!("reg {:X} val {:X}", tr[0], buf[0]);
        assert_eq!(tr[1], buf[0]);

        let tr = [0x14, 0x1F];
        let _ = self.d.write(TPS_ADDR, &tr);
        let _ = self.d.write_read(TPS_ADDR, &tr[..1], &mut buf);
        rtt_print!("reg {:X} val {:X}", tr[0], buf[0]);
        assert_eq!(tr[1], buf[0]);

        let tr = [0x17, 0x3F];
        let _ = self.d.write(TPS_ADDR, &tr);
        let _ = self.d.write_read(TPS_ADDR, &tr[..1], &mut buf);
        rtt_print!("reg {:X} val {:X}", tr[0], buf[0]);
        assert_eq!(tr[1], buf[0]);

        let tr = [0x04, 0x30];
        let _ = self.d.write(TPS_ADDR, &tr);
        let _ = self.d.write_read(TPS_ADDR, &tr[..1], &mut buf);
        rtt_print!("reg {:X} val {:X}", tr[0], buf[0]);
        assert_eq!(tr[1], buf[0]);

        Ok(())
    }
}

mod util {
    trait Rev16 {
        fn rev16(self) -> [u8; 2];
    }

    impl Rev16 for u16 {
        fn rev16(self) -> [u8; 2] {
            [(self >> 8) as u8, (self << 8) as u8]
        }
    }
}
