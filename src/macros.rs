#![allow(unused_macros)]

macro_rules! register_u16_rw {
    ($reg_name: ident, $reg_address : literal ) => {
        #[repr(C)]
        pub struct $reg_name {
            r: RW<u16>,
        }

        impl $reg_name {
            pub fn get() -> &'static mut $reg_name {
                unsafe { &mut *($reg_address as *mut $reg_name) }
            }
            pub fn read(&mut self) -> u16 {
                self.r.read()
            }
            pub fn write(&mut self, bb: u16) {
                unsafe { self.r.write(bb) };
            }
        }
    };
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(WouldBlock) {
                break res;
            }
            if $exit_cond {
                break res;
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        busy_wait!(
            $nb_expr,
            DWT::get_cycle_count().wrapping_sub(started) >= cycles
        )
    }};
    ($cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        loop {
            if DWT::get_cycle_count().wrapping_sub(started) >= cycles {
                break;
            }
        }
    }};
}
