#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy)]
pub struct AdcFrame {
    pub chL :  i32,
    pub chF :  i32,
    pub ch1 :  i32,
    pub ch2 :  i32,
    pub ch3 :  i32,
    pub ch4 :  i32,
    pub ch5 :  i32,
    pub ch6 :  i32,
    pub reo0 : i32,
    pub reo1 : i32,
    pub brth : i32,
    pub ad :   i32,
    pub mic1 : i32,
    pub Red :  i32,
    pub Ir :   i32,
    pub nu :   i32,
    pub cnt :  u32,
    pub sec :  u32,
    pub tick : u16,
    pub nu2 :  u16,
}

impl AdcFrame {
    pub const fn new() -> Self {
        AdcFrame {
            chL :  0,
            chF :  0,
            ch1 :  0,
            ch2 :  0,
            ch3 :  0,
            ch4 :  0,
            ch5 :  0,
            ch6 :  0,
            reo0 : 0,
            reo1 : 0,
            brth : 0,
            ad :   0,
            mic1 : 0,
            Red :  0,
            Ir :   0,
            nu :   0,
            cnt :  0,
            sec :  0,
            tick : 0,
            nu2 :  0,
        }
    }

    // TODO: untested!
     pub fn as_bytes(&self) -> &[u8] {
         use core::mem::size_of;
         use core::mem::transmute;
         let ptr : &u8 = unsafe { transmute(self) };
         unsafe { core::slice::from_raw_parts(ptr, size_of::<Self>()) }
     }
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy)]
pub struct AfeFrame {
    pub ir :    i32,
    pub bg_ir : i32,
    pub rd :    i32,
    pub bg_rd : i32,
    pub cnt :   u32,
    pub sec :   u32,
    pub tick :  u16,
    pub nu2 :   u16,
}

impl AfeFrame {
    pub const fn new() -> Self {
        AfeFrame {
            ir :    0,
            bg_ir : 0,
            rd :    0,
            bg_rd : 0,
            cnt :   0,
            sec :   0,
            tick :  0,
            nu2 :   0,
        }
    }

    // TODO: untested!
     pub fn as_bytes(&self) -> &[u8] {
         use core::mem::size_of;
         use core::mem::transmute;
         let ptr : &u8 = unsafe { transmute(self) };
         unsafe { core::slice::from_raw_parts(ptr, size_of::<Self>()) }
     }
}