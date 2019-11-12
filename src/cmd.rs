use crate::rtt_print;
use ellocopo::Error;
use crate::pld::{PldPWMComp, PldPWMValve, PldAfeCmd_low, PldAfeCmd_high};

pub mod built_info {
   include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

static mut INF_CUR : u8 = 0x20;
static mut RED_CUR : u8 = 0x20;
static mut ECHO_BUF : [u8;0x40] = [0;0x40];
static mut ECHO_BUF_SZ : usize = 0;

#[no_mangle]
pub fn cb_erase_hard() -> Result<(), Error> {
    Ok(())
}

#[no_mangle]
pub fn cb_write_hard_comp(v : u16) -> Result<(), Error> {
    PldPWMComp::get().write(v);
    Ok(())
}

#[no_mangle]
pub fn cb_read_hard_comp() -> Result<u16, Error> {
    Ok(PldPWMComp::get().read())
}

#[no_mangle]
pub fn cb_write_hard_valv(v : u16) -> Result<(), Error> {
    PldPWMValve::get().write(v);
    Ok(())
}

#[no_mangle]
pub fn cb_read_hard_valv() -> Result<u16, Error> {
    Ok(PldPWMValve::get().read())
}

#[no_mangle]
pub fn cb_write_hard_red(v : u8) -> Result<(), Error> {
    unsafe { RED_CUR = v; }
    let tmp : u16 = unsafe { ((RED_CUR as u16) << 8) | INF_CUR as u16 };
    PldAfeCmd_low::get().write(tmp);
    PldAfeCmd_high::get().write(0x2202);
    Ok(())
}

#[no_mangle]
pub fn cb_read_hard_red() -> Result<u8, Error> {
    Ok(unsafe { RED_CUR })
}

#[no_mangle]
pub fn cb_write_hard_inf(v : u8) -> Result<(), Error> {
    unsafe { INF_CUR = v; }
    let tmp : u16 = unsafe { ((RED_CUR as u16) << 8) | INF_CUR as u16 };
    PldAfeCmd_low::get().write(tmp);
    PldAfeCmd_high::get().write(0x2202);
    Ok(())
}

#[no_mangle]
pub fn cb_read_hard_inf() -> Result<u8, Error> {
    Ok(unsafe { INF_CUR })
}

#[no_mangle]
pub fn cb_erase_info() -> Result<(), Error> {
    Ok(())
}

#[no_mangle]
pub fn cb_write_info_version(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_info_version() -> Result<&'static str, Error> {
    Ok(built_info::PKG_VERSION)
}

#[no_mangle]
pub fn cb_erase_build() -> Result<(), Error> {
    Ok(())
}

#[no_mangle]
pub fn cb_write_build_version(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_version() -> Result<&'static str, Error> {
     Ok(built_info::PKG_VERSION)
}

#[no_mangle]
pub fn cb_write_build_compiler(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_compiler() -> Result<&'static str, Error> {
     Ok(built_info::RUSTC_VERSION)
}

#[no_mangle]
pub fn cb_write_build_git(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_git() -> Result<&'static str, Error> {
    Ok(built_info::GIT_VERSION.unwrap_or("None"))
}

#[no_mangle]
pub fn cb_write_build_time(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_time() -> Result<&'static str, Error> {
    Ok(built_info::BUILT_TIME_UTC)
}

#[no_mangle]
pub fn cb_write_build_target(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_target() -> Result<&'static str, Error> {
    Ok(built_info::TARGET)
}

#[no_mangle]
pub fn cb_write_build_host(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_host() -> Result<&'static str, Error> {
    Ok(built_info::HOST)
}

#[no_mangle]
pub fn cb_write_build_profile(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_profile() -> Result<&'static str, Error> {
    Ok(built_info::PROFILE)
}

#[no_mangle]
pub fn cb_write_build_authors(_v : &'static str) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_build_authors() -> Result<&'static str, Error> {
    Ok(built_info::PKG_AUTHORS)
}

#[no_mangle]
pub fn cb_erase_test() -> Result<(), Error> {
    Ok(())
}

#[no_mangle]
pub fn cb_write_test_echo(v : &'static [u8]) -> Result<(), Error> {
    unsafe {
        ECHO_BUF_SZ = v.len();
        (&mut ECHO_BUF[..ECHO_BUF_SZ]).copy_from_slice(v);
    }
    Ok(())
}

#[no_mangle]
pub fn cb_read_test_echo() -> Result<&'static [u8], Error> {
    unsafe { Ok(&ECHO_BUF[..ECHO_BUF_SZ]) }
}

#[no_mangle]
pub fn cb_write_test_test1(_v : &'static [u8]) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_test_test1() -> Result<&'static [u8], Error> {
    Ok(b"\"Oh, you can't help that,\" said the Cat:")
}

#[no_mangle]
pub fn cb_write_test_test2(_v : &'static [u8]) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_test_test2() -> Result<&'static [u8], Error> {
    Ok(b"\"we're all mad here. I'm mad. You're mad.\"")
}

#[no_mangle]
pub fn cb_write_test_test3(_v : &'static [u8]) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_test_test3() -> Result<&'static [u8], Error> {
    Ok(b"\"How do you know I'm mad?\" said Alice.")
}

#[no_mangle]
pub fn cb_write_test_test4(_v : &'static [u8]) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_test_test4() -> Result<&'static [u8], Error> {
    Ok(b"\"You must be,\" said the Cat,")
}

#[no_mangle]
pub fn cb_write_test_test5(_v : &'static [u8]) -> Result<(), Error> {
    Err(Error::NonWriteable)
}

#[no_mangle]
pub fn cb_read_test_test5() -> Result<&'static [u8], Error> {
    Ok(b"or you wouldn't have come here.\"")
}