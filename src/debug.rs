#[rustfmt::skip]
#[macro_export]
macro_rules! dbg_info {
    () => {
        concat!("file: ", file!(), " line : ", line!(), " column : ", column!())
    };
}
