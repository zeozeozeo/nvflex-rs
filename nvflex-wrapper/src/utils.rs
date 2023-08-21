#[macro_export]
macro_rules! rstr {
    ($cstring:expr) => {{
        #[allow(unused_unsafe)]
        unsafe { std::ffi::CStr::from_ptr($cstring) }
            .to_str()
            .expect("unable to convert C string")
    }};
}
