use crate::{BufferType, Library};
use nvflex_sys::*;
use std::marker::PhantomData;
use std::mem::size_of;

pub struct Buffer<T> {
    pub(crate) buf: *mut NvFlexBuffer,
    phantom: PhantomData<T>,
}

impl<T> Buffer<T> {
    /// Allocate a Flex buffer. Buffers are used to pass data to the API in an efficient manner.
    ///
    /// The element byte stride is the size of `T`.
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    /// * `element_count` - The number of elements in the buffer
    /// * `typ` - The type of buffer to allocate, can be either host memory or device memory
    #[inline]
    pub fn new(lib: &Library, element_count: i32, typ: BufferType) -> Self {
        unsafe {
            Self {
                buf: NvFlexAllocBuffer(lib.lib, element_count, size_of::<T>() as _, typ as _),
                phantom: PhantomData,
            }
        }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl<T> Send for Buffer<T> {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl<T> Sync for Buffer<T> {}

impl<T> Drop for Buffer<T> {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexFreeBuffer(self.buf) }
    }
}
