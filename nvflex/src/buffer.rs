use crate::{BufferType, Library, MapFlags};
use nvflex_sys::*;
use std::ffi::c_void;
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
    pub fn new(lib: Library, element_count: i32, typ: BufferType) -> Self {
        unsafe {
            Self {
                buf: NvFlexAllocBuffer(lib.lib, element_count, size_of::<T>() as _, typ as _),
                phantom: PhantomData,
            }
        }
    }

    /// Maps a buffer for reading and writing. When the buffer is created with NvFlexBufferType::eHost, then the returned pointer will be a host memory address
    /// that can be read/written.
    /// Mapping a buffer implicitly synchronizes with the GPU to ensure that any reads or writes from the buffer (e.g.: from the NvFlexGet*() or NvFlexSet*() methods) have completed.
    ///
    /// # Parameters
    ///
    /// * `flags` - Hints to Flex how the buffer is to be accessed, typically this should be [`MapFlags::Wait`]
    ///
    /// # Returns
    ///
    /// A pointer to the mapped memory
    #[inline]
    pub fn map(&self, flags: MapFlags) -> *mut T {
        unsafe { NvFlexMap(self.buf, flags as _) as *mut T }
    }

    /// Unmaps a buffer that was mapped through `map()`, note that buffers must be unmapped before they can be passed to a NvFlexGet*() or NvFlexSet*() method
    #[inline]
    pub fn unmap(&self) {
        unsafe { NvFlexUnmap(self.buf) };
    }

    // TODO: translate the C code example to Rust
    /// Registers an OpenGL buffer to Flex which can be used to copy directly into a graphics resource. Example usage is below
    ///
    /// ```c
    /// GLuint vbo;
    /// glGenBuffers(1, &vbo);
    /// glBindBuffer(GL_ARRAY_BUFFER, vbo);
    /// glBufferData(GL_ARRAY_BUFFER, size, NULL, GL_DYNAMIC_DRAW)
    ///
    /// NvFlexBuffer* vboBuffer = NvFlexRegisterOGLBuffer(lib, vbo, n, sizeof(float)*4);
    ///
    /// // simulate
    /// ...
    ///
    /// // copy directly from Flex into render buffer
    /// NvFlexGetParticles(vboBuffer, n);
    ///
    /// // render
    /// ...
    /// ```
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    /// * `buf` - An OpenGL buffer identifier
    /// * `element_count` - The number of elements in the buffer
    ///
    /// # Returns
    ///
    /// A valid [`Buffer`] instance that may be used with NvFlexGet*() methods to populate the render buffer using direct GPU-GPU copies
    #[inline]
    pub fn register_ogl_buffer(lib: Library, buf: i32, element_count: i32) -> Self {
        unsafe {
            Self {
                buf: NvFlexRegisterOGLBuffer(lib.lib, buf, element_count, size_of::<T>() as _),
                phantom: PhantomData,
            }
        }
    }

    /// Unregister a [`Buffer`] allocated through `register_ogl_buffer()`
    #[inline]
    pub fn unregister_ogl_buffer(&self) {
        unsafe { NvFlexUnregisterOGLBuffer(self.buf) };
    }

    /// Registers a Direct3D buffer to Flex which can be used to copy directly into a graphics resource
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    /// * `buffer` - A pointer to either an ID3D11Buffer or ID3D12Resource object
    /// * `element_count` - The number of elements in the buffer
    ///
    /// # Returns
    ///
    ///  A valid [`Buffer`] instance that may be used with NvFlexGet*() methods to populate the render buffer using direct GPU-GPU copies
    #[inline]
    pub fn register_d3d_buffer(lib: Library, buffer: *mut c_void, element_count: i32) -> Self {
        unsafe {
            Self {
                buf: NvFlexRegisterD3DBuffer(lib.lib, buffer, element_count, size_of::<T>() as _),
                phantom: PhantomData,
            }
        }
    }

    /// Unregister a [`Buffer`] allocated through `register_d3d_buffer()`
    #[inline]
    pub fn unregister_d3d_buffer(&self) {
        unsafe { NvFlexUnregisterD3DBuffer(self.buf) };
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
