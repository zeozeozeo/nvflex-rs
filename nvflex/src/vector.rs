use nvflex_sys::*;
use std::{
    ops::{Index, IndexMut},
    sync::atomic::AtomicPtr,
};

/// Controls memory space of [`FlexVec`]'s underlying [`NvFlexBuffer`], see [`NvFlexAllocBuffer()`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BufferType {
    /// A host mappable buffer, pinned memory on CUDA, staging buffer on DX.
    Host = 0,
    /// A device memory buffer, device memory pointer on CUDA, buffer pointer on DX.
    Device = 1,
}

impl Into<NvFlexBufferType> for BufferType {
    #[inline]
    fn into(self) -> NvFlexBufferType {
        match self {
            BufferType::Host => eNvFlexBufferHost,
            BufferType::Device => eNvFlexBufferDevice,
        }
    }
}

/// Controls behavior of [`NvFlexMap()`].
pub enum MapFlags {
    /// Calling thread will be blocked until buffer is ready for access, default.
    Wait = 0,
    /// Calling thread will check if buffer is ready for access, if not ready then the method will return NULL immediately.
    DoNotWait = 1,
}

impl Into<NvFlexMapFlags> for MapFlags {
    #[inline]
    fn into(self) -> NvFlexMapFlags {
        match self {
            MapFlags::Wait => eNvFlexMapWait,
            MapFlags::DoNotWait => eNvFlexMapDoNotWait,
        }
    }
}

/// A vector type which wraps [`NvFlexBuffer`]. It will map the underlying FleX buffer before any read/write access to elements
/// or a resize operation occurs, but it won't unmap it (except for constructors and when the buffer is dropped).
/// Calling any constructor guarantees that the buffer is unmapped.
///
/// This is similar to `NvFlexExt`'s [`NvFlexVector`].
pub struct FlexVec<T: Clone> {
    /// Pointer to the underlying [`NvFlexBuffer`].
    buffer: AtomicPtr<NvFlexBuffer>,
    /// Pointer to [`NvFlexLibrary`].
    lib: AtomicPtr<NvFlexLibrary>,
    /// Total capacity of the buffer. Changing the capacity requires reallocating the FleX buffer.
    capacity: i32,
    /// Amount of elements inside the buffer. This is different from the buffer capacity.
    len: i32,
    /// Controls memory space of the underlying [`NvFlexBuffer`].
    buffer_type: BufferType,
    /// Pointer to the mapped buffer. If the buffer is not mapped, this will be NULL.
    mapped_buf: AtomicPtr<T>,
    /// Phantom data to make sure we own `T`.
    phantom: std::marker::PhantomData<T>,
}

impl<T: Clone> FlexVec<T> {
    pub fn with_capacity(lib: *mut NvFlexLibrary, buffer_type: BufferType, capacity: i32) -> Self {
        let mut vec = Self {
            buffer: AtomicPtr::new(std::ptr::null_mut()),
            lib: AtomicPtr::new(lib),
            capacity: 0, // `set_capacity()` will set this
            len: 0,
            buffer_type,
            mapped_buf: AtomicPtr::new(std::ptr::null_mut()),
            phantom: std::marker::PhantomData, // pretend like we own `T`
        };

        vec.set_capacity(capacity);
        // `FlexVec::set_capacity()` may leave the buffer mapped, we have make sure to unmap it
        vec.unmap(); // will do nothing if the buffer is not mapped

        vec
    }

    #[inline]
    pub unsafe fn lib_ptr(&self) -> *mut NvFlexLibrary {
        *self.lib.as_ptr()
    }

    #[inline]
    pub unsafe fn mapped_buf_ptr(&self) -> *mut T {
        *self.mapped_buf.as_ptr()
    }

    #[inline]
    pub unsafe fn buf_ptr(&self) -> *mut NvFlexBuffer {
        *self.buffer.as_ptr()
    }

    #[inline]
    pub unsafe fn alloc_buffer(&self, length: i32) -> *mut NvFlexBuffer {
        NvFlexAllocBuffer(
            self.lib_ptr(),
            length,
            std::mem::size_of::<T>() as _,
            self.buffer_type.into(),
        )
    }

    #[inline]
    pub fn map(&mut self, flags: MapFlags) {
        unsafe {
            let buf_ptr = self.buf_ptr();
            if !self.mapped_buf_ptr().is_null() || buf_ptr.is_null() {
                return;
            }

            self.mapped_buf = AtomicPtr::new(NvFlexMap(buf_ptr, flags.into()) as _);
        }
    }

    #[inline]
    pub fn unmap(&mut self) {
        unsafe {
            let buf_ptr = self.buf_ptr();
            if self.mapped_buf_ptr().is_null() || buf_ptr.is_null() {
                return;
            }

            NvFlexUnmap(buf_ptr);
            self.mapped_buf = AtomicPtr::new(std::ptr::null_mut());
        }
    }

    pub fn set_capacity(&mut self, capacity: i32) {
        if self.capacity == capacity {
            return;
        }
        unsafe {
            let new_buf = self.alloc_buffer(capacity);

            // copy old buffer contents into the new buffer if it exists
            let mapped_ptr = self.mapped_buf_ptr();
            let new_ptr = if !mapped_ptr.is_null() {
                let new_ptr = NvFlexMap(new_buf, eNvFlexMapWait);
                std::ptr::copy_nonoverlapping(
                    new_ptr,
                    mapped_ptr as _,
                    (self.len as usize) * std::mem::size_of::<T>(),
                );
                new_ptr
            } else {
                std::ptr::null_mut()
            };

            // unmap old buffer and free it
            self.unmap();
            {
                let old_buf_ptr = self.buf_ptr();
                if !old_buf_ptr.is_null() {
                    NvFlexFreeBuffer(old_buf_ptr);
                }
            }

            // swap buffers
            self.buffer = AtomicPtr::new(new_buf);
            self.mapped_buf = AtomicPtr::new(new_ptr as _);
            self.capacity = capacity;
        }
    }

    #[inline]
    pub fn reserve(&mut self, additional: i32) {
        self.set_capacity(self.capacity + additional);
    }

    /// Shortens the vector, keeping the first `new_len` elements and dropping
    /// the rest. Note that this has no effect on the allocated capacity of the vector.
    #[inline]
    pub fn truncate(&mut self, new_len: i32) {
        if new_len > self.len || new_len < 0 {
            return;
        }
        self.len = new_len;
    }

    #[inline]
    fn grow_if_needed(&mut self, new_len: i32) {
        if new_len > self.capacity {
            self.set_capacity(new_len * 3 / 2); // growth factor of 1.5
        }
    }

    pub fn resize(&mut self, new_len: i32, value: T) {
        if new_len == self.len {
            return;
        }

        if new_len < self.len {
            self.truncate(new_len);
        } else {
            self.grow_if_needed(new_len);

            // initialize any new entries
            let old_len = self.len;

            // write all elements except the last one
            for i in old_len..new_len - 1 {
                self[i] = value.clone();
                self.len += 1; // increment length after every step in case clone() panics
            }

            // we can write the last element directly without cloning needlessly
            self[new_len - 1] = value;
            self.len += 1;
        }
    }

    #[inline]
    pub fn len(&self) -> i32 {
        self.len
    }

    #[inline]
    pub fn capacity(&self) -> i32 {
        self.capacity
    }

    /// Appends an element to the back of a collection.
    #[inline]
    pub fn push(&mut self, value: T) {
        self.resize(self.len + 1, value);
    }

    /// Removes the last element from a vector and returns it, or [`None`] if it
    /// is empty.
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            None
        } else {
            self.len -= 1;
            unsafe { Some(std::ptr::read(self.mapped_buf_ptr().add(self.len as _))) }
        }
    }

    /// Moves all the elements of `other` into `self`, leaving `other` empty.
    #[inline]
    pub fn append(&mut self, other: &mut Self) {
        let other_len = other.len();
        let self_len = self.len;
        self.grow_if_needed(self_len + other_len);
        unsafe {
            std::ptr::copy_nonoverlapping(
                other.mapped_buf_ptr(),
                self.mapped_buf_ptr().add(self_len as _),
                other_len as _,
            );
        };
        other.truncate(0);
    }

    #[inline]
    unsafe fn check_index(&self, index: i32) {
        let mapped_ptr = self.mapped_buf_ptr();
        if mapped_ptr.is_null() {
            panic!("buffer is not mapped");
        }
        if index >= self.len {
            panic!(
                "index out of bounds: attempted to access value at index {} but vector length is {}",
                index, self.len
            );
        }
    }
}

impl<T: Clone> Index<i32> for FlexVec<T> {
    type Output = T;
    fn index(&self, index: i32) -> &Self::Output {
        unsafe {
            self.check_index(index);
            &*self.mapped_buf_ptr().add(index as _)
        }
    }
}

impl<T: Clone> IndexMut<i32> for FlexVec<T> {
    fn index_mut(&mut self, index: i32) -> &mut T {
        unsafe {
            self.check_index(index);
            &mut *self.mapped_buf_ptr().add(index as _)
        }
    }
}

impl<T: Clone> Drop for FlexVec<T> {
    fn drop(&mut self) {
        unsafe {
            if !self.mapped_buf_ptr().is_null() {
                NvFlexUnmap(self.buf_ptr());
            }
            if !self.buf_ptr().is_null() {
                NvFlexFreeBuffer(self.buf_ptr());
            }

            self.capacity = 0;
            self.len = 0;
        }
    }
}
