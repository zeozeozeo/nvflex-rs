use crate::{BufferType, Library, MapFlags};
use nvflex_sys::*;
use std::{
    marker::PhantomData,
    mem::size_of,
    ops::{Index, IndexMut},
};

/// A vector type that wraps a [`Buffer`], behaves like a standard vector for POD types (no construction)
///
/// The vector must be mapped using `map()` before any read/write access to elements or resize operation
#[derive(Debug)]
pub struct Vector<T: Clone> {
    pub(crate) lib: *mut NvFlexLibrary,
    pub(crate) buffer: *mut NvFlexBuffer,
    pub(crate) mapped_ptr: *mut T,
    pub(crate) count: i32,
    pub(crate) capacity: i32,
    pub(crate) typ: BufferType,
    phantom: PhantomData<T>,
}

impl<T: Clone> Vector<T> {
    /// # Parameters
    ///
    /// * `size` - Usually 0
    /// * `typ` - Usually [`BufferType::Host`]
    pub fn new(l: &Library, size: i32, typ: BufferType) -> Self {
        let v = Self {
            lib: l.lib,
            buffer: std::ptr::null_mut(),
            mapped_ptr: std::ptr::null_mut(),
            count: 0,
            capacity: 0,
            typ,
            phantom: PhantomData,
        };
        if size != 0 {}
        v
    }

    /// Reinitialize the vector leaving it unmapped
    #[inline]
    pub fn init(&mut self, size: i32) {
        self.destroy();
        self.resize(size);
        self.unmap();
    }

    #[inline]
    pub fn push_back(&mut self, t: T) {
        assert!(!self.mapped_ptr.is_null() || self.buffer.is_null());

        self.reserve(self.count + 1);

        // copy element
        unsafe {
            self.mapped_ptr.add(self.count as _).write(t);
        }
        self.count += 1;
    }

    #[inline]
    pub unsafe fn assign(&mut self, src_ptr: *const T, new_count: i32) {
        assert!(!self.mapped_ptr.is_null() || self.buffer.is_null());

        self.resize(new_count);

        std::ptr::copy_nonoverlapping(src_ptr, self.mapped_ptr, new_count as _);
    }

    #[inline]
    pub unsafe fn copyto(&mut self, dest: *mut T, count: i32) {
        assert!(!self.mapped_ptr.is_null());

        std::ptr::copy_nonoverlapping(self.mapped_ptr, dest, count as _);
    }

    #[inline]
    pub fn size(&self) -> i32 {
        self.count
    }

    #[inline]
    pub fn len(&self) -> i32 {
        self.count
    }

    #[inline]
    pub fn empty(&self) -> bool {
        self.count == 0
    }

    #[inline]
    pub fn back(&self) -> &T {
        assert!(!self.mapped_ptr.is_null());
        assert!(!self.empty());

        unsafe {
            &*self.mapped_ptr.add(self.count as usize - 1) // TODO
        }
    }

    /// Parameters
    ///
    /// * `flags` - Usually [`MapFlags::Wait`]
    pub fn map(&mut self, flags: MapFlags) {
        unsafe {
            if self.buffer.is_null() {
                return;
            }

            assert!(self.mapped_ptr.is_null());
            self.mapped_ptr = NvFlexMap(self.buffer, flags as _) as *mut T;
        }
    }

    pub fn unmap(&mut self) {
        unsafe {
            if self.buffer.is_null() {
                return;
            }

            assert!(!self.mapped_ptr.is_null());

            NvFlexUnmap(self.buffer);
            self.mapped_ptr = std::ptr::null_mut();
        }
    }

    pub fn reserve(&mut self, min_capacity: i32) {
        unsafe {
            if min_capacity > self.capacity {
                // growth factor of 1.5
                let new_capacity = min_capacity * 3 / 2;

                let new_buf =
                    NvFlexAllocBuffer(self.lib, new_capacity, size_of::<T>() as _, self.typ as _);

                // copy contents to new buffer
                let new_ptr = NvFlexMap(new_buf, MapFlags::Wait as _) as *mut T;
                std::ptr::copy_nonoverlapping(self.mapped_ptr, new_ptr, self.count as _);

                // unmap old buffer, but leave new buffer mapped
                self.unmap();

                if !self.buffer.is_null() {
                    NvFlexFreeBuffer(self.buffer);
                }

                // swap
                self.buffer = new_buf;
                self.mapped_ptr = new_ptr;
                self.capacity = new_capacity;
            }
        }
    }

    #[inline]
    pub fn resize(&mut self, new_count: i32) {
        assert!(!self.mapped_ptr.is_null() || self.buffer.is_null());

        self.reserve(new_count);

        // resize but do not initialize new entries
        self.count = new_count;
    }

    #[inline]
    pub fn resize_with(&mut self, new_count: i32, val: T) {
        assert!(!self.mapped_ptr.is_null() || self.buffer.is_null());

        let start_init = self.count;
        let end_init = new_count;

        self.resize(new_count);

        // init any new entries
        unsafe {
            for i in start_init..end_init - 1 {
                self.mapped_ptr.add(i as _).write(val.clone());
            }

            // don't clone the value needlessly for the last iteration
            self.mapped_ptr.add(end_init as usize - 1).write(val);
        }
    }

    pub fn destroy(&mut self) {
        unsafe {
            if !self.mapped_ptr.is_null() {
                NvFlexUnmap(self.buffer);
            }
            if !self.buffer.is_null() {
                NvFlexFreeBuffer(self.buffer);
            }

            self.mapped_ptr = std::ptr::null_mut();
            self.buffer = std::ptr::null_mut();
            self.capacity = 0;
            self.count = 0;
        }
    }
}

impl<T: Clone> Index<usize> for Vector<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        assert!(!self.mapped_ptr.is_null());
        assert!(index < self.count as _);

        unsafe { &*self.mapped_ptr.add(index) }
    }
}

impl<T: Clone> IndexMut<usize> for Vector<T> {
    fn index_mut<'a>(&'a mut self, index: usize) -> &'a mut T {
        assert!(!self.mapped_ptr.is_null());
        assert!(index < self.count as _);

        unsafe { &mut *self.mapped_ptr.add(index) }
    }
}

impl<T: Clone> Drop for Vector<T> {
    #[inline]
    fn drop(&mut self) {
        self.destroy();
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl<T> Send for Vector<T> {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl<T> Sync for Vector<T> {}
