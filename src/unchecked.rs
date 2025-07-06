use std::ops::{Index, IndexMut, Range};

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UncheckedArray<T, const N: usize>(pub [T; N]);

impl<T: Default + Copy, const N: usize> Default for UncheckedArray<T, N> {
    fn default() -> Self {
        Self([T::default(); N])
    }
}

impl<T, const N: usize> UncheckedArray<T, N> {
    pub const fn from_array(array: [T; N]) -> Self {
        Self(array)
    }

    #[inline(always)]
    pub fn as_ptr(&self) -> *const T {
        self.0.as_ptr()
    }

    #[inline(always)]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.0.as_mut_ptr()
    }

    pub fn len(&self) -> usize {
        N
    }
}

impl<T, const N: usize> Index<usize> for UncheckedArray<T, N> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { self.0.get_unchecked(index) }
    }
}

impl<T, const N: usize> Index<Range<usize>> for UncheckedArray<T, N> {
    type Output = UncheckedSlice<T>;

    #[inline(always)]
    fn index(&self, index: Range<usize>) -> &Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { std::mem::transmute(self.0.get_unchecked(index.start..index.end)) }
    }
}

impl<T, const N: usize> IndexMut<usize> for UncheckedArray<T, N> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { self.0.get_unchecked_mut(index) }
    }
}

impl<T, const N: usize> IndexMut<Range<usize>> for UncheckedArray<T, N> {
    #[inline(always)]
    fn index_mut(&mut self, index: Range<usize>) -> &mut Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { std::mem::transmute(self.0.get_unchecked_mut(index.start..index.end)) }
    }
}

impl<T, const N: usize> AsRef<UncheckedSlice<T>> for UncheckedArray<T, N> {
    #[inline(always)]
    fn as_ref(&self) -> &UncheckedSlice<T> {
        unsafe { std::mem::transmute(&self.0[..]) }
    }
}

#[repr(transparent)]
pub struct UncheckedSlice<T>([T]);

impl<T> UncheckedSlice<T> {
    #[inline(always)]
    pub fn inner_slice(&self) -> &[T] {
        &self.0
    }

    #[inline(always)]
    pub fn inner_slice_mut(&mut self) -> &mut [T] {
        &mut self.0
    }
}

impl<T> Index<usize> for UncheckedSlice<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { self.0.get_unchecked(index) }
    }
}

impl<T> IndexMut<usize> for UncheckedSlice<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        // Safety: The caller must ensure that `index` is within bounds.
        unsafe { self.0.get_unchecked_mut(index) }
    }
}
