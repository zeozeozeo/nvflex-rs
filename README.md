# Safe [NVIDIA FleX](https://developer.nvidia.com/flex) bindings for Rust

<div align="center">

![License](https://img.shields.io/github/license/zeozeozeo/nvflex-rs)
![GitHub issues](https://img.shields.io/github/issues/zeozeozeo/nvflex-rs)
![GitHub pull requests](https://img.shields.io/github/issues-pr/zeozeozeo/nvflex-rs)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/zeozeozeo/nvflex-rs/main)

</div>

This is a collection of crates which provide safe idiomatic NVIDIA FleX bindings for Rust (`nvflex`), as well as raw bindings to the C++ FleX library (`nvflex-sys`).

## The `nvflex` crate

Provides safe idiomatic Rust bindings over NVIDIA FleX's C++ API.

By default, structs don't implement Send/Sync, but you can enable that functionality by enabling the `unsafe_send_sync` feature. Note that [FleX is NOT thread safe](https://gameworksdocs.nvidia.com/FleX/1.2/lib_docs/manual.html#threading), meaning only one thread should ever use the API at a time.

FleX is *not* open source, so you have to bundle the dynamic libraries with your executable. When building, this crate will copy the required libraries into the `target/debug` or `target/release` directory.

`Vector<T>` is basically `NvFlexVector<T>` from `NvFlexExt.h`, rewritten in Rust. It exists even if the `ext` feature is disabled.

To enable NVIDIA FleX extensions (NvFlexExt), enable the `ext` feature.

### Supported APIs

* On Windows: D3D11/D3D12/CUDA
* On Linux: only CUDA is supported
* On Android: only CUDA is supported (you need a Tegra GPU)

## Fixme
We somehow need to make the `Solver` aware that the `Library` is destroyed
and it shouldn't call `NvFlexDestroySolver()` when it's dropped if `NvFlexShutdown()` already destroyed it.

This means that right now, if you create a solver, it will only be destroyed when `Library` goes
out of scope.

## The `nvflex-sys` crate

`nvflex-sys` provides raw NVIDIA FleX bindings for Rust, generated with [bindgen](https://github.com/rust-lang/rust-bindgen).

Note that some functions (`NvFlexMakePhaseWithChannels`, `NvFlexMakePhase`, `NvFlexMakeShapeFlagsWithChannels` and `NvFlexMakeShapeFlags`) are implemented in Rust, because they are defined inside the `NvFlex.h` header.

### TODO

* Proper comment formatting

# License

BSL-1.0.

* Note: NVIDIA FleX is licensed under the [Nvidia Source Code License](https://raw.githubusercontent.com/NVIDIAGameWorks/FleX/master/LICENSE.txt)
