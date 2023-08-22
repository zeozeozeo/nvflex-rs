# Work in progress

# Safe Rust bindings for [NVIDIA FleX](https://developer.nvidia.com/flex)

This is a collection of crates which provide safe Rust bindings for FleX (`nvflex`), as well as raw sys bindings to the C++ library (`nvflex-sys`).

## The `nvflex` crate

Provides safe Rust bindings over NVIDIA FleX's C++ API.

By default, structs don't implement Send/Sync, but you can enable that functionality by enabling the `unsafe_send_sync` feature. Note that [FleX is NOT thread safe](https://gameworksdocs.nvidia.com/FleX/1.2/lib_docs/manual.html#threading), meaning only one thread should ever use the API at a time.

FleX is *not* open source, so you have to bundle the dynamic libraries with your executable. When building, this crate will copy the required libraries into the `target/debug` or `target/release` directory.

### Supported APIs

* On Windows: D3D11/D3D12/CUDA
* On Linux: only CUDA is supported
* On Android: only CUDA is supported (you need a Tegra GPU)

# License

BSL-1.0.

* Note: NVIDIA FleX is licensed under the [Nvidia Source Code License](https://raw.githubusercontent.com/NVIDIAGameWorks/FleX/master/LICENSE.txt)
