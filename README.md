# `stm32mp1xx-hal`

> [HAL] for the STM32MP1 family of microcontrollers

[HAL]: https://crates.io/crates/embedded-hal

[![Continuous integration](https://github.com/stm32-rs/stm32mp1xx-hal/workflows/Continuous%20integration/badge.svg)](https://github.com/stm32-rs/stm32mp1xx-hal)
[![crates.io](https://img.shields.io/crates/v/stm32mp1xx-hal.svg)](https://crates.io/crates/stm32mp1xx-hal)
[![Released API docs](https://docs.rs/stm32np1xx-hal/badge.svg)](https://docs.rs/stm32np1xx-hal)

## Quick start guide

Embedded Rust development requires a bit more setup than ordinary development.
For this guide, we'll assume you're using a stm32mp1-dk2 board, but if you have another board you should be able to adapt it.

### Installing software

To program your microcontroller, you need to install:
- [openocd](http://openocd.org/)
- `gdb-multiarch` (on some platforms you may need to use `gdb-arm-none-eabi` instead, make sure to update `.cargo/config` to reflect this change)

Finally, you need to install arm target support for the Rust compiler. To do
so, run
```
rustup target install thumbv7m-none-eabi
```


### Setting up your project

Create a new Rust project as you usually do with `cargo init`. The hello world
of embedded development is usually to blink an LED and code to do so is
available in [examples/blinky.rs](examples/blinky.rs). Copy that file to the
`main.rs` of your project.

You also need to add some dependencies to your `Cargo.toml`:

```toml
[dependencies]
embedded-hal = "0.2.3"
nb = "0.1.2"
cortex-m = "0.6.2"
cortex-m-rt = "0.6.11"
# Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
panic-halt = "0.2.0"

[dependencies.stm32mp1xx-hal]
version = "0.6.1"
features = ["rt", "stm32mp157"]
```

If you build your project now, you should get a single error: `error: language
item required, but not found: eh_personality`. This unhelpful error message 
is fixed by compiling for the right target.

We also need to tell Rust how to link our executable, and how to lay out the
result in memory. To accomplish all this, copy [.cargo/config](.cargo/config) and
[memory.x](memory.x) from the stm32mp1xx-hal repo to your project.

```bash
cargo build
```

If everything went well, your project should have built without errors.


### Going further

From here on, you can start adding more code to your project to make it do
something more interesting. For crate documentation, see
[docs.rs/stm32f1xx-hal](https://docs.rs/stm32mp1xx-hal). There are also a lot
more [examples](examples) available. If something is unclear in the docs or
examples, please, open an issue and we will try to improve it.




## Selecting a microcontroller

This crate supports multiple microcontrollers in the
stm32mp1 family. Which specific microcontroller you want to build for has to be
specified with a feature, for example `stm32mp157`. 

If no microcontroller is specified, the crate will not compile.


### Supported Microcontrollers

* `stm32mp157`

## Trying out the examples

You may need to give `cargo` permission to call `gdb` from the working directory.
- Linux
  ```bash
  echo "set auto-load safe-path $(pwd)" >> ~/.gdbinit
  ```
- Windows
  ```batch
  echo set auto-load safe-path %CD% >> %USERPROFILE%\.gdbinit
  ```

Compile, load, and launch the hardware debugger.
```bash
$ rustup target add thumbv7m-none-eabi

# on another terminal
$ openocd /usr/share/openocd/scripts/board/stm32mp15x_dk2.cfg

# flash and debug the "Hello, world" example
$ cargo run --features stm32mp157 --example hello
```

[embeddonomicon]: https://rust-embedded.github.io/book/start/hardware.html



## Using as a Dependency

When using this crate as a dependency in your project, the microcontroller can 
be specified as part of the `Cargo.toml` definition.

```toml
[dependencies.stm32mp1xx-hal]
version = "0.6.1"
features = ["stm32mp157", "rt"]
```

## Documentation

The documentation can be found at [docs.rs](https://docs.rs/stm32mp1xx-hal/).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
