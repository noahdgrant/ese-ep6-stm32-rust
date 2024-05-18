# About

 TODO

## Usage
To build, flash, and listen for logs use...
``` console
cargo run
```

To change the log level that is displayed use...
``` console
DEFMT_LOG=<level> cargo run
```

TODO: find a good way to only flash the device and not listen for logs because
currently when you press Ctrl+C after running `cargo run` the program stops running
on the STM32F3.
