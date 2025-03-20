# ch592 minimal
WCH CH592/1 minimal example without SDK.
Just the pointer #defines in CH592SFR.h from the official SDK, a linker script and the startup assembly.
The C source flashes the LED on pin 8 of the WeAct module, and sleeps in between.

# compile
For compilation GCC `riscv64-elf` and WCH MounRiver Studio are tested, with the latter producing a 150 byte smaller firmware.

# flash
The resulting `.bin` can be flashed with the awesome [`chprog.py`](https://github.com/wagiminator/MCU-Flash-Tools), which
is used in the `flash` and `f` targets in the Makefile. Also `.hex` is produced, and any flasher of your choice can be used.
