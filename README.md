# bluepill_microzig
test repo for experimenting with microzig on a bluepill board

Create project zig init-exe
Add depdency: git submodule add gitrepo deps/stmicro-stm32


git submodule update --init --recursive
set PATH=D:\arm\tools\gcc-arm-none-eabi-10.3-2021.10\bin;%PATH%

arm-none-eabi-gdb -x gdbdebug ./zig-out/bin/blink.elf
arm-none-eabi-objdump -D ./zig-out/bin/blink.elf


Start with blink led
