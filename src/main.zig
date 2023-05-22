const std = @import("std");
const board = @import("board.zig");
//const stm32 = @import("stmicro-stm32");
//const hal = stm32.hal;
const microzig = @import("microzig");
const chip = microzig.chip;
const hal = microzig.hal;
const per = microzig.chip.peripherals;

pub fn printChar(c: u8) void {
    var ch_ptr: *const u8 = &c;
    asm volatile (
        \\mov r0, 03 
        \\mov r1, %[ch]
        \\nop
        \\bkpt #0xAB
        :
        : [ch] "r" (ch_ptr),
        : "r0", "r1"
    );
}

// http://embed.rs/articles/2016/semi-hosting-rust/
pub fn debug_spam() void {
    printChar('A');
    printChar('X');
    printChar('B');
    printChar('C');
}

pub fn main() !void {

    per.RCC.APB2ENR.modify( .{.IOPCEN = 1,});

    const LED_PIN = hal.parse_pin("PC13");
    _ = LED_PIN;

    per.GPIOC.CRH.modify(.{
        .MODE13 = 0b10,
    });

    var loop_idx: u32 = 0;
    while (true) {
        debug_spam();

        var i: u32 = 0;
        while (i < 300_000) : (i +%= 1) {
            microzig.cpu.nop();
        }

        if ((loop_idx % 2) == 0) {
            printChar('@');
            per.GPIOC.ODR.modify(.{
                .ODR13 = 0b1,
            });
        } else {
            printChar('#');
            per.GPIOC.ODR.modify(.{
                .ODR13 = 0b0,
            });
        }

        loop_idx +%= 1;

        // per.GPIOC.ODR.toggle(.{
        //     .ODR13,
        // });
    }
}
