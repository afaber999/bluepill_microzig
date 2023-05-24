const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;

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

pub fn main() !void {
    printChar('G');
    printChar('G');
    printChar('G');
    printChar('G');

    const LED_PIN = hal.parse_pin(board.pin_map.LED);

    hal.gpio.set_output(LED_PIN);

    var loop_idx: u32 = 0;

    while (true) {
        if ((loop_idx % 2) == 0) {
            hal.gpio.write(LED_PIN, hal.gpio.State.high);
        } else {
            hal.gpio.write(LED_PIN, hal.gpio.State.low);
        }

        var i: u32 = 0;
        while (i < 300_000) : (i +%= 1) {
            microzig.cpu.nop();
        }

        loop_idx +%= 1;
    }
}
