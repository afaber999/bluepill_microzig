const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;

pub fn main() !void {
    dbg.put_str("Blinky!\n");
    
    const LED_PIN = hal.parse_pin(board.pin_map.LED);

    hal.gpio.set_output(LED_PIN);

    var loop_idx: u32 = 0xBABEFACE;

    while (true) {
        var tw: u16 = 0xABCD;
        var tb: u8 = 0x5A;

        dbg.put_hex(loop_idx);
        dbg.put_str("\n");

        dbg.put_hex(tw);
        dbg.put_str("\n");

        dbg.put_hex(tb);
        dbg.put_str("\n");

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
