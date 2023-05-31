const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;

pub fn main() !void {
    //hal.clocks.setup_high_performance();
    hal.uart.init(9600);

    hal.clocks.dbg_show();

    const LED_PIN = hal.parse_pin(board.pin_map.LED);

    hal.gpio.set_output(LED_PIN);

    // main loop
    var loop_idx: u32 = 0;

    while (true) {

        // print line with characters A..Z every so often
        if (loop_idx == 0) {
            hal.gpio.toggle(LED_PIN);
            loop_idx = 3000_000;
            var ch: u8 = 'A';
            while (ch <= 'Z') : (ch += 1) {
                hal.uart.put_char(ch);
            }
            hal.uart.put_char('\r');
            hal.uart.put_char('\n');
        }

        if (hal.uart.has_char()) {
            // echo back the received character 10 times
            hal.uart.put_char('#');
            var ch = hal.uart.get_char();
            for (0..10) |i| {
                _ = i;
                hal.uart.put_char(ch);
            }
            hal.uart.put_char('#');
            hal.uart.put_char('\r');
            hal.uart.put_char('\n');
        }
        loop_idx -%= 1;
    }
}
