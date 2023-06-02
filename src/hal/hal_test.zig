const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;

pub fn main() !void {

    hal.rcc.setup_high_performance(8_000_000);

    // hal.rcc.dbg_show();

    // num 1 = USART2
    var uart = hal.usart.num(1);
    uart.apply(9600);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);

    //uart.init(9600);

    const LED_PIN = hal.parse_pin(board.pin_map.LED);

    hal.gpio.set_output(LED_PIN);

    // main loop
    var loop_idx: u32 = 0;

    while (true) {
        uart.put_hex(loop_idx);
        uart.put_char('\r');
        uart.put_char('\n');
        loop_idx += 1;

        hal.systick.delay_ms(5000);
    }
}
