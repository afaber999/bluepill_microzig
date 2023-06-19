const std = @import("std");
const microzig = @import("microzig");
const hal = microzig.hal;
const board = microzig.board;

const tst = @import("usb_test_0.zig");

pub const std_options = struct {
    pub const log_level = .debug;
    pub const logFn = hal.usart.log;
};

pub fn main() !void {

    // setup high speed configuration, board has a 8MHz crystal
    hal.rcc.setup_high_performance(8_000_000);

    // use USART2 for logging
    var uart = hal.usart.num(2);
    uart.apply(115200);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);

    tst.main();
}
