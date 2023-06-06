const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;

const LED_PIN = hal.parse_pin(board.pin_map.LED);

pub const std_options = struct {
    pub const log_level = .debug;
    pub const logFn = hal.usart.log;
};

pub fn time_test() void {

//    var start_millis : u32 = 0xFFFFFFFF-50;
    var start_millis : u32 = 0xFF-50;
    var b : u32 = 12;
    var target_time: i32 = @bitCast(i32, start_millis +% b);


    for (0..13) | i| {
        var millis: i32 = @bitCast(i32, start_millis +% @intCast(u32,i));
        var r = target_time - millis;
        // r > 0 
        std.log.info("i = {} target_time = {} millis = {} r = {} ", .{i,target_time,  millis ,r});
        
    }
    
}

pub fn main() !void {
    // setup high speed configuration, board has a 8MHz crystal
    hal.rcc.setup_high_performance(8_000_000);

    // use USART2 for logging
    var uart = hal.usart.num(2);
    uart.apply(9600);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);
    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    // main loop
    var loop_idx: u32 = 0;

    std.log.info("Programming target region...", .{});

    time_test();


    while (true) {
        uart.put_hex(loop_idx);
        uart.put_char('\r');
        uart.put_char('\n');
        loop_idx += 1;
        std.log.err("Loop index from logging as err ... 0x{X:0>8}", .{loop_idx});

        hal.systick.delay_ms(5000);
    }
}
