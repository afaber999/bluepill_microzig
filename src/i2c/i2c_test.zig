const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;
const peripherals = microzig.chip.peripherals;

const LED_PIN = hal.parse_pin(board.pin_map.LED);

const SCL_PIN = hal.parse_pin("PB10");
const SDA_PIN = hal.parse_pin("PB11");

pub fn i2c_pin_config() void {

    // set i2c pins
    hal.gpio.set_output(SCL_PIN, hal.gpio.OutputMode.alternate_pushpull, hal.gpio.OutputSpeed.output_10MHz);
    hal.gpio.set_output(SDA_PIN, hal.gpio.OutputMode.alternate_opendrain, hal.gpio.OutputSpeed.output_10MHz);

    // enable i2c
    peripherals.RCC.APB1ENR.modify(.{ .I2C2EN = 0b1 });
}

pub fn i2c_master_init() void {
    i2c_pin_config();

    // set clock, should be the same as apb which is 72 / 2 = 26
    peripherals.I2C2.CR2.modify(.{ .FREQ = 36 });

    // set speed
    peripherals.I2C2.CCR.modify(.{ .CCR = 0x140 });
}

pub fn main() !void {
    hal.rcc.setup_high_performance(8_000_000);

    // num 1 = USART2
    var uart = hal.usart.num(2);
    uart.apply(9600);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);
    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    i2c_master_init();

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
