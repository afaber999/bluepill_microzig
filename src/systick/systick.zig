const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;

// onboard LED
const LED_PIN = hal.parse_pin(board.pin_map.LED);

// var to count the number of systicks
var num_interrupts: u32 = 0;

pub fn main() !void {

    // set onboard LED as output
    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    // systick setup
    microzig.cpu.disable_interrupts();

    hal.peripherals.STK.CTRL.write_raw(0);
    hal.peripherals.STK.LOAD_.modify(.{ .RELOAD = 7200000 / 8 - 1 });
    hal.peripherals.STK.VAL.write_raw(0);
    hal.peripherals.STK.CTRL.modify(.{ .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 0 });

    microzig.cpu.enable_interrupts();

    // main loop
    while (true) {

        // small delay
        var i: u32 = 0;
        while (i < 300_000) : (i +%= 1) {
            microzig.cpu.nop();
        }

        // show number of interrupts
        hal.semihosting.put_str("#num interrupts: ");
        hal.semihosting.put_hex(num_interrupts);
        hal.semihosting.put_str("\n");
    }
}

// SysTick interrupt handling function
pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn SysTick() void {
            num_interrupts += 1;

            // toggle onboard LED
            if ((num_interrupts % 2) == 0) {
                hal.gpio.write(LED_PIN, hal.gpio.State.high);
            } else {
                hal.gpio.write(LED_PIN, hal.gpio.State.low);
            }
        }
    };
};


