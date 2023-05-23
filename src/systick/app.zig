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

const LED_PIN = hal.parse_pin(board.pin_map.LED);

pub fn main() !void {

    hal.gpio.set_output(LED_PIN);

    hal.debug_log("INTERRUPTS AT START: ");
    hal.debug_print_hex(num_interrupts);
    hal.debug_log("\n");

    microzig.cpu.disable_interrupts();

    hal.peripherals.STK.CTRL.write_raw(0);
    hal.peripherals.STK.LOAD_.modify(.{ .RELOAD = 7200000 / 8 - 1 });
    hal.peripherals.STK.VAL.write_raw(0);
    hal.peripherals.STK.CTRL.modify(.{ .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 1 });
    //    hal.peripherals.STK.CTRL.modify(.{ .ENABLE = 0,});
    // stm.STK_CTRL.* = 0;
    // stm.STK_LOAD.* = 7200000 / 8 - 1;
    // stm.STK_VAL.* = 0;
    // stm.STK_CTRL.* = 7;

    microzig.cpu.enable_interrupts();

    var loop_idx: u32 = 0;

    while (true) {
        var i: u32 = 0;
        while (i < 300_000) : (i +%= 1) {
            microzig.cpu.nop();
        }
        hal.debug_log("LOOP_INTERRUPTS: ");
        hal.debug_print_hex(num_interrupts);
        // hal.debug_log("\n");

        loop_idx +%= 1;
    }
}

pub var num_interrupts: u32 = 0;

pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn SysTick() void {
            num_interrupts += 1;
            if ((num_interrupts % 2) == 0) {
                hal.gpio.write(LED_PIN, hal.gpio.State.high);
            } else {
                hal.gpio.write(LED_PIN, hal.gpio.State.low);
            }
            // interrupt handling code
        }
    };
};
