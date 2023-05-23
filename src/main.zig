const std = @import("std");
//const stm32 = @import("stmicro-stm32");
//const hal = stm32.hal;
const microzig = @import("microzig");
const board = microzig.board;
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
    printChar('1');
}

//pub const RCC = microzig.peripherals.RCC;

// fn get_regs(uart: UART) *volatile UartRegs {
//     return switch (@enumToInt(uart)) {
//         0 => UART0,
//         1 => UART1,
//     };
// }

inline fn set_reg_field(reg: anytype, comptime field_name: anytype, value: anytype) void {
    var val = reg.read();
    @field(val, field_name) = value;
    reg.write(val);
}

fn get_rcc() *volatile chip.types.RCC {
    return per.RCC;
}

pub fn main() !void {
    // const Foo = MakeStruct(.{
    //     .{ "someNumber", i32 },
    //     .{ "?aBool", bool },
    //     .{ "?yourString", yourString },
    // });

    //set_reg_field(&per.RCC.APB2ENR, "IOPCEN", 1);

    // var val = per.RCC.APB2ENR.read();
    // @field(val, "IOPCEN") = 1;
    // per.RCC.APB2ENR.write(val);

    // per.RCC.APB2ENR.modify(.{
    //     .IOPCEN = 1,
    // });

    // const reg = get_rcc();
    // _ = reg;
    // const field = @field(per.RCC, "APB2ENR");
    // _ = field;
    // const field_name = "IOPCEN";
    // @field(temp, field_name) = 1;
    // per.RCC.APB2ENR.write(temp);

    //set_reg_field(per.RCC.APB2ENR, field_name, 1);

    const LED_PIN = hal.parse_pin(board.pin_map.LED);

    hal.gpio.set_output(LED_PIN);

    // per.GPIOC.CRH.modify(.{
    //     .MODE13 = 0b10,
    // });

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
