const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;

// default internal clock speed bluepill board (8MHz)
const SystemCoreClock = 8000000;

inline fn put_char(ch: u8) void {
    while (hal.peripherals.USART2.SR.read().TXE != 0b1) {}
    hal.peripherals.USART2.DR.write_raw(ch);
}

inline fn has_char() bool {
    return (hal.peripherals.USART2.SR.read().RXNE == 0b1);
}

inline fn get_char() u8 {
    return @truncate(u8, hal.peripherals.USART2.DR.raw);
}

pub fn main() !void {

    // Enable peripheral clocks: GPIOA, USART2.
    hal.peripherals.RCC.APB1ENR.modify(.{
        .USART2EN = 1,
    });
    hal.peripherals.RCC.APB2ENR.modify(.{
        .IOPAEN = 1,
    });

    // pin 2/3 alternate push/pull, max 10 MHz
    hal.peripherals.GPIOA.CRL.modify(.{
        .MODE2 = 0b01,
        .CNF2 = 0b10,
        .MODE3 = 0b00,
        .CNF3 = 0b10,
    });

    // Set the baud rate to 9600.
    const uartdiv = SystemCoreClock / 9600;

    hal.peripherals.USART2.BRR.modify(.{
        .DIV_Mantissa = (uartdiv / 16),
        .DIV_Fraction = (uartdiv % 16),
    });

    // enable usart, TX and RX
    hal.peripherals.USART2.CR1.modify(.{
        .RE = 1,
        .TE = 1,
        .UE = 1,
    });

    // main loop
    var loop_idx: u32 = 0;

    while (true) {

        // print line with characters A..Z every so often
        if (loop_idx == 0) {
            loop_idx = 3000_000;
            var ch: u8 = 'A';
            while (ch <= 'Z') : (ch += 1) {
                put_char(ch);
            }
            put_char('\r');
            put_char('\n');
        }

        if (has_char()) {
            // echo back the received character 10 times
            put_char('#');
            var ch = get_char();
            for (0..10) |i| {
                _ = i;
                put_char(ch);
            }
            put_char('#');
            put_char('\r');
            put_char('\n');
        }
        loop_idx -%= 1;
    }
}
