const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;

pub fn RingBuffer(comptime T: type, comptime length: usize) type {
    return struct {
        const Self = @This();

        rx: usize = 0,
        wx: usize = 0,

        data: [length]T = undefined,

        pub inline fn put(self: *Self, val: T) void {
            self.data[self.wx] = val;
            self.wx = (self.wx + 1 )  % self.data.len;
        }

        pub inline fn get(self: *Self) ?T {
            if (self.wx == self.rx) {
                return null;
            }
            var ret = self.data[self.rx];
            self.rx = (self.rx + 1) % self.data.len;
            return ret;
        }
    };
}

// create a rx ring buffer
var ring_buffer = RingBuffer(u8, 32){};

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

var num_interrupts: u32 = 0;

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

    microzig.cpu.disable_interrupts();

    // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
    hal.peripherals.SCB.AIRCR.modify(.{
        .PRIGROUP = 0,
    });

    // const USART2_IRQn = 38;
    // PRIO GROUP = 38 / 4 = 9
    // PRIO SGROUP = 38 % 4 = 2
    hal.peripherals.NVIC.IPR9.modify(.{
        .IPR_N2 = 1,
    });

    // enable usart, TX and RX and RX interrupt
    hal.peripherals.USART2.CR1.modify(.{
        .RE = 1,
        .TE = 1,
        .UE = 1,
        .RXNEIE = 1,
    });

    const USART2_irqn = 38;
    var val = hal.peripherals.NVIC.ISER1.read();
    val.SETENA = val.SETENA | (1 << (USART2_irqn - 32));
    hal.peripherals.NVIC.ISER1.write(val);

    microzig.cpu.enable_interrupts();

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

            // show number of interrupts
            hal.semihosting.put_str("#num interrupts: ");
            hal.semihosting.put_hex(num_interrupts);
            hal.semihosting.put_str("\n");
        }

        while (ring_buffer.get()) |ch| {

            put_char('#');

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

// USART2 interrupt handling function
pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn USART2() void {
            num_interrupts += 1;

            if (has_char()) {
                ring_buffer.put(get_char());
            }
        }
    };
};
