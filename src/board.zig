pub const micro = @import("microzig");

pub const cpu_frequency = 8_000_000;

pub const pin_map = .{
    .ONBOARD_LED = "PC13",
};

// pub fn debug_write(string: []const u8) void {
//     const uart1 = micro.core.experimental.Uart(1, .{}).get_or_init(.{
//         .baud_rate = 9600,
//         .data_bits = .eight,
//         .parity = null,
//         .stop_bits = .one,
//     }) catch unreachable;

//     const writer = uart1.writer();
//     _ = writer.write(string) catch unreachable;
//     uart1.internal.txflush();
// }
