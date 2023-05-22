const std = @import("std");

const stm32 = @import("deps/stmicro-stm32/build.zig");
const microzig = @import("deps/stmicro-stm32/deps/microzig/build.zig");

pub fn build(b: *std.build.Builder) !void {
    const optimize = b.standardOptimizeOption(.{});

    var elf = microzig.addEmbeddedExecutable(b, .{
        .name = "blink.elf",
        .source_file = .{
            .path = "src/main.zig",
        },
        .backing = .{
            // .board = stm32.boards.STM32F3DISCOVERY,
            .chip = stm32.chips.stm32f103x8,
        },
        .optimize = optimize,
    });
    elf.installArtifact(b);
}
