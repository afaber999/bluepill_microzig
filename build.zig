const std = @import("std");

const stm32 = @import("deps/stmicro-stm32/build.zig");
const microzig = @import("deps/stmicro-stm32/deps/microzig/build.zig");

pub fn build(b: *std.build.Builder) !void {
    const optimize = b.standardOptimizeOption(.{});

    //var arStr: [2][]const u8 = [_][]const u8{"Hello", "There"};
    //var files = [2][]const u8{ "src/blinky/app.zig", "src/systick/app.zig" };

    var elf = microzig.addEmbeddedExecutable(b, .{
        .name = "blink.elf",
        .source_file = .{
            .path = "src/blinky/app.zig",
        },
        .backing = .{
            .board = stm32.boards.stm32f1bluepill,
        },
        .optimize = optimize,
    });
    elf.installArtifact(b);

    elf = microzig.addEmbeddedExecutable(b, .{
        .name = "systick.elf",
        .source_file = .{
            .path = "src/systick/app.zig",
        },
        .backing = .{
            .board = stm32.boards.stm32f1bluepill,
        },
        .optimize = optimize,
    });
    elf.installArtifact(b);
}
