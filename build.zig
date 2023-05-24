const std = @import("std");

const stm32 = @import("deps/stmicro-stm32/build.zig");
const microzig = @import("deps/stmicro-stm32/deps/microzig/build.zig");
const prj_files = @import("src/projects.zig");

pub fn build(b: *std.build.Builder) !void {
    const optimize = b.standardOptimizeOption(.{});

    inline for (@typeInfo(prj_files).Struct.decls) |decl| {
        if (!decl.is_pub)
            continue;

        const elf = microzig.addEmbeddedExecutable(b, .{
            .name = decl.name ++ ".elf",
            .source_file = @field(prj_files, decl.name).source,
            .backing = .{
                .board = stm32.boards.stm32f1bluepill,
            },
            .optimize = optimize,
        });
        elf.installArtifact(b);
    }
}
