const std = @import("std");

const stm32 = @import("deps/stmicro-stm32/build.zig");
const microzig = @import("deps/stmicro-stm32/deps/microzig/build.zig");
//const gdb_exe = b.addExecutable("arm-none-eabi-gdb", "doc/docgen.zig");

fn root() []const u8 {
    return comptime (std.fs.path.dirname(@src().file) orelse ".") ++ "/";
}

const ProjectFile = struct {
    name: []const u8,
    source: std.build.FileSource,
    bin_path: []const u8,
};

const prj_files = [_]ProjectFile{
    .{
        .name = "blinky",
        .source = .{ .path = root() ++ "src/blinky/blinky.zig" },
        .bin_path = root() ++ "zig-out/bin/blinky.elf",
    },
    .{
        .name = "systick",
        .source = .{ .path = root() ++ "src/systick/systick.zig" },
        .bin_path = root() ++ "zig-out/bin/systick.elf",
    },
    .{
        .name = "usart",
        .source = .{ .path = root() ++ "src/usart/usart.zig" },
        .bin_path = root() ++ "zig-out/bin/usart.elf",
    },
    .{
        .name = "usart_interrupt",
        .source = .{ .path = root() ++ "src/usart/usart_interrupt.zig" },
        .bin_path = root() ++ "zig-out/bin/usart_interrupt.elf",
    },    
};

const linkerscript_path = root() ++ "stm32f1xx.ld";

pub fn build(b: *std.build.Builder) !void {
    const optimize = b.standardOptimizeOption(.{});
    var project = b.option([]const u8, "project", "specify a specific project");

    inline for (prj_files) |prj_file| {
        const elf = microzig.addEmbeddedExecutable(b, .{
            .name = prj_file.name ++ ".elf",
            .source_file = prj_file.source,
            .backing = .{
                .board = stm32.boards.stm32f1bluepill,
            },
            .optimize = optimize,
            .linkerscript_source_file = .{ .path = linkerscript_path },
        });

        elf.installArtifact(b);

        // const objdump = b.step("objdump", "Show dissassembly of the code using gdb-arm-objdump");
        // _ = objdump;
        // b.addSystemCommand(&.{
        //     "arm-none-eabi-gdb",
        //     "B",
        // });

        // const avr_objdump = b.addSystemCommand(&.{
        //     "avr-objdump",
        //     "-dh",
        //     "arm-none-eabi-gdb",
        // });
        // objdump.dependOn(&avr_objdump.step);
        // avr_objdump.step.dependOn(&elf.install_step.?.step);
    }

    if (project) |project_name| {
        inline for (prj_files) |prj_file| {
            if (std.mem.eql(u8, project_name, prj_file.name)) {
                std.debug.print("\nproject_name :{s}\n", .{prj_file.bin_path});

                const objdump = b.step("objdump", "Show dissassembly of the code using arm-none-eabi-objdump");
                const obj_dump_exe = b.addSystemCommand(&.{
                    "arm-none-eabi-objdump",
                    "-Dh",
                    prj_file.bin_path,
                    " > " ++ prj_file.bin_path ++ ".lst",
                });
                objdump.dependOn(&obj_dump_exe.step);
                //obj_dump_exe.step.dependOn();
                //_ = obj_dump_exe;

                const flash = b.step("flash", "Flash the elf with gdb");
                const gdb_exe = b.addSystemCommand(&.{
                    "arm-none-eabi-gdb",
                    "-x",
                    "gdbdebug",
                    prj_file.bin_path,
                });
                flash.dependOn(&gdb_exe.step);

                break;
            }
        } else {
            std.debug.print("project name not found in project list {s}", .{project_name});
        }
    }
    // inline for (@typeInfo(prj_files).Struct.decls) |decl| {
    //     std.debug.print("\n", .{});
    //     if (flash) |flash_item| {
    //         std.debug.print("compare :{s}: with :{s}:\n", .{ decl.name, flash_item });
    //         //     std.debug.print("flash option {s}", .{flash_item});
    //         // if (flash == decl.name) {
    //         //     std.debug.print("flash option {s}", .{flash_item});
    //         // }
    //     }
    // }
}
