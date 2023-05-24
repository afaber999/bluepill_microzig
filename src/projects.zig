const std = @import("std");

fn root() []const u8 {
    return std.fs.path.dirname(@src().file) orelse unreachable;
}

const ProjectFile = struct {
    name: []const u8,
    source: std.build.FileSource,
};

const root_path = root() ++ "/";

pub const blinky = ProjectFile{
    .name = "blinky",
    .source = .{ .path = root_path ++ "blinky/blinky.zig" },
};

pub const systick = ProjectFile{
    .name = "systick",
    .source = .{ .path = root_path ++ "systick/systick.zig" },
};
