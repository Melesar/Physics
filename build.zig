const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib = b.dependency("raylib", .{ .target = target, .optimize = optimize, .config = "-DPLATFORM_DESKTOP" });
    const root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });

    var dir = try std.fs.cwd().openDir("core", .{ .iterate = true });
    defer dir.close();

    const flags = &.{ "-g", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O0", "-fsanitize=address" };
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.lastIndexOf(u8, entry.name, ".c") == null) continue;
        if (std.mem.eql(u8, entry.name, "main.c")) continue;

        root_module.addCSourceFile(.{
            .file = b.path(b.pathJoin(&.{ "core", entry.name })),
            .flags = flags,
        });
    }

    const lib = b.addLibrary(.{
        .name = "bandura",
        .linkage = .dynamic,
        .root_module = root_module,
    });

    lib.addIncludePath(raylib.path("src"));
    lib.addIncludePath(raylib.path("examples"));
    lib.addIncludePath(b.path("include"));

    lib.linkLibrary(raylib.artifact("raylib"));

    if (target.result.os.tag == .linux) {
        lib.linkSystemLibrary("m");
        lib.linkSystemLibrary("pthread");
        lib.linkSystemLibrary("GLX");
        lib.linkSystemLibrary("X11");
        lib.linkSystemLibrary("Xcursor");
        lib.linkSystemLibrary("Xext");
        lib.linkSystemLibrary("Xfixes");
        lib.linkSystemLibrary("Xi");
        lib.linkSystemLibrary("Xinerama");
        lib.linkSystemLibrary("Xrandr");
        lib.linkSystemLibrary("Xrender");
    }

    b.installArtifact(lib);
}
