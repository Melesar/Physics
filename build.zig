const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib = b.dependency("raylib", .{
        .target = target,
        .optimize = optimize,
    });

    const pikhotics = b.createModule(.{
        .optimize = optimize,
        .target = target,
        .link_libc = true,
    });

    pikhotics.addIncludePath(raylib.path("src"));
    pikhotics.addIncludePath(raylib.path("examples"));
    pikhotics.addIncludePath(b.path("include"));

    var dir = try std.fs.cwd().openDir("core", .{ .iterate = true });
    defer dir.close();

    const flags = &.{ "-g", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O0", "-fsanitize=address" };
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.lastIndexOf(u8, entry.name, ".c") == null) continue;
        if (std.mem.eql(u8, entry.name, "main")) continue;

        pikhotics.addCSourceFile(.{
            .file = b.path(b.pathJoin(&.{ "core", entry.name })),
            .flags = flags,
        });
    }

    const lib = b.addLibrary(.{
        .name = "pikhosics",
        .linkage = .dynamic,
        .root_module = pikhotics,
    });
    lib.linkLibrary(raylib.artifact("raylib"));

    const exe = b.addExecutable(.{
        .name = "physics",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });

    exe.addCSourceFile(.{ .file = b.path("core/main.c"), .flags = flags });
    exe.addCSourceFile(.{ .file = b.path("scenarios/rigidbodies.c"), .flags = flags });

    exe.linkFramework("IOKit");
    exe.linkFramework("Cocoa");
    exe.linkFramework("OpenGL");
    exe.linkLibrary(lib);

    b.installArtifact(exe);
}
