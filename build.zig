const std = @import("std");

const ResolvedTarget = std.Build.ResolvedTarget;

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib = b.dependency("raylib", .{ .target = target, .optimize = optimize, .config = "-DPLATFORM_DESKTOP", .linkage = .static });
    const root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });

    const flags = &.{ "-g", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O0", "-fsanitize=address" };
    const library_sources = try collectSources(b, "core");
    defer b.allocator.free(library_sources);

    root_module.addCSourceFiles(.{
        .files = library_sources,
        .flags = flags,
    });

    const lib = b.addLibrary(.{
        .name = "bandura",
        .linkage = .dynamic,
        .root_module = root_module,
    });

    lib.addIncludePath(raylib.path("src"));
    lib.addIncludePath(raylib.path("examples"));
    lib.addIncludePath(b.path("include"));
    linkLibraries(lib, target);

    lib.linkLibrary(raylib.artifact("raylib"));

    b.installArtifact(lib);
}

fn linkLibraries(compile: *std.Build.Step.Compile, target: ResolvedTarget) void {
    switch (target.result.os.tag) {
        .linux => {
            compile.linkSystemLibrary("m");
            compile.linkSystemLibrary("pthread");
            compile.linkSystemLibrary("GLX");
            compile.linkSystemLibrary("X11");
            compile.linkSystemLibrary("Xcursor");
            compile.linkSystemLibrary("Xext");
            compile.linkSystemLibrary("Xfixes");
            compile.linkSystemLibrary("Xi");
            compile.linkSystemLibrary("Xinerama");
            compile.linkSystemLibrary("Xrandr");
            compile.linkSystemLibrary("Xrender");
        },
        else => return,
    }
}

fn collectSources(b: *std.Build, directory: []const u8) ![]const []const u8 {
    var sources = try std.ArrayList([]const u8).initCapacity(b.allocator, 16);
    errdefer sources.deinit(b.allocator);

    var dir = try std.fs.cwd().openDir(directory, .{ .iterate = true });
    defer dir.close();

    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.lastIndexOf(u8, entry.name, ".c") == null) continue;

        try sources.append(b.allocator, b.pathJoin(&.{ directory, entry.name }));
    }

    return sources.toOwnedSlice(b.allocator);
}
