const std = @import("std");
const zcc = @import("compile_commands");

const ResolvedTarget = std.Build.ResolvedTarget;

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib = b.dependency("raylib", .{ .target = target, .optimize = optimize, .config = "-DPLATFORM_DESKTOP", .linkage = .static });
    const libRootModule = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });

    const flags = &.{ "-g", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O0" };
    const librarySources = try collectSources(b, "lib");
    defer b.allocator.free(librarySources);

    libRootModule.addCSourceFiles(.{
        .files = librarySources,
        .flags = flags,
    });

    const lib = b.addLibrary(.{
        .name = "bandura",
        .linkage = .dynamic,
        .root_module = libRootModule,
    });

    lib.addIncludePath(b.path("include"));

    b.installArtifact(lib);

    const scenarioSources = try collectSources(b, "scenarios");
    defer b.allocator.free(scenarioSources);

    var targets = try std.ArrayList(*std.Build.Step.Compile).initCapacity(b.allocator, scenarioSources.len + 2);
    defer targets.deinit(b.allocator);

    targets.appendAssumeCapacity(lib);

    const binarySources = try collectSources(b, "bin");
    defer b.allocator.free(binarySources);

    for (scenarioSources) |scenarioFile| {
        const scenarioModule = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });
        scenarioModule.addCSourceFiles(.{
            .files = binarySources,
            .flags = flags,
        });
        scenarioModule.addCSourceFile(.{ .file = b.path(scenarioFile), .flags = flags });

        const scenarioName = std.fs.path.stem(scenarioFile);
        const scenario = b.addExecutable(.{
            .name = scenarioName,
            .root_module = scenarioModule,
        });

        scenario.addIncludePath(raylib.path("src"));
        scenario.addIncludePath(raylib.path("examples"));
        scenario.addIncludePath(b.path("include"));

        linkLibraries(scenario, target);

        scenario.linkLibrary(raylib.artifact("raylib"));
        scenario.linkLibrary(lib);

        b.installArtifact(scenario);

        const runScenario = b.addRunArtifact(scenario);
        runScenario.step.dependOn(b.getInstallStep());

        const runStep = b.step(b.fmt("run-{s}", .{scenarioName}), b.fmt("Run {s} scenario", .{scenarioName}));
        runStep.dependOn(&runScenario.step);

        targets.appendAssumeCapacity(scenario);
    }

    _ = zcc.createStep(b, "cdb", try targets.toOwnedSlice(b.allocator));

    const testsModule = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("tests/tests.zig"),
        .link_libc = true,
    });

    const tests = b.addTest(.{
        .root_module = testsModule,
    });

    tests.linkLibrary(lib);
    tests.addIncludePath(b.path("include"));

    const runTests = b.addRunArtifact(tests);
    const testStep = b.step("test", "Run tests");
    testStep.dependOn(&runTests.step);
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
        .macos => {
            compile.linkFramework("IOKit");
            compile.linkFramework("Cocoa");
            compile.linkFramework("OpenGL");
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
