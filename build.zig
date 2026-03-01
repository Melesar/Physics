const std = @import("std");
const zcc = @import("compile_commands");

const ResolvedTarget = std.Build.ResolvedTarget;

const COMMON_FLAGS = &.{ "-std=c99", "-Wall", "-Wextra", "-Wno-unused-parameter" };

const Options = struct {
    diagnostic: bool,

    fn getOptions(b: *std.Build) Options {
        return .{
            .diagnostic = b.option(bool, "diagnostic", "Enable diagnostics") orelse true,
        };
    }
};

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const options = Options.getOptions(b);

    const raylib = b.dependency("raylib", .{ .target = target, .optimize = optimize, .config = "-DPLATFORM_DESKTOP", .linkage = .static });
    const banduraModule = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });

    const flags = try compilerFlags(b, options, target.result, optimize);
    defer b.allocator.free(flags);

    const banduraSources = try collectSources(b, "bandura/src");
    defer b.allocator.free(banduraSources);

    banduraModule.addCSourceFiles(.{
        .files = banduraSources,
        .flags = flags,
    });

    const banduraLib = b.addLibrary(.{
        .name = "bandura",
        .linkage = .dynamic,
        .root_module = banduraModule,
    });

    banduraLib.addIncludePath(b.path("bandura/include"));

    b.installArtifact(banduraLib);
    // b.addInstallHeaderFile(b.path("bandura/include/bandura.h"), ".");

    const scenarioSources = try collectSources(b, "runner/scenarios");
    defer b.allocator.free(scenarioSources);

    var targets = try std.ArrayList(*std.Build.Step.Compile).initCapacity(b.allocator, scenarioSources.len + 2);
    defer targets.deinit(b.allocator);

    targets.appendAssumeCapacity(banduraLib);

    const binarySources = try collectSources(b, "runner");
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
        scenario.addIncludePath(b.path("runner/include"));
        scenario.addIncludePath(b.path("bandura/include"));
        // scenario.addIncludePath(b.pathJoin(b.install_path, "include"));

        linkLibraries(scenario, target);

        scenario.linkLibrary(raylib.artifact("raylib"));
        scenario.linkLibrary(banduraLib);

        b.installArtifact(scenario);

        const runScenario = b.addRunArtifact(scenario);
        runScenario.step.dependOn(b.getInstallStep());

        const runStep = b.step(b.fmt("run-{s}", .{scenarioName}), b.fmt("Run {s} scenario", .{scenarioName}));
        runStep.dependOn(&runScenario.step);

        targets.appendAssumeCapacity(scenario);
    }

    _ = zcc.createStep(b, "cdb", try targets.toOwnedSlice(b.allocator));

    const tests = b.addTest(.{
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .root_source_file = b.path("tests/tests.zig"),
            .link_libc = true,
        }),
    });

    tests.linkLibrary(banduraLib);
    tests.addIncludePath(b.path("bandura/include"));

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

fn compilerFlags(b: *std.Build, options: Options, target: std.Target, optimize: std.builtin.OptimizeMode) ![]const []const u8 {
    var flags = try std.ArrayList([]const u8).initCapacity(b.allocator, 32);
    errdefer flags.deinit(b.allocator);

    var sanitizers = try std.ArrayList([]const u8).initCapacity(b.allocator, 2);
    errdefer sanitizers.deinit(b.allocator);

    try flags.appendSlice(b.allocator, COMMON_FLAGS);

    if (options.diagnostic) {
        try flags.append(b.allocator, "-DDIAGNOSTICS");
    }

    switch (optimize) {
        .Debug => {
            try flags.appendSlice(b.allocator, &.{ "-g", "-O0", "-DDEBUG_MODE" });
        },

        .ReleaseSafe => {
            try flags.appendSlice(b.allocator, &.{"-O2"});
        },

        .ReleaseFast => {
            try flags.appendSlice(b.allocator, &.{"-O3"});
        },

        .ReleaseSmall => {
            try flags.appendSlice(b.allocator, &.{"-Os"});
        },
    }

    if (optimize == .Debug or optimize == .ReleaseSafe) {
        try sanitizers.append(b.allocator, "float-divide-by-zero");

        if (target.os.tag != .macos or target.cpu.arch != .aarch64) {
            try sanitizers.append(b.allocator, "leak");
        }
    }

    if (sanitizers.items.len > 0) {
        try flags.append(b.allocator, b.fmt("-fsanitize={s}", .{try std.mem.join(b.allocator, ",", try sanitizers.toOwnedSlice(b.allocator))}));
    }

    return flags.toOwnedSlice(b.allocator);
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
