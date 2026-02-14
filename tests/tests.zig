const std = @import("std");
const expect = std.testing.expect;
const c = @cImport(@cInclude("physics.h"));

test "Dummy test" {
    const config = c.physics_default_config();
    const world = c.physics_init(&config);

    try expect(world.*.dynamics.count == 0);
}
