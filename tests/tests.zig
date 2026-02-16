const std = @import("std");

const c = @cImport(@cInclude("physics.h"));

test "Box collision" {
    const config = c.physics_default_config();
    const world = c.physics_init(&config);

    const box1 = c.physics_add_box(world, c.BODY_DYNAMIC, 10, .{ .x = 1.3, .y = 1.3, .z = 1.3 });
    box1.position.* = .{ .x = 0, .y = 1.98, .z = 0 };
    box1.rotation.* = .{ .x = 0.18, .y = 0.18, .z = 0.18, .w = 0.95 };
    box1.velocity.* = .{ .x = 0, .y = -9.88, .z = 0 };
    box1.angular_momentum.* = c.one();

    const box2 = c.physics_add_box(world, c.BODY_DYNAMIC, 10, .{ .x = 1.3, .y = 1.3, .z = 1.3 });
    box2.position.* = .{ .x = 0.45, .y = 0.65, .z = -0.59 };
    box2.rotation.* = .{ .x = -0.33, .y = 0.33, .z = -0.62, .w = 0.63 };
    box2.velocity.* = .{ .x = 0, .y = -0.15, .z = 0 };
    box2.angular_momentum.* = .{ .x = 0.04, .y = 0, .z = 0.21 };

    c.physics_step(world, 1.0 / 120.0);

    var expectedCollisionPoint = c.scale(c.one(), -0.5);
    expectedCollisionPoint = c.rotate(expectedCollisionPoint, world.*.dynamics.rotations[1]);

    const actualCollisionPoint = world.*.collisions.*.contacts[0].point;
    const err = c.distance(expectedCollisionPoint, actualCollisionPoint);

    try std.testing.expectApproxEqAbs(0.0, err, 0.001);
}
