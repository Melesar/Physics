#include "core.h"
#include "physics.h"
#include "raylib.h"

#include <math.h>
#include <unistd.h>

bool is_collision;
raycast_hit hit;

void scenario_initialize(program_config* config, physics_config *physics) {
  (void) physics;

  config->window_title = "Rigidbodies";
  config->camera_position = (v3) { 22.542, 11.645, 20.752 };
  config->camera_target = (v3) { 0, 0, 0 };
}

void scenario_setup_scene(physics_world *world) {
  #define NEW_BOX physics_add_box(world, BODY_DYNAMIC, 10, (v3) { 1.3, 1.3, 1.3 })

  *NEW_BOX.position = (v3) { 0, 0.5, 0 };

  body big_box = physics_add_box(world, BODY_STATIC, 10, (v3) { 10, 3, 1 });
  *big_box.position = (v3) { 0, 1.5, -5 };

  big_box = physics_add_box(world, BODY_STATIC, 10, (v3) { 1, 3, 10 });
  *big_box.position = (v3) { -7, 1.5, 0 };

  #undef NEW_BOX
}

void scenario_simulate(physics_world *world, float dt) {
  (void) world;
  (void) dt;
}

void scenario_handle_input(physics_world *world, Camera *cam) {
  (void) cam;

  if (IsKeyPressed(KEY_X)) {
    body big_box = physics_add_box(world, BODY_DYNAMIC, 10, (v3) {1.3, 1.3, 1.3});
    *big_box.position = (v3) { 0, 7, 0 };
    *big_box.angular_momentum = (v3) { 1, 1, 1 };

    physics_awaken_body(world, world->dynamics.count - 1);
  }

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    v3 direction = normalize(sub(cam->target, cam->position));

    body ball = physics_add_sphere(world, BODY_DYNAMIC, 3, 0.7);
    *ball.position = add(cam->position, direction);

    count_t index = world->dynamics.count - 1;
    world->dynamics.impulses[index] = scale(direction, 70);
    physics_awaken_body(world, index);
  }
}

void scenario_draw_scene(physics_world *world) {
  (void) world;
}

void scenario_draw_ui(struct nk_context* cx) {
  (void) cx;
}
