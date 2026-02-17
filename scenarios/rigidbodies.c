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

  // for (int i = -1; i <= 1; ++i) {
  //   for (int j = -1; j <= 1; ++j) {
  //     *NEW_BOX.position = (v3) { i, 0.5f, j };
  //   }
  // }

  // for (int i = -1; i <= 1; ++i) {
  //   *NEW_BOX.position = (v3) { i, 1.5f, -1 };
  // }

  // *NEW_BOX.position = (v3) { -1, 1.5f, 0 };
  // *NEW_BOX.position = (v3) { 1, 1.5f, 0 };

  // *NEW_BOX.position = (v3) { 0, 2.5f, -1 };

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
}

void scenario_draw_scene(physics_world *world) {
  (void) world;
}

void scenario_draw_ui(struct nk_context* cx) {
  (void) cx;
}
