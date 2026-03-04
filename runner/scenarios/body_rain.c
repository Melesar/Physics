#include "core.h"

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Body rain";
  config->camera_position = (v3) { 0, 5, -10 };
  config->camera_target = (v3) { 0, 5, 10 };
}

void scenario_setup_scene(physics_world *world) {
  for(count_t j = 0; j < 3; ++j)
  for(count_t i = 0; i < 64; ++i) {
    bool is_box = (i + j) % 2;
    v3 position = { (int)i * 5 - 32, 5, j + 2 };

    if (is_box) {
      *physics_add_box_dynamic(world, 10, (v3) { 1, 1, 1 }).position = position;
    } else {
      *physics_add_sphere_dynamic(world, 10, 1.0).position = position;
    }
  }
}

void scenario_handle_input(physics_world *world, Camera *camera) {

}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
