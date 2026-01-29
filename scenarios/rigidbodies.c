#include "core.h"
#include "physics.h"
#include "raylib.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void initialize_program(program_config* config, physics_config *physics_config) {
  config->window_title = "Rigidbodies";
  config->camera_position = (v3) { 22.542, 11.645, 20.752 };
  config->camera_target = (v3) { 0, 0, 0 };
}

void setup_scene(physics_world *world) {
  body box_1 = physics_add_box(world, BODY_DYNAMIC, 3, (v3) { 1, 1, 1 });
  *box_1.position = (v3) { -3, 5, 0 };
  *box_1.velocity = (v3) { 5, 0, 0 };
  *box_1.angular_momentum = (v3) { 1, 1, 1 };

  body box_2 = physics_add_box(world, BODY_DYNAMIC, 3, (v3) { 1, 1, 1 });
  *box_2.position = (v3) { 3, 5, 0 };
  *box_2.velocity = (v3) { -5, 0, 0 };
  *box_2.angular_momentum = (v3) { -1, 0, 3 };

  physics_awaken_body(world, 0);
  physics_awaken_body(world, 1);
}

void on_input(Camera *camera) {}

void draw(float interpolation) {}

void draw_ui(struct nk_context* ctx) {

}
