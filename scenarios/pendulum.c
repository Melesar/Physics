#include "core.h"
#include "raylib.h"

void initialize_program(program_config* config) {
   config->camera_mode = CAMERA_CUSTOM; 
   config->window_title = "Pendulums";
   config->camera_position = (Vector3) { 2, 7, 10 };
   config->camera_target = (Vector3) { 2, 0.5, 0 };
}

void setup_scene() {
  
}

void save_state() {
  
}
void simulate(float dt) {
  
}

void draw(float interpolation) {
  
}

void draw_ui(struct nk_context* ctx) {
  
}
