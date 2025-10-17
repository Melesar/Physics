#ifndef CORE_H
#define CORE_H

#define NK_INCLUDE_STANDARD_VARARGS
#include "raylib-nuklear.h"

typedef struct {
  char* window_title;
  int camera_mode;
  Vector3 camera_position;
  Vector3 camera_target;
} program_config;

struct object {
  char* label;
  Mesh mesh;
  Material material;
};

void initialize_program(program_config* config);
void setup_scene(Shader shader);
void save_state();
void on_input();
void reset();
void simulate(float dt);
void draw(float interpolation);
void draw_ui(struct nk_context* ctx);

void draw_arrow(Vector3 start, Vector3 end, Color color);

#endif
