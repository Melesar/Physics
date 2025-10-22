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
void on_input(Camera *camera);
void reset();
void simulate(float dt);
void draw(float interpolation);
void draw_ui(struct nk_context* ctx);

void draw_arrow(Vector3 start, Vector3 end, Color color);
void draw_stat_float(struct nk_context* ctx, char* title, float value);
void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value);
void draw_stat_matrix(struct nk_context* ctx, char* title, Matrix value);

void draw_property_float(struct nk_context* ctx, char* title, float* value, float min, float max, float step_arrow, float step_drag);

#endif
