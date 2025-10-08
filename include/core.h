#ifndef CORE_H
#define CORE_H

#include "raymath.h"

typedef struct {
  char* window_title;
  int camera_mode;
  Vector3 camera_position;
  Vector3 camera_target;
} program_config;

void initialize_program(program_config* config);
void setup_scene();
void save_state();
void simulate(float dt);
void draw(float interpolation);


#endif
