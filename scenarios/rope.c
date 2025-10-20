#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include "stdlib.h"

typedef struct {
  int num_links;

  Vector3 *link_pos;
  Vector3 *link_vel;

  Vector3 anchor;
  rigidbody body;
  
} rope;

const int num_links = 10;
const float link_spacing = 0.8;
const float body_mass = 3;

rope r;

void initialize_program(program_config* config) {
  config->window_title = "Rope";
  config->camera_mode = CAMERA_CUSTOM;
  config->camera_position = (Vector3) { 0.609, 8.439, 22.771 };
  config->camera_target = (Vector3) { 0, 3, 0 };
}

void setup_scene(Shader shader) {
  float rope_len = num_links * link_spacing;
  r.num_links = num_links;
  r.link_pos = (Vector3*) malloc(num_links * sizeof(Vector3));
  r.link_vel = (Vector3*) malloc(num_links * sizeof(Vector3));
  r.anchor = (Vector3) { 0, rope_len + 2, 0 };

  reset();
}

void save_state() {
  
}

void on_input() {
  
}

void reset() {
  r.body = rb_new(Vector3Add(r.anchor, (Vector3) { (r.num_links * link_spacing) + link_spacing, 0, 0 }), body_mass);
  for (int i = 0; i < num_links; ++i) {
    r.link_pos[i] = Vector3Add(r.anchor, (Vector3) { (i + 1) * link_spacing, 0, 0 });
    r.link_vel[i] = Vector3Zero();
  }
}

void simulate(float dt) {
  
}

void draw(float interpolation) {
  DrawSphere(r.anchor, link_spacing * 0.5, RED);
  DrawSphere(r.body.position, 0.5, PURPLE);

  DrawLine3D(r.anchor, r.link_pos[0], BLACK);
  DrawLine3D(r.link_pos[num_links - 1], r.body.position, BLACK);
  for (int i = 0; i < num_links - 1; ++i) {
    DrawLine3D(r.link_pos[i], r.link_pos[i + 1], BLACK);
  }
}

void draw_ui(struct nk_context* ctx) {
  
}

