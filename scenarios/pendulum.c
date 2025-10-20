#include "core.h"
#include "raylib.h"
#include "physics.h"
#include "math.h"
#include "raymath.h"
#include <string.h>

struct pendulum {
  Vector3 anchor;
  rigidbody body;

  float string_length;
  float stabilization_factor;
};

const float initial_angle = 0.25 * PI;
const float string_length = 2;
const float default_mass = 1;
const float default_stiffness = 20;
const float defualt_damping = 3;
const float default_stabilisation = 0.2;

struct object graphics;
struct pendulum double_pendulum[2];
constraints cc;

int tick_count;
int solver_iterations = 3;

static float pendulum_angle_ex(Vector3 position, Vector3 anchor, float length) {
  Vector3 a = Vector3Subtract(position, anchor);
  float inner = -a.y / length;
  float angle = (a.x > 0 ? 1 : -1) * acosf(inner);

  return angle;
}

static Vector3 pendulum_position(Vector3 anchor, float length, float angle) {
  Vector3 offset = { sin(angle), -cos(angle), 0 };
  return Vector3Add(anchor, Vector3Scale(offset, length));
}

void initialize_program(program_config* config) {
  config->camera_mode = CAMERA_CUSTOM; 
  config->window_title = "Pendulums";
  config->camera_position = (Vector3) { 0, 5, 10 };
  config->camera_target = (Vector3) { 0, 3, 0 };
}

void setup_scene(Shader shader) {
  Mesh mesh = GenMeshSphere(0.1, 32, 32);

  Vector3 anchor = { 0, 5, 4 };
  Material material = LoadMaterialDefault();
  material.shader = shader;
  material.maps[MATERIAL_MAP_DIFFUSE].color = PURPLE;

  double_pendulum[0] = (struct pendulum) {
    .anchor = anchor,
    .body = rb_new(pendulum_position(anchor, string_length, initial_angle), default_mass),
    .string_length = string_length,
    .stabilization_factor = default_stabilisation,
  };
  double_pendulum[1] = (struct pendulum) {
    .anchor = Vector3Zero(),
    .body = rb_new(pendulum_position(double_pendulum[0].body.position, string_length, 0.5 * PI), default_mass),
    .string_length = string_length,
    .stabilization_factor = default_stabilisation,
  };

  cc = constraints_new(2, 2, 3, default_stabilisation);

  graphics = (struct object) { .label = "Double", .mesh = mesh, .material = material };
}

void save_state() {
}

void reset() {
  double_pendulum[0].body.position =
      pendulum_position(double_pendulum[0].anchor,
                        double_pendulum[0].string_length, initial_angle);
  double_pendulum[1].body.position = pendulum_position(
      double_pendulum[0].body.position, string_length, PI * 0.5);

  double_pendulum[0].body.linear_velocity =
    double_pendulum[1].body.linear_velocity = Vector3Zero();
}

void on_input() {
 
}

static void update_constraints(struct pendulum *main, struct pendulum *secondary) {
  Vector3 r[] = { Vector3Subtract(main->anchor, main->body.position), Vector3Subtract(main->body.position, secondary->body.position) };
  Vector3 n[] = { Vector3Normalize(r[0]), Vector3Normalize(r[1]) };
  float c[] = { Vector3Length(r[0]) - main->string_length, Vector3Length(r[1]) - secondary->string_length };
  float j1[] = { -n[0].x, -n[0].y, -n[0].z, 0, 0, 0 };
  float j2[] = { n[0].x, n[0].y, n[0].z, -n[1].x, -n[1].y, -n[1].z };
  Vector2 inv_mass = { 1.0f / main->body.mass, 1.0f / secondary->body.mass };
  Vector3 v[] = { main->body.linear_velocity, secondary->body.linear_velocity };

  memcpy(cc.errors, c, 2 * sizeof(float));
  memcpy(cc.j, j1, 6 * sizeof(float));
  memcpy(cc.j + 6, j2, 6 * sizeof(float));
  memcpy(cc.v, v, 6 * sizeof(float));

  cc.inv_m[0] = cc.inv_m[1] = cc.inv_m[2] = inv_mass.x;
  cc.inv_m[3] = cc.inv_m[4] = cc.inv_m[5] = inv_mass.y;
}

static void solve_double_pendulum(float dt) {
  struct pendulum *main = &double_pendulum[0];
  struct pendulum *secondary = &double_pendulum[1];

  update_constraints(main, secondary);
  constraints_solve(&cc, dt);

  Vector3 dv1 = *(Vector3*)cc.dv;
  Vector3 dv2 = *(Vector3*)(cc.dv + 3);
  main->body.linear_velocity = Vector3Add(main->body.linear_velocity, dv1);
  secondary->body.linear_velocity = Vector3Add(secondary->body.linear_velocity, dv2);
}

static void simulate_double_pendulum(float dt) {
  struct pendulum *main = &double_pendulum[0];
  struct pendulum *secondary = &double_pendulum[1];

  Vector3 v[] = { main->body.linear_velocity, secondary->body.linear_velocity };
  Vector3 dvv = Vector3Scale(GRAVITY_V, dt);

  for (int i = 0; i < 2; ++i) {
    double_pendulum[i].body.linear_velocity = Vector3Add(v[i], dvv);
  }

  for (int i = 0; i < solver_iterations; ++i) {
    solve_double_pendulum(dt);
  }

  main->body.position = Vector3Add(main->body.position, Vector3Scale(main->body.linear_velocity, dt));
  secondary->body.position = Vector3Add(secondary->body.position, Vector3Scale(secondary->body.linear_velocity, dt));
}

void simulate(float dt) {
  simulate_double_pendulum(dt);
}

void draw(float interpolation) {
  Vector3 anchor = double_pendulum[0].anchor;
  rigidbody r = double_pendulum[0].body;

  DrawSphere(anchor, 0.05, RED);

  DrawMesh(graphics.mesh, graphics.material, rb_transformation(&r));
  DrawMesh(graphics.mesh, graphics.material, rb_transformation(&double_pendulum[1].body));
}

static void draw_stat_float(struct nk_context* ctx, char* title, float value) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_value_float(ctx, title, value);
  nk_layout_row_end(ctx);
}

static void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s: (%.3f, %.3f, %.3f)", title, value.x, value.y, value.z);
  nk_layout_row_end(ctx);
}

void draw_ui(struct nk_context* ctx) {
    if (nk_begin_titled(ctx, "debug", "Debug", nk_rect(50, 50, 220, 550), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
      nk_layout_row_static(ctx, 30, 200, 1);
      nk_label(ctx, graphics.label, NK_TEXT_ALIGN_LEFT);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#length1", 0.1, &double_pendulum[0].string_length, 10, 0.5, 0.1);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#length2", 0.1, &double_pendulum[1].string_length, 10, 0.5, 0.1);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#mass1", 0.1, &double_pendulum[0].body.mass, 10, 0.5, 0.1);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#mass2", 0.1, &double_pendulum[1].body.mass, 10, 0.5, 0.1);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#comp1", 0, &double_pendulum[0].stabilization_factor, 1, 0.1, 0.01);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#comp2", 0, &double_pendulum[1].stabilization_factor, 1, 0.1, 0.01);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_int(ctx, "#iterations", 1, &solver_iterations, 10, 1, 0.5);
      nk_layout_row_end(ctx);

      float actual_distance_1 =  Vector3Length(Vector3Subtract(double_pendulum[0].anchor, double_pendulum[0].body.position));
      draw_stat_float(ctx, "Length 1", actual_distance_1);
      draw_stat_float(ctx, "Error 1", fabs(actual_distance_1 - double_pendulum[0].string_length));

      float actual_distance_2 =  Vector3Length(Vector3Subtract(double_pendulum[0].body.position, double_pendulum[1].body.position));
      draw_stat_float(ctx, "Length 2", actual_distance_2);
      draw_stat_float(ctx, "Error 2", fabs(actual_distance_2 - double_pendulum[1].string_length));
    }

  nk_end(ctx);
}
