#include "core.h"
#include "raylib.h"
#include "physics.h"
#include "math.h"
#include "raymath.h"
#include "string.h"
#include <stdlib.h>

#define START_RUNNING true
#define NUM_PENDULUMS 2

struct spring_pendulum {
  Vector3 anchor;
  float stiffness;
  float damping;
  rigidbody body;
};

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

struct object graphics[NUM_PENDULUMS];
struct spring_pendulum spring;
struct pendulum p;

oscillation_period periods[NUM_PENDULUMS];

int tick_count;
bool is_running;
bool step_forward;

static float pendulum_angle_ex(Vector3 position, Vector3 anchor, float length) {
  Vector3 a = Vector3Subtract(position, anchor);
  float inner = -a.y / length;
  float angle = (a.x > 0 ? 1 : -1) * acosf(inner);

  return angle;
}

static float pendulum_angle(struct pendulum* p) {
  return pendulum_angle_ex(p->body.position, p->anchor, p->string_length);
}

static float pendulum_height(struct pendulum* p) {
  Vector3 r = Vector3Subtract(p->body.position, p->anchor);
  return p->string_length + r.y;
}

static float pendulum_energy(struct pendulum* p) {
  float velocitySqr = Vector3LengthSqr(p->body.linear_velocity);
  return p->body.mass * (GRAVITY * pendulum_height(p) + 0.5 * velocitySqr);
}

static Vector3 pendulum_position(Vector3 anchor, float length, float angle) {
  Vector3 offset = { sin(angle), -cos(angle), 0 };
  return Vector3Add(anchor, Vector3Scale(offset, length));
}

void initialize_program(program_config* config) {
  is_running = START_RUNNING;
  step_forward = false;

  config->camera_mode = CAMERA_CUSTOM; 
  config->window_title = "Pendulums";
  config->camera_position = (Vector3) { 0, 5, 10 };
  config->camera_target = (Vector3) { 0, 3, 0 };
}

void setup_scene() {
  Mesh mesh = GenMeshSphere(0.3, 32, 32);

  Color colors[] = { YELLOW, BLUE };
  Vector3 anchors[] = { { 0, 5, 0  }, { 0, 5, 2 } };
  char* labels[] = { "Spring", "Rigid" };
  Material materials[NUM_PENDULUMS];

  materials[0] = LoadMaterialDefault();
  materials[0].maps[MATERIAL_MAP_DIFFUSE].color = colors[0];
  materials[1] = LoadMaterialDefault();
  materials[1].maps[MATERIAL_MAP_DIFFUSE].color = colors[1];

  p = (struct pendulum) {
     .anchor = anchors[0],
     .body = rb_new(pendulum_position(anchors[0], string_length, initial_angle), default_mass),
     .string_length = string_length,
     .stabilization_factor = 0.2,
  };

  spring = (struct spring_pendulum) {
    .anchor = anchors[1],
    .body = rb_new(pendulum_position(anchors[1], string_length, initial_angle), default_mass),
    .stiffness = default_stiffness,
    .damping = defualt_damping,
  };

  for (int i = 0; i < NUM_PENDULUMS; ++i) {
    graphics[i] = (struct object) { .label = labels[i], .mesh = mesh, .material = materials[i] };
    periods[i] = oscillation_period_new();
  }
}

void save_state() {
}

void process_inputs() {
  if (IsKeyPressed(KEY_SPACE)) {
    is_running = !is_running;
  }

  if (IsKeyDown(KEY_S)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) { 
    p.body.position = spring.body.position = pendulum_position(p.anchor, p.string_length, initial_angle);
    p.body.linear_velocity = spring.body.linear_velocity = (Vector3) { 0 };
  }
}

static Vector3 spring_acc(Vector3 position, Vector3 velocity, struct spring_pendulum* p) {
  Vector3 spring_acc = Vector3Scale(Vector3Subtract(position, p->anchor), p->stiffness);
  Vector3 damping_acc = Vector3Scale(velocity, p->damping);
  Vector3 gravity_acc = GRAVITY_V;

  Vector3 acc = Vector3Subtract(gravity_acc, spring_acc);
  acc = Vector3Subtract(acc, damping_acc);

  TraceLog(LOG_DEBUG, "Spring %f, damping %f, final %f", Vector3Length(spring_acc), Vector3Length(damping_acc), Vector3Length(acc));

  return acc;
}

static void rk4(struct spring_pendulum* p, float dt) {
  rigidbody* body = &p->body;

  Vector3 x0 = body->position;
  Vector3 v0 = body->linear_velocity;

  Vector3 dv1 = Vector3Scale(spring_acc(x0, v0, p),  dt);
  Vector3 dx1 = Vector3Scale(v0, dt);
  Vector3 dv2 = Vector3Scale(spring_acc(Vector3Add(x0, Vector3Scale(dx1, 0.5f)), Vector3Add(v0, Vector3Scale(dv1, 0.5f)), p), dt);
  Vector3 dx2 = Vector3Scale(Vector3Add(v0, Vector3Scale(dv1, 0.5f)), dt);
  Vector3 dv3 = Vector3Scale(spring_acc(Vector3Add(x0, Vector3Scale(dx2, 0.5f)),Vector3Add(v0, Vector3Scale(dv1, 0.5f)), p), dt);
  Vector3 dx3 = Vector3Scale(Vector3Add(v0, Vector3Scale(dv2, 0.5f)), dt);
  Vector3 dv4 = Vector3Scale(spring_acc(Vector3Add(x0, dx3),Vector3Add(v0, dv1), p), dt);
  Vector3 dx4 = Vector3Scale(Vector3Add(v0, dv3), dt);

  Vector3 dx = Vector3Scale(Vector3Add(dx1, Vector3Add(Vector3Scale(dx2, 2.0f), Vector3Add(Vector3Scale(dx3, 2.0f), dx4))), 1.0 / 6);
  Vector3 dv = Vector3Scale(Vector3Add(dv1, Vector3Add(Vector3Scale(dv2, 2.0f), Vector3Add(Vector3Scale(dv3, 2.0f), dv4))), 1.0 / 6);

  body->linear_velocity = Vector3Add(v0, dv);
  body->position = Vector3Add(x0, dx);
}

static void simulate_with_constraint(struct pendulum* p, float dt) {
  Vector3 acc = Vector3Scale(GRAVITY_V, 1.0 / p->body.mass);
  Vector3 dvv = Vector3Scale(acc, dt);
  
  rigidbody* body = &p->body;

  Vector3 x0 = body->position;
  Vector3 v0 = body->linear_velocity;

  Vector3 dx1 = Vector3Scale(v0, dt);
  Vector3 dx2 = Vector3Scale(Vector3Add(v0, Vector3Scale(dvv, 0.5f)), dt);
  Vector3 dx3 = Vector3Scale(Vector3Add(v0, Vector3Scale(dvv, 0.5f)), dt);
  Vector3 dx4 = Vector3Scale(Vector3Add(v0, dvv), dt);

  Vector3 dv = Vector3Scale(Vector3Add(dvv, Vector3Add(Vector3Scale(dvv, 2.0f), Vector3Add(Vector3Scale(dvv, 2.0f), dvv))), 1.0 / 6);

  body->linear_velocity = Vector3Add(v0, dv);

  Vector3 r = Vector3Subtract(p->anchor, body->position);
  Vector3 n = Vector3Normalize(r);
  Vector3 J = n;

  float invMass = 1.0 / body->mass;
  Vector3 mInv = Vector3Scale(Vector3One(), invMass);
  float error = fabs(Vector3Length(r) - p->string_length);
  float a = Vector3DotProduct(Vector3Scale(J, invMass), J);
  float b = -(Vector3DotProduct(J, body->linear_velocity) - error * p->stabilization_factor / dt);
  float lambda = b / a;

  dv = Vector3Scale(Vector3Scale(J, invMass), lambda);

  body->linear_velocity = Vector3Add(body->linear_velocity, dv);
  body->position = Vector3Add(body->position, Vector3Scale(body->linear_velocity, dt));
}

void simulate(float dt) {
  if (!is_running && !step_forward) {
    return;
  }

  rk4(&spring, dt);
  simulate_with_constraint(&p, dt);

  step_forward = false;
}

void draw(float interpolation) {
  for (int i = 0; i < NUM_PENDULUMS; ++i) {
    struct object obj = graphics[i];

    Vector3 anchor;
    rigidbody r;
    switch (i) {
      case 1:
        anchor = spring.anchor;
        r = spring.body;
        break;

      case 0:
        anchor = p.anchor;
        r = p.body;
        break;
    }

    DrawSphere(anchor, 0.05, RED);
    DrawMesh(obj.mesh, obj.material, rb_transformation(&r));
  }
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
      nk_label(ctx, graphics[0].label, NK_TEXT_ALIGN_LEFT);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#stiffness", 0, &spring.stiffness, 100, 1, 2);
      nk_layout_row_end(ctx);

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#damping", 0, &spring.damping, 10, 0.2, 0.1);
      nk_layout_row_end(ctx);

      nk_layout_row_static(ctx, 30, 200, 1);
      nk_label(ctx, graphics[1].label, NK_TEXT_ALIGN_LEFT);

      float actual_distance =  Vector3Length(Vector3Subtract(p.anchor, p.body.position));
      draw_stat_float(ctx, "Energy", pendulum_energy(&p));
      draw_stat_float(ctx, "Length", actual_distance);
      draw_stat_float(ctx, "Error", fabs(actual_distance - p.string_length));

      nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
      nk_layout_row_push(ctx, 0.1);
      nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
      nk_layout_row_push(ctx, 0.9);
      nk_property_float(ctx, "#correction", 0, &p.stabilization_factor, 1, 0.1, 0.05);
      nk_layout_row_end(ctx);
 
    }

  nk_end(ctx);
}
