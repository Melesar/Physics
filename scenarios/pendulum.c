#include "core.h"
#include "raylib.h"
#include "physics.h"
#include "math.h"
#include "raymath.h"
#include "string.h"
#include <stdlib.h>

struct pendulum {
  Vector3 anchor;
  rigidbody body;

  float string_length;
};

const int num_pendulums = 2;
const float initial_angle = PI * 0.25;
const float string_length = 2;
const float default_mass = 1;

struct object graphics[num_pendulums];
struct pendulum pendulums[num_pendulums];
struct pendulum prev_state[num_pendulums];
oscillation_period periods[num_pendulums];

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

static Vector3 pendulum_acceleration_ex(Vector3 position, Vector3 velocity, struct pendulum* p) {
  Vector3 r = Vector3Subtract(position, p->anchor);
  float current_length = Vector3Length(r);
  Vector3 r_hat = Vector3Scale(r, 1.0f / current_length);
  
  Vector3 gravity_acc = (Vector3){ 0, -GRAVITY, 0 };
  
  float v_radial = Vector3DotProduct(velocity, r_hat);
  
  float v_tangent_sq = Vector3LengthSqr(velocity) - v_radial * v_radial;
  float centripetal = v_tangent_sq / p->string_length;
  
  float tension_magnitude = Vector3DotProduct(gravity_acc, r_hat) + centripetal;
  
  if (tension_magnitude < 0) tension_magnitude = 0;
  
  Vector3 tension = Vector3Scale(r_hat, tension_magnitude);
  
  return Vector3Add(gravity_acc, tension);
}

static Vector3 pendulum_acceleration(struct pendulum* p) {
  return pendulum_acceleration_ex(p->body.position, p->body.linear_velocity, p);
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

static void pendulum_normalize(struct pendulum* p) {
    Vector3 r = Vector3Subtract(p->body.position, p->anchor);
    float len = Vector3Length(r);

    if (len <= 0) {
      return;
    }

    rigidbody* body = &p->body;
    body->position = Vector3Add(p->anchor, Vector3Scale(r, p->string_length / len));

    Vector3 r_hat = Vector3Scale(r, 1.0f / len);
    body->linear_velocity = Vector3Subtract(body->linear_velocity, 
        Vector3Scale(r_hat, Vector3DotProduct(body->linear_velocity, r_hat)));
}

void initialize_program(program_config* config) {
  is_running = false;
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
  char* labels[] = { "EExp", "RK4" };

  for (int i = 0; i < num_pendulums; ++i) {
    Material m = LoadMaterialDefault();
    m.maps[MATERIAL_MAP_DIFFUSE].color = colors[i];

    pendulums[i] = (struct pendulum) {
       .anchor = anchors[i],
       .body = rb_new(pendulum_position(anchors[i], string_length, initial_angle), default_mass),
       .string_length = string_length,
    };

    graphics[i] = (struct object) { .label = labels[i], .mesh = mesh, .material = m };
    periods[i] = oscillation_period_new();
  }
}

void save_state() {
  memcpy(prev_state, pendulums, sizeof(pendulums));
}

void process_inputs() {
  if (IsKeyPressed(KEY_SPACE)) {
    is_running = !is_running;
  }

  if (IsKeyDown(KEY_S)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) {
    for (int i = 0; i < num_pendulums; ++i) {
      struct pendulum* p = &pendulums[i];
      p->body.position = pendulum_position(p->anchor, p->string_length, initial_angle);
      p->body.linear_velocity = (Vector3) { 0 };
    }
  }
}

static void euler_explicit(struct pendulum* p, float dt) {
    rigidbody* mass = &p->body;

    Vector3 acceleration = pendulum_acceleration(p);
    mass->linear_velocity = Vector3Add(mass->linear_velocity, Vector3Scale(acceleration, dt));    
    mass->position = Vector3Add(mass->position, Vector3Scale(mass->linear_velocity, dt));

    // pendulum_normalize(p);
}

static Vector3 rk4_acc(Vector3 position, Vector3 velocity, struct pendulum* p) {
  return pendulum_acceleration_ex(position, velocity, p);
}

static void rk4(struct pendulum* p, float dt) {
  rigidbody* body = &p->body;

  Vector3 x0 = body->position;
  Vector3 v0 = body->linear_velocity;

  Vector3 dv1 = Vector3Scale(rk4_acc(x0, v0, p),  dt);
  Vector3 dx1 = Vector3Scale(v0, dt);
  Vector3 dv2 = Vector3Scale(rk4_acc(Vector3Add(x0, Vector3Scale(dx1, 0.5f)), Vector3Add(v0, Vector3Scale(dv1, 0.5f)), p), dt);
  Vector3 dx2 = Vector3Scale(Vector3Add(v0, Vector3Scale(dv1, 0.5f)), dt);
  Vector3 dv3 = Vector3Scale(rk4_acc(Vector3Add(x0, Vector3Scale(dx2, 0.5f)),Vector3Add(v0, Vector3Scale(dv1, 0.5f)), p), dt);
  Vector3 dx3 = Vector3Scale(Vector3Add(v0, Vector3Scale(dv2, 0.5f)), dt);
  Vector3 dv4 = Vector3Scale(rk4_acc(Vector3Add(x0, dx3),Vector3Add(v0, dv1), p), dt);
  Vector3 dx4 = Vector3Scale(Vector3Add(v0, dv3), dt);

  Vector3 dx = Vector3Scale(Vector3Add(dx1, Vector3Add(Vector3Scale(dx2, 2.0f), Vector3Add(Vector3Scale(dx3, 2.0f), dx4))), 1.0 / 6);
  Vector3 dv = Vector3Scale(Vector3Add(dv1, Vector3Add(Vector3Scale(dv2, 2.0f), Vector3Add(Vector3Scale(dv3, 2.0f), dv4))), 1.0 / 6);

  body->linear_velocity = Vector3Add(v0, dv);
  body->position = Vector3Add(x0, dx);

  pendulum_normalize(p);
}

void simulate(float dt) {
  if (!is_running && !step_forward) {
    return;
  }

  euler_explicit(&pendulums[0], dt);
  rk4(&pendulums[1], dt);

  for (int i = 0; i < num_pendulums; i++) {
    oscillation_period_track(&periods[i], &pendulums[i].body, &prev_state[i].body);
  }

  step_forward = false;
}

void draw(float interpolation) {
  for (int i = 0; i < num_pendulums; ++i) {
    struct pendulum current = pendulums[i];
    struct pendulum prev = prev_state[i];
    struct object obj = graphics[i];

    DrawSphere(current.anchor, 0.05, RED);

    rigidbody actual = rb_interpolate(&prev.body, &current.body, interpolation);
    // DrawMesh(obj.mesh, obj.material, rb_transformation(&actual));
  
    draw_arrow(actual.position, Vector3Add(actual.position, actual.linear_velocity), RED);
    draw_arrow(actual.position, Vector3Add(actual.position, pendulum_acceleration(&current)), BLUE);
    draw_arrow(actual.position, Vector3Add(actual.position, (Vector3) { 0, -GRAVITY, 0 }), PURPLE);
    // draw_arrow(actual.position, Vector3Add(actual.position, ), PURPLE);

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
    nk_value_float(ctx, "Period", 2 * PI * sqrt(string_length / default_mass));
    nk_value_float(ctx, "Total energy", default_mass * string_length * (1 - cosf(initial_angle)) * GRAVITY);

    for(int i = 0; i < num_pendulums; ++i) {
      struct pendulum* p = &pendulums[i];

      nk_layout_row_static(ctx, 15, 200, 1);
      nk_label(ctx, graphics[i].label, NK_TEXT_ALIGN_LEFT);

      float angle = pendulum_angle(p);
      draw_stat_float(ctx, "Energy", pendulum_energy(p));
      draw_stat_float(ctx, "Period", periods[i].period);
      draw_stat_float(ctx, "a.y", -Vector3Subtract(p->body.position, p->anchor).y);
      draw_stat_float(ctx, "Angle", angle);
      draw_stat_float(ctx, "Height", pendulum_height(p));
      draw_stat_float3(ctx, "Velocity", p->body.linear_velocity);
      draw_stat_float3(ctx, "Acc", pendulum_acceleration(p));
    }
  }

  nk_end(ctx);
}
