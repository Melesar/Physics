#include "raylib.h"
#include "physics.h"
#include "core.h"
#include "math.h"
#include "raymath.h"
#include "string.h"

#define NUM_BODIES 4

const float stiffness = 8;
const float initial_offset = 5;

oscillation_period periods[NUM_BODIES];
struct object graphics[NUM_BODIES];
rigidbody prev_state[NUM_BODIES];
rigidbody masses[NUM_BODIES];

float total_time;
bool interpolate = true;

static float total_energy(int spring_index) {
  rigidbody spring = masses[spring_index];
  float velocity_sqr = Vector3LengthSqr(spring.linear_velocity);
  float kinetic = 0.5 * spring.mass * velocity_sqr;
  float elastic = 0.5 * stiffness * spring.position.x * spring.position.x;

  return kinetic + elastic;
}

void initialize_program(program_config* config) {
  config->camera_mode = CAMERA_CUSTOM;
  config->window_title = "Mass on a spring";
  config->camera_position = (Vector3) { 2, 7, 10 };
  config->camera_target = (Vector3) { 2, 0.5, 0 };
}

void setup_scene() {
  Mesh cubeMesh = GenMeshCube(1, 1, 1);

  char* labels[NUM_BODIES] = { "Exact", "Euler explicit", "Euler implicit", "RK4" };
  Color colors[NUM_BODIES] = { YELLOW, GREEN, BLUE, PURPLE }; 
  float offsets[NUM_BODIES] = { -2, 0, 2, 4 };

  for (int i = 0; i < NUM_BODIES; ++i) {
    Material m = LoadMaterialDefault();
    m.maps[MATERIAL_MAP_DIFFUSE].color = colors[i];

    graphics[i] = (struct object) { labels[i], cubeMesh, m };
    masses[i] = rb_new((Vector3){ initial_offset, 0.5, offsets[i] }, 1);
    periods[i] = oscillation_period_new();
  }

  save_state();
}

void save_state() {
  memcpy(prev_state, masses, sizeof(masses));
}

void process_inputs() {}

void simulate(float dt) {
  rigidbody* exact = &masses[0];
  float c = sqrtf(stiffness / exact->mass);
  exact->position.x = initial_offset * cosf(c * total_time);
  exact->linear_velocity.x = -c * initial_offset * sinf(c * total_time);

  rigidbody* euler_explicit = &masses[1];
  float acc = -stiffness * euler_explicit->position.x / euler_explicit->mass;
  euler_explicit->linear_velocity.x += acc * dt;
  euler_explicit->position.x += euler_explicit->linear_velocity.x * dt;

  rigidbody* euler_implicit = &masses[2];
  float omsq = stiffness / euler_implicit->mass;
  float x0 = euler_implicit->position.x;
  float v0 = euler_implicit->linear_velocity.x;
  float invDenom = 1 / (1 + dt * dt * omsq);
  float x = (x0 + dt * v0) * invDenom;
  float v = (v0 - dt * omsq * x0) * invDenom;

  euler_implicit->position.x = x;
  euler_implicit->linear_velocity.x = v;

  rigidbody* rk4 = &masses[3];
  x0 = rk4->position.x;
  v0 = rk4->linear_velocity.x;

  float omegasq = -stiffness / rk4->mass;
  float dv1 = dt * omegasq * x0;
  float dx1 = dt * v0;
  float dv2 = dt * omegasq * (x0 + 0.5 * dx1);
  float dx2 = dt * (v0 + 0.5 * dv1);
  float dv3 = dt * omegasq * (x0 + 0.5 * dx2);
  float dx3 = dt * (v0 + 0.5 * dv2);
  float dv4 = dt * omegasq * (x0 + dx3);
  float dx4 = dt * (v0 + dv3);

  float dx = (dx1 + 2 * dx2 + 2 * dx3 + dx4) / 6; 
  float dv = (dv1 + 2 * dv2 + 2 * dv3 + dv4) / 6;

  rk4->position.x += dx;
  rk4->linear_velocity.x += dv;
  
  total_time += dt;

  for (int i = 0; i < NUM_BODIES; ++i) {
    oscillation_period_track(&periods[i], &masses[i], &prev_state[i]);
  }
}

void draw(float interpolation) {
  for (int i = 0; i < NUM_BODIES; ++i) {
    rigidbody body;
    if (interpolate) {
      const rigidbody* current = &masses[i];
      const rigidbody* prev = &prev_state[i];

      body = rb_interpolate(current, prev, interpolation);
    } else {
      body = masses[i];
    }

    struct object obj = graphics[i];
    
    DrawMesh(obj.mesh, obj.material, rb_transformation(&body));
  }
}

static void spring_stats(struct nk_context* ctx, int index) {
  nk_layout_row_static(ctx, 15, 200, 1);
  nk_label(ctx, graphics[index].label, NK_TEXT_ALIGN_LEFT);

  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_value_float(ctx, "Energy", total_energy(index));
  nk_layout_row_end(ctx);

  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_value_float(ctx, "Period", periods[index].period);
  nk_layout_row_end(ctx);
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin_titled(ctx, "debug", "Debug", nk_rect(50, 50, 220, 350), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_radio_label(ctx, "Interpolation", &interpolate);
    nk_value_float(ctx, "Period", 2 * PI * sqrt(masses[0].mass / stiffness));

    for(int i = 0; i < NUM_BODIES; ++i) {
      spring_stats(ctx, i);
    }
  }

  nk_end(ctx);
}
