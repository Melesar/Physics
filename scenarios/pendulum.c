#include "core.h"
#include "raylib.h"
#include "physics.h"
#include "math.h"

#define START_RUNNING false
#define NUM_PENDULUMS 3

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
const float default_stabilisation = 0.2;

struct object graphics[NUM_PENDULUMS];
struct spring_pendulum spring;
struct pendulum p;
struct pendulum double_pendulum[2];

oscillation_period periods[NUM_PENDULUMS];

int tick_count;
int solver_iterations = 3;

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
  Mesh mesh = GenMeshSphere(0.1, 32, 32);

  Color colors[] = { YELLOW, BLUE, PURPLE };
  Vector3 anchors[] = { { 0, 5, 0  }, { 0, 5, 2 }, { 0, 5, 4 } };
  char* labels[] = { "Spring", "Rigid", "Double" };
  Material materials[NUM_PENDULUMS];

  materials[0] = LoadMaterialDefault();
  materials[0].maps[MATERIAL_MAP_DIFFUSE].color = colors[0];
  materials[1] = LoadMaterialDefault();
  materials[1].maps[MATERIAL_MAP_DIFFUSE].color = colors[1];
  materials[2] = LoadMaterialDefault();
  materials[2].maps[MATERIAL_MAP_DIFFUSE].color = colors[2];

  p = (struct pendulum) {
     .anchor = anchors[0],
     .body = rb_new(pendulum_position(anchors[0], string_length, initial_angle), default_mass),
     .string_length = string_length,
     .stabilization_factor = default_stabilisation,
  };

  spring = (struct spring_pendulum) {
    .anchor = anchors[1],
    .body = rb_new(pendulum_position(anchors[1], string_length, initial_angle), default_mass),
    .stiffness = default_stiffness,
    .damping = defualt_damping,
  };

  double_pendulum[0] = (struct pendulum) {
    .anchor = anchors[2],
    .body = rb_new(pendulum_position(anchors[2], string_length, initial_angle), default_mass),
    .string_length = string_length,
    .stabilization_factor = default_stabilisation,
  };
  double_pendulum[1] = (struct pendulum) {
    .anchor = Vector3Zero(),
    .body = rb_new(pendulum_position(double_pendulum[0].body.position, string_length, 0.5 * PI), default_mass),
    .string_length = string_length,
    .stabilization_factor = default_stabilisation,
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
    p.body.position = pendulum_position(p.anchor, p.string_length, initial_angle);
    spring.body.position = pendulum_position(spring.anchor, string_length, initial_angle);
    double_pendulum[0].body.position = pendulum_position(double_pendulum[0].anchor, double_pendulum[0].string_length, initial_angle);
    double_pendulum[1].body.position = pendulum_position(double_pendulum[0].body.position, string_length, PI * 0.5);

    p.body.linear_velocity =
      spring.body.linear_velocity =
        double_pendulum[0].body.linear_velocity =
          double_pendulum[1].body.linear_velocity = (Vector3) { 0 };
  }
}

static Vector3 spring_acc(Vector3 position, Vector3 velocity, struct spring_pendulum* p) {
  Vector3 spring_acc = Vector3Scale(Vector3Subtract(position, p->anchor), p->stiffness);
  Vector3 damping_acc = Vector3Scale(velocity, p->damping);
  Vector3 gravity_acc = GRAVITY_V;

  Vector3 acc = Vector3Subtract(gravity_acc, spring_acc);
  return Vector3Subtract(acc, damping_acc);
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

static void solve_double_pendulum(Vector2 inv_mass, float dt) {
  struct pendulum *main = &double_pendulum[0];
  struct pendulum *secondary = &double_pendulum[1];

  Vector3 r[] = { Vector3Subtract(main->anchor, main->body.position), Vector3Subtract(main->body.position, secondary->body.position) };
  Vector3 n[] = { Vector3Normalize(r[0]), Vector3Normalize(r[1]) };
  float c[] = { Vector3Length(r[0]) - main->string_length, Vector3Length(r[1]) - secondary->string_length };
  float j1[] = { -n[0].x, -n[0].y, -n[0].z, 0, 0, 0 };
  float j2[] = { n[0].x, n[0].y, n[0].z, -n[1].x, -n[1].y, -n[1].z };

  float beta[] = { main->stabilization_factor, secondary->stabilization_factor };
  float inv_m[] = { inv_mass.x, inv_mass.x, inv_mass.x, inv_mass.y, inv_mass.y, inv_mass.y };
  Vector3 v[] = { main->body.linear_velocity, secondary->body.linear_velocity };

  float jm[12];
  for (int i = 0; i < 6; ++i) {
    jm[i] = j1[i] * inv_m[i];
    jm[i + 6] = j2[i] * inv_m[i];
  }

  float a[4] = { 0 };
  for (int i = 0; i < 6; i++) {
    a[0] += jm[i] * j1[i];
    a[1] += jm[i] * j2[i];
    a[2] += jm[i + 6] * j1[i];
    a[3] += jm[i + 6] * j2[i];
  }

  float b[2] = { 0 };
  for (int i = 0; i < 6; ++i) {
    float vi = *((float*)v+i);
    b[0] += (j1[i] * vi);
    b[1] += (j2[i] * vi);
  }

  float inv_t = 1.0 / dt;
  b[0] = -(b[0] + beta[0] * c[0] * inv_t);
  b[1] = -(b[1] + beta[1] * c[1] * inv_t);

  float lambda[2];
  gauss_seidel_solve(a, b, lambda, 2);

  float dv[6];
  for (int i = 0; i < 6; ++i) {
    float jt_lambda = j1[i] * lambda[0] + j2[i] * lambda[1];
    dv[i] = jt_lambda * inv_m[i];
  }

  Vector3 dv1 = *(Vector3*)dv;
  Vector3 dv2 = *(Vector3*)(dv + 3);
  main->body.linear_velocity = Vector3Add(main->body.linear_velocity, dv1);
  secondary->body.linear_velocity = Vector3Add(secondary->body.linear_velocity, dv2);
}

static void simulate_double_pendulum(float dt) {
  struct pendulum *main = &double_pendulum[0];
  struct pendulum *secondary = &double_pendulum[1];
  
  Vector2 inv_mass = { 1.0 / main->body.mass, 1.0 / secondary->body.mass };

  Vector3 acc[] = { GRAVITY_V, GRAVITY_V };
  Vector3 dvv[] = { Vector3Scale(acc[0], dt), Vector3Scale(acc[1], dt) };
  Vector3 v[] = { main->body.linear_velocity, secondary->body.linear_velocity };

  Vector3 dv[2];
  for (int i = 0; i < 2; ++i) {
    double_pendulum[i].body.linear_velocity = Vector3Add(v[i], dvv[i]);
  }

  for (int i = 0; i < solver_iterations; ++i) {
    solve_double_pendulum(inv_mass, dt);
  }

  main->body.position = Vector3Add(main->body.position, Vector3Scale(main->body.linear_velocity, dt));
  secondary->body.position = Vector3Add(secondary->body.position, Vector3Scale(secondary->body.linear_velocity, dt));
}

void simulate(float dt) {
  if (!is_running && !step_forward) {
    return;
  }

  rk4(&spring, dt);
  simulate_with_constraint(&p, dt);
  simulate_double_pendulum(dt);

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

      case 2:
        anchor = double_pendulum[0].anchor;
        r = double_pendulum[0].body;

        DrawMesh(obj.mesh, obj.material, rb_transformation(&double_pendulum[1].body));
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
      nk_label(ctx, graphics[2].label, NK_TEXT_ALIGN_LEFT);

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
