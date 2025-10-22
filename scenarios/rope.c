#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"
#include "stdlib.h"
#include <string.h>

typedef struct {
  int num_links;
  float spacing;

  Vector3 *link_pos;
  Vector3 *link_vel;

  Vector3 anchor;
  rigidbody body;
  
} rope;

const int num_links = 10;
const float link_spacing = 0.8;
const float body_mass = 3;
const float link_mass = 0.5;
const float damping = 0.990;

const float initial_angle_degrees = 90.0;
const float stabilization = 0.2;
const int solver_iterations = 20;

rope r;
constraints cc;
bool body_fixed;

void initialize_program(program_config* config) {
  config->window_title = "Rope";
  config->camera_mode = CAMERA_CUSTOM;
  config->camera_position = (Vector3) { 0.609, 8.439, 22.771 };
  config->camera_target = (Vector3) { 0, 3, 0 };
}

void setup_scene(Shader shader) {
  float rope_len = num_links * link_spacing;
  r.num_links = num_links;
  r.spacing = link_spacing;
  r.link_pos = (Vector3*) malloc(num_links * sizeof(Vector3));
  r.link_vel = (Vector3*) malloc(num_links * sizeof(Vector3));
  r.anchor = (Vector3) { 0, rope_len + 2, 0 };

  cc = constraints_new(r.num_links + 1, r.num_links + 1, 3, stabilization, 50);

  reset();
}

void save_state() {
  
}

void on_input(Camera *camera) {
  Vector2 mouse_pos = GetMousePosition();
  Vector3 body_pos = r.body.position;
  Ray ray = GetScreenToWorldRay(mouse_pos, *camera);

  bool is_mouse_pressed = IsMouseButtonDown(0);
  if (!body_fixed && is_mouse_pressed) {
    RayCollision collision = GetRayCollisionSphere(ray, body_pos, 0.5);
    body_fixed = collision.hit;
  } else if (body_fixed && !is_mouse_pressed) {
    body_fixed = false;
  }

  if (!body_fixed)
    return;

  Vector3 forward = GetCameraForward(camera);
  Vector3 right = GetCameraRight(camera);
  Vector3 up = GetCameraUp(camera);
  Vector3 rr = Vector3Subtract(body_pos, camera->position);

  float distance = Vector3DotProduct(rr, forward);
  float mouse_distance = distance / Vector3DotProduct(forward, ray.direction);
  Vector3 world_mouse_position = Vector3Add(ray.position, Vector3Scale(ray.direction, mouse_distance));
  Vector3 delta = Vector3Subtract(world_mouse_position, r.anchor);
  delta = Vector3ClampValue(delta, r.spacing, (r.num_links + 1) * r.spacing);

  r.body.position = Vector3Add(r.anchor, delta);
}

void reset() {
  float angle = initial_angle_degrees * DEG2RAD;
  float distance = (r.num_links + 1) * link_spacing;

  r.body = rb_new(Vector3Add(r.anchor, (Vector3) { distance * sinf(angle), -distance * cosf(angle), 0 }), body_mass);

  for (int i = 0; i < num_links; ++i) {
    distance = (i + 1) * link_spacing;
    r.link_pos[i] = Vector3Add(r.anchor, (Vector3) { distance * sinf(angle), -distance * cosf(angle), 0 });
    r.link_vel[i] = Vector3Zero();
  }

  body_fixed = false;
}

static void setup_constraints() {
  int dim = r.num_links + 1;
  for (int i = 0; i < dim; ++i) {
    Vector3 p1 = i > 0 ? r.link_pos[i - 1] : r.anchor;
    Vector3 p2 = i < r.num_links ? r.link_pos[i] : r.body.position;

    cc.errors[i] = Vector3Distance(p1, p2) - r.spacing;

    Vector3 *vv = (Vector3*)cc.v;
    vv[i] = i < r.num_links ? r.link_vel[i] : r.body.linear_velocity;

    float inv_m = i < r.num_links ? 1.0 / link_mass : 1.0 / r.body.mass;
    Vector3 *mm = (Vector3*)cc.inv_m;
    mm[i] = Vector3Scale(Vector3One(), inv_m);
   
    Vector3 n = Vector3Normalize(Vector3Subtract(p1, p2));
    Vector3 nn = Vector3Negate(n);

    Vector3 *jj = (Vector3*)(cc.j + i * dim * 3);

    memset(jj, 0, dim * sizeof(Vector3));

    if (i > 0)
      jj[i - 1] = n;

    if (!body_fixed || i < dim - 1)
      jj[i] = nn;
  }
}

void simulate(float dt) {
  Vector3 dv = Vector3Scale(GRAVITY_V, dt);

  r.body.linear_velocity = !body_fixed ? Vector3Add(r.body.linear_velocity, dv) : Vector3Zero();
  for (int i = 0; i < r.num_links; ++i) {
    r.link_vel[i] = Vector3Add(r.link_vel[i], dv);
  }

  for (int i = 0; i < solver_iterations; ++i) {
    setup_constraints();
    constraints_solve(&cc, dt);

    for (int k = 0; k < r.num_links + 1; ++k) {
      Vector3 *v = k < r.num_links ? &r.link_vel[k] : &r.body.linear_velocity;
      Vector3 dv = *(Vector3*)(cc.dv + 3 * k);

      *v = Vector3Add(*v, dv);
    }
  }

  r.body.linear_velocity = Vector3Scale(r.body.linear_velocity, damping);
  for (int i = 0; i < r.num_links; ++i) {
    r.link_vel[i] = Vector3Scale(r.link_vel[i], damping);
  }

  for (int i = 0; i < r.num_links + 1; ++i) {
    Vector3 *p = i < r.num_links ? &r.link_pos[i] : &r.body.position;
    Vector3 v = i < r.num_links ? r.link_vel[i] : r.body.linear_velocity;

    *p = Vector3Add(*p, Vector3Scale(v, dt));
  }
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

