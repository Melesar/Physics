#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include "gizmos.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define NUM_BOXES 20
// #define NUM_BOXES 2

float velocity_damping = 0.998;

float restitution_coeff = 0.3;
float baumgarde_coeff = 0.1;

float kinetic_friction_coeff = 0.3;
float static_friction_coeff = 0.95;

float penetration_slop = 0.05;
float restitution_slop = 0.5;

int solver_iterations = 50;

const Vector3 box_size = { 1, 1, 1 };
const float box_mass = 3;
const int max_collisions = 3 * NUM_BOXES / 2 * (NUM_BOXES - 1) + 8 * NUM_BOXES;

rigidbody boxes[NUM_BOXES];
Material box_materials[NUM_BOXES];
Color colors[] = { BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE,  BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE  };
Mesh box_mesh;

collision *prev_collisions;
collision *collisions;

void initialize_program(program_config* config) {
  config->window_title = "Rigidbodies";  
  config->camera_position = (Vector3) { 22.542, 11.645, 20.752 };
  config->camera_target = (Vector3) { 0, 0, 0 };
}

static void create_box(rigidbody *box, int offset_x, int offset_y, int offset_z) {
  Vector3 half = scale(box_size, 0.5);

  *box = (rigidbody) { 0 };
  box->p = (Vector3) { (offset_x - 1) * box_size.x, (offset_z + 0.5) * box_size.y, (offset_y - 1) * box_size.z };
  box->r = QuaternionIdentity();
  // box->l = one();
  box->mass = box_mass;
  box->i0_inv = Vector3Invert(box_inertia_tensor(box_size, box_mass));
}

void setup_scene(Shader shader) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      create_box(&boxes[i * 3 + j], i, j, 0);
    }
  }

  for (int i = 0; i < 3; ++i) {
    create_box(&boxes[9 + i], 0, i, 1);
    create_box(&boxes[12 + i], 2, i, 1);
  }

  create_box(&boxes[15], 1, 0, 1);
  create_box(&boxes[16], 1, 1, 1);

  for (int i = 0; i < 3; ++i) {
    create_box(&boxes[17 + i], i, 0, 2);
  }

  box_mesh = GenMeshCube(1, 1, 1);

  for (int i = 0; i < NUM_BOXES; ++i) {
    Material m = LoadMaterialDefault();
    m.shader = shader;
    m.maps[MATERIAL_MAP_DIFFUSE].color = colors[i];

    box_materials[i] = m;
  }

  collisions = malloc(max_collisions * sizeof(collision));
  prev_collisions = malloc(max_collisions * sizeof(collision));

  memset(collisions, 0, max_collisions * sizeof(collision));
  memset(prev_collisions, 0, max_collisions * sizeof(collision));
}

void reset() {}

void on_input(Camera *camera) {}

typedef struct {
  float inv_mass[2];
  Vector3 jacobian[4];
  Vector3 v[2], omega[2];
  Matrix I[2];
    
} constraint;

static void constraint_calculate_pre_lambda(const constraint *c, float *effective_mass, float *v_proj) {
  *effective_mass = *v_proj = 0;
  for (int i = 0; i < 2; i++) {
    Vector3 j_linear = c->jacobian[i * 2];
    Vector3 j_radial = c->jacobian[i * 2 + 1];

    float m1 = c->I[i].m0 * j_radial.x + c->I[i].m1 * j_radial.y + c->I[i].m2 * j_radial.z;
    float m2 = c->I[i].m4 * j_radial.x + c->I[i].m5 * j_radial.y + c->I[i].m6 * j_radial.z;   
    float m3 = c->I[i].m8 * j_radial.x + c->I[i].m9 * j_radial.y + c->I[i].m10 *j_radial.z;
    Vector3 m = { m1, m2, m3 };

    *effective_mass += c->inv_mass[i] * dot(j_linear, j_linear) + dot(m, j_radial);
    *v_proj += dot(j_linear, c->v[i]) + dot(j_radial, c->omega[i]);
  }
}

static void constraint_calculate_impulses(const constraint *c, float lambda, Vector3 *delta) {
  for (int i = 0; i < 2; ++i) {
    delta[i * 2] = add(delta[i * 2], scale(c->jacobian[i * 2], c->inv_mass[i] * lambda));

    Vector3 mm1 = { c->I[i].m0, c->I[i].m4, c->I[i].m8 };
    Vector3 mm2 = { c->I[i].m1, c->I[i].m5, c->I[i].m9 };
    Vector3 mm3 = { c->I[i].m2, c->I[i].m6, c->I[i].m10 };
    Vector3 mm = { dot(mm1, c->jacobian[i * 2 + 1]), dot(mm2, c->jacobian[i * 2 + 1]), dot(mm3, c->jacobian[i * 2 + 1]) };

    delta[i * 2 + 1] = add(delta[i * 2 + 1], scale(mm, lambda));
  }
}

static void update_jacobian(constraint *c, Vector3 ra, Vector3 rb, Vector3 dir) {
  c->jacobian[0] = negate(dir);
  c->jacobian[1] = negate(cross(ra, dir));
  c->jacobian[2] = dir;
  c->jacobian[3] = cross(rb, dir);
}

static void contact_constraint_solve(rigidbody *rb_a, rigidbody *rb_b, collision *collision, float dt) {
  Matrix inertias[2];
  Vector3 omegas[2];

  rb_angular_params(rb_a, &inertias[0], &omegas[0]);
  rb_angular_params(rb_b, &inertias[1], &omegas[1]);
    
  float inv_masses[] = { 1.0 / rb_a->mass, 1.0 / rb_b->mass };
  
  Vector3 n = collision->normal;
  Vector3 ra = collision->local_contact_a;
  Vector3 rb = collision->local_contact_b;

  constraint c = { 0 };
  update_jacobian(&c, ra, rb, n);

  c.inv_mass[0] = inv_masses[0];
  c.inv_mass[1] = inv_masses[1];

  c.v[0] = rb_a->v;
  c.v[1] = rb_b->v;

  c.omega[0] = omegas[0];
  c.omega[1] = omegas[1];

  c.I[0] = inertias[0];
  c.I[1] = inertias[1];

  Vector3 delta[4] = { 0 };
  float effective_mass, v_proj;
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float closing_velocity = dot(sub(add(rb_a->v, cross(omegas[0], collision->local_contact_a)), add(rb_b->v, cross(omegas[1], collision->local_contact_b))), n);
  float restitution = restitution_coeff * fmax(closing_velocity - restitution_slop, 0);
  float bias = -baumgarde_coeff * fmax(collision->depth - penetration_slop, 0) / dt + restitution;
  float d_lambda_normal = -(v_proj + bias) / effective_mass;
  float new_lambda_normal = fmaxf(collision->pn + d_lambda_normal, 0.0);

  float d_lambda = new_lambda_normal - collision->pn;
  constraint_calculate_impulses(&c, d_lambda, delta);
  collision->pn = new_lambda_normal;

  update_jacobian(&c, ra, rb, collision->tangent);
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float friction_coef = fabs(closing_velocity) > 0.1 ? kinetic_friction_coeff : static_friction_coeff;
  float limit_friction = fabs(friction_coef * new_lambda_normal);
  float d_lambda_tangent = -v_proj / effective_mass;
  float new_lambda_tangent = fmaxf(-limit_friction, fminf(limit_friction, collision->pt + d_lambda_tangent));

  d_lambda = new_lambda_tangent - collision->pt;
  constraint_calculate_impulses(&c, d_lambda, delta);
  collision->pt = new_lambda_tangent;
  
  update_jacobian(&c, ra, rb, collision->bitangent);
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float d_lambda_bitangent = -v_proj / effective_mass;
  float new_lambda_bitangent = fmaxf(-limit_friction, fmin(limit_friction, collision->pb + d_lambda_bitangent));

  d_lambda = new_lambda_bitangent - collision->pb;
  constraint_calculate_impulses(&c, d_lambda, delta);
  collision->pb = new_lambda_bitangent;

  rb_a->v = add(rb_a->v, delta[0]);
  rb_a->l = add(rb_a->l, delta[1]);
  rb_b->v = add(rb_b->v, delta[2]);
  rb_b->l = add(rb_b->l, delta[3]);

  omegas[0] = transform(rb_a->l, inertias[0]);
  omegas[1] = transform(rb_b->l, inertias[1]);
}

static void warm_up_collisions() {
  const float distance_threshold_sqr = 0.0001;

  for (int i = 0; i < max_collisions; ++i) {
    collision *current = &collisions[i];
    if (!current->valid)
      return;

    for (int j = 0; j < max_collisions; ++j) {
      collision *prev = &prev_collisions[j];
      if (!prev->valid)
        break;

      if (Vector3DistanceSqr(prev->world_contact_a, current->world_contact_a) < distance_threshold_sqr &&
          Vector3DistanceSqr(prev->world_contact_b, current->world_contact_b) < distance_threshold_sqr) {
        current->pn = prev->pn;
        current->pt = prev->pt;
        current->pb = prev->pb;

        break;
      }
    }
  }
}

static void apply_cached_impulses() {
  for (int i = 0; i < max_collisions; ++i) {
    collision *col = &collisions[i];
    if (!col->valid)
      return;

    Vector3 delta[4] = { 0 };
    constraint c = { 0 };

    c.inv_mass[0] = 1.0 / col->body_a->mass;
    c.inv_mass[1] = 1.0 / col->body_b->mass;

    c.I[0] = rb_inertia_world(col->body_a);
    c.I[1] = rb_inertia_world(col->body_b);

    update_jacobian(&c, col->local_contact_a, col->local_contact_b, col->normal);
    constraint_calculate_impulses(&c, col->pn, delta);

    update_jacobian(&c, col->local_contact_a, col->local_contact_b, col->tangent);
    constraint_calculate_impulses(&c, col->pt, delta);

    update_jacobian(&c, col->local_contact_a, col->local_contact_b, col->bitangent);
    constraint_calculate_impulses(&c, col->pb, delta);
    
    col->body_a->v = add(col->body_a->v, delta[0]);
    col->body_a->l = add(col->body_a->l, delta[1]);
    col->body_b->v = add(col->body_b->v, delta[2]);
    col->body_b->l = add(col->body_b->l, delta[3]);
  }
}

void simulate(float dt) {
  collision* cs = collisions;
  collisions = prev_collisions;
  prev_collisions = cs;

  memset(collisions, 0, max_collisions * sizeof(collision));

  int offset = 0;
  for (int i = 0; i < NUM_BOXES; ++i) {
    for (int j = 0; j < i; ++j) {
      offset += box_box_contact_manifold(&boxes[i], &boxes[j], box_size, box_size, &collisions[offset], 3);
    }
  }

  for (int i = 0; i < NUM_BOXES; ++i) {
    offset += box_plane_contact_manifold(&boxes[i], box_size, zero(), up(), &collisions[offset], 8);
  }

  warm_up_collisions();
  apply_cached_impulses();

  for (int i = 0; i < NUM_BOXES; ++i) {
    boxes[i].v = add(boxes[i].v, scale(GRAVITY_V, dt));
  }
  
  for (int h = 0; h < solver_iterations; ++h) {
    for (int i = 0; i < max_collisions; ++i) {
      collision *c = &collisions[i];
      if (!c->valid)
        break;

      contact_constraint_solve(c->body_a, c->body_b, c, dt);
    }
  }

  for (int i = 0; i < NUM_BOXES; ++i) {
    rigidbody *box = &boxes[i];
    Vector3 omega = rb_angular_velocity(box);
    Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
    Quaternion dq = qscale(qmul(q_omega, box->r), 0.5 * dt);
    Quaternion orientation = qadd(box->r, dq);

    box->r = qnormalize(orientation);
    box->p = add(box->p, scale(box->v, dt));
    box->v = scale(box->v, velocity_damping);
    box->l = scale(box->l, velocity_damping);
  }
  
}

static void draw_collision(const collision *c) {
  if (!c->valid) return;

  // Fights with the grid rendering for some reason. Investigate.
  // DrawSphere(c->world_contact_a, 0.03f, GREEN);

  draw_arrow(c->world_contact_a, c->normal, RED);
  draw_arrow(c->world_contact_a, c->tangent, BLUE);
  draw_arrow(c->world_contact_a, c->bitangent, GREEN);
}

void draw(float interpolation) {
  for (int i = 0; i < NUM_BOXES; ++i) {
    DrawMesh(box_mesh, box_materials[i], rb_transformation(&boxes[i]));
  }
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin_titled(ctx, "debug", "Debug", nk_rect(50, 50, 350, 550), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {

    draw_property_float(ctx, "#damping", &velocity_damping, 0.990, 0.999, 0.001, 0.001);
    draw_property_float(ctx, "#baumgarde", &baumgarde_coeff, 0, 1, 0.1, 0.05);
    draw_property_float(ctx, "#restitution", &restitution_coeff, 0, 1, 0.1, 0.05);
    draw_property_int(ctx, "#solver_iterations", &solver_iterations, 0, 50, 1, 1);
  }

  nk_end(ctx);
}

void save_state() { }

