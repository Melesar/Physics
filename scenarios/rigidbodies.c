#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include "gizmos.h"
#include <math.h>

const float sphere_radius = 1;
const float cylinder_mass = 3;
const cylinder cylinder_shape = { .height = 2.5, .radius = 1 };

rigidbody sphere_body;
rigidbody cylinder_body;
struct object cylinder_graphics;
Mesh cylinder_mesh;
Model sphere_model;
Model cylinder_model;

float input_impulse_strength = 15;
float velocity_damping = 0.998;
float restitution_coeff = 0.7;
float baumgarde_coeff = 0.2;
float friction_coeff = 0.3;
int solver_iterations = 1;

const Vector3 initial_position = { 0, 4, 0 };
const Vector3 initial_moment_of_inertia = { 0, 0, 0 };
const Vector3 mesh_origin_offset = { 0, -0.5 * cylinder_shape.height, 0 };

static struct object generate_cylinder_graphics(Shader shader) {
  cylinder_mesh = GenMeshCylinder(cylinder_shape.radius, cylinder_shape.height, 32);
  Vector3 *vertices = (Vector3*)cylinder_mesh.vertices;
  for (int i = 0; i < cylinder_mesh.vertexCount; ++i) {
    vertices[i] = Vector3Add(vertices[i], mesh_origin_offset);
  }
  UpdateMeshBuffer(cylinder_mesh, RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION, vertices, cylinder_mesh.vertexCount * sizeof(Vector3), 0);

  Material m = LoadMaterialDefault();
  m.maps[MATERIAL_MAP_DIFFUSE].color = COLOR_GREEN_ACTIVE;
  m.shader = shader;

  return (struct object) {
    .mesh = cylinder_mesh,
    .material = m,
    .label = "Cylinder"
  };
}

void initialize_program(program_config* config) {
  config->window_title = "Rigidbodies";  
  config->camera_position = (Vector3) { 22.542, 11.645, 20.752 };
  config->camera_target = (Vector3) { 0, 0, 0 };
}

void setup_scene(Shader shader) {
  Vector3 inertia = cylinder_inertia_tensor(cylinder_shape, cylinder_mass);
  cylinder_body = rb_new(initial_position, cylinder_mass);
  cylinder_body.r = QuaternionFromEuler(0, 0, 30 * DEG2RAD);
  cylinder_body.i0_inv = Vector3Invert(inertia);
  cylinder_body.l = initial_moment_of_inertia;

  cylinder_graphics = generate_cylinder_graphics(shader);

  cylinder_model = LoadModelFromMesh(cylinder_mesh);
  cylinder_model.materials[0] = cylinder_graphics.material;

  Mesh sphere_mesh = GenMeshSphere(sphere_radius, 32, 32);
  sphere_model = LoadModelFromMesh(sphere_mesh);
  sphere_model.materials[0].shader = shader;

  inertia = sphere_inertia_tensor(sphere_radius, cylinder_mass);
  sphere_body = rb_new(add(initial_position, scale(up(), 3)), cylinder_mass);
  sphere_body.i0_inv = Vector3Invert(inertia);

  register_gizmo(&cylinder_body.p, &cylinder_body.r);
  register_gizmo(&sphere_body.p, &sphere_body.r);
}

void reset() {
  cylinder_body.l = cylinder_body.v = zero();
  cylinder_body.r = QuaternionFromEuler(0, 0, 30 * DEG2RAD);
  cylinder_body.p = initial_position;

  sphere_body.p = add(cylinder_body.p, scale(up(), 3));
  sphere_body.v = zero();
}

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

static void constraint_calculate_velocities(const constraint *c, float lambda, Vector3 *delta) {
  for (int i = 0; i < 2; ++i) {
    delta[i * 2] = add(delta[i * 2], scale(c->jacobian[i * 2], c->inv_mass[i] * lambda));

    Vector3 mm1 = { c->I[i].m0, c->I[i].m4, c->I[i].m8 };
    Vector3 mm2 = { c->I[i].m1, c->I[i].m5, c->I[i].m9 };
    Vector3 mm3 = { c->I[i].m2, c->I[i].m6, c->I[i].m10 };
    Vector3 mm = { dot(mm1, c->jacobian[i * 2 + 1]), dot(mm2, c->jacobian[i * 2 + 1]), dot(mm3, c->jacobian[i * 2 + 1]) };

    delta[i * 2 + 1] = add(delta[i * 2 + 1], scale(mm, lambda));
  }
}

static void contact_constraint_solve(rigidbody *rb_a, rigidbody *rb_b, Vector3 *omega_a, Vector3 *omega_b, const collision *collision, float dt) {
  Matrix inertias[] = { rb_inertia_world(rb_a), rb_inertia_world(rb_b) };
  float inv_masses[] = { 1.0 / rb_a->mass, 1.0 / rb_b->mass };
  
  Vector3 n = negate(collision->normal);
  Vector3 ra = collision->local_contact_a;
  Vector3 rb = collision->local_contact_b;

  constraint c = { 0 };
  c.jacobian[0] = negate(n);
  c.jacobian[1] = negate(cross(ra, n));
  c.jacobian[2] = n;
  c.jacobian[3] = cross(rb, n);

  c.inv_mass[0] = inv_masses[0];
  c.inv_mass[1] = inv_masses[1];

  c.v[0] = rb_a->v;
  c.v[1] = rb_b->v;

  c.omega[0] = *omega_a;
  c.omega[1] = *omega_b;

  c.I[0] = inertias[0];
  c.I[1] = inertias[1];

  Vector3 delta[4] = { 0 };
  float effective_mass, v_proj;
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float restitution = restitution_coeff * dot(add(sub(negate(rb_a->v), cross(*omega_a, collision->local_contact_a)), add(rb_b->v, cross(*omega_b, collision->local_contact_b))), n);
  float bias = -baumgarde_coeff * collision->depth / dt + restitution;
  float lambda_normal = fmaxf(-(v_proj + bias) / effective_mass, 0.0f);

  constraint_calculate_velocities(&c, lambda_normal, delta);

  c.jacobian[0] = negate(collision->tangent);
  c.jacobian[1] = negate(cross(ra, collision->tangent));
  c.jacobian[2] = collision->tangent;
  c.jacobian[3] = cross(rb, collision->tangent);
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float limit_friction = fabs(friction_coeff * lambda_normal);
  float lambda_tangent = -v_proj / effective_mass;
  lambda_tangent = fmaxf(-limit_friction, fminf(limit_friction, lambda_tangent));

  constraint_calculate_velocities(&c, lambda_tangent, delta);
  
  c.jacobian[0] = negate(collision->bitangent);
  c.jacobian[1] = negate(cross(ra, collision->bitangent));
  c.jacobian[2] = collision->bitangent;
  c.jacobian[3] = cross(rb, collision->bitangent);
  constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

  float lambda_bitangent = -v_proj / effective_mass;
  lambda_bitangent = fmaxf(-limit_friction, fmin(limit_friction, lambda_bitangent));

  constraint_calculate_velocities(&c, lambda_bitangent, delta);

  rb_a->v = add(rb_a->v, delta[0]);
  rb_a->l = add(rb_a->l, delta[1]);
  rb_b->v = add(rb_b->v, delta[2]);
  rb_b->l = add(rb_b->l, delta[3]);

  *omega_a = transform(rb_a->l, inertias[0]);
  *omega_b = transform(rb_b->l, inertias[1]);
}

void simulate(float dt) {
  const int num_bodies = 2;
  const int max_collisions = 2;
  
  rigidbody *bodies[] = { &sphere_body, &cylinder_body };
  Vector3 omegas[num_bodies];
  float inv_masses[num_bodies];
  Matrix inertias[num_bodies];


  for (int i = 0; i < num_bodies; ++i) {
    rigidbody *rb = bodies[i];

    rb_apply_force(rb, GRAVITY_V);

    rb->l = add(rb->l, rb->ti);
    rb->l = add(rb->l, scale(rb->t, dt));

    float inv_mass = 1.0 / rb->mass;
    Vector3 acc = scale(add(rb->fi, scale(rb->f, dt)), inv_mass);
    rb->v = add(rb->v, acc);

    inv_masses[i] = inv_mass;
    rb_angular_params(rb, &inertias[i], &omegas[i]);
  }

  const int max_cylinder_collisions = 6;
  collision cylinder_manifold[max_cylinder_collisions];
  int num_cylinder_collisions = cylinder_plane_contact_manifold(&cylinder_body, cylinder_shape.height, cylinder_shape.radius, zero(), up(), cylinder_manifold, max_cylinder_collisions);

  collision sphere_collision = sphere_plane_check_collision(&sphere_body, sphere_radius, zero(), up());

  collision shapes_manifold[3];
  int num_shape_collisions = cylinder_sphere_contact_manifold(&cylinder_body, &sphere_body, cylinder_shape.height, cylinder_shape.radius, sphere_radius, shapes_manifold, 3);

  rigidbody plane_dummy = {0};
  plane_dummy.mass = INFINITY;
  plane_dummy.r = QuaternionIdentity();

  Vector3 plane_omega;
  for (int iteration = 0; iteration < solver_iterations; ++iteration) {
    for(int i = 0; i < num_cylinder_collisions; ++i) {
      collision col = cylinder_manifold[i];
      if (!col.valid) {
        continue;
      }

      plane_omega = zero();
      contact_constraint_solve(&cylinder_body, &plane_dummy, &omegas[1], &plane_omega, &col, dt);
    }

    plane_omega = zero();
    if (sphere_collision.valid)
      contact_constraint_solve(&sphere_body, &plane_dummy, &omegas[0], &plane_omega, &sphere_collision, dt);

    for (int i = 0; i < num_shape_collisions; ++i) {
      if (shapes_manifold[i].valid)
        contact_constraint_solve(&cylinder_body, &sphere_body, &omegas[1], &omegas[0], &shapes_manifold[i], dt);
      
    }
    // if (shapes_collision.valid)
    //   contact_constraint_solve(bodies[1], bodies[0], &omegas[1], &omegas[0], &shapes_collision, dt);
  }

  for (int i = 0; i < num_bodies; ++i) {
    Vector3 omega = omegas[i];
    rigidbody *rb = bodies[i];

    Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
    Quaternion dq = qscale(qmul(q_omega, rb->r), 0.5 * dt);
    Quaternion q_orientation = qadd(rb->r, dq);
    rb->r = qnormalize(q_orientation);
    rb->t = rb->ti = zero();

    rb->p = add(rb->p, scale(rb->v, dt));
    rb->f = rb->fi = zero();

    rb->v = scale(rb->v, velocity_damping);
    rb->l = scale(rb->l, velocity_damping);
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
  cylinder_model.transform = rb_transformation(&cylinder_body);

  draw_model_with_wireframe(cylinder_model, Vector3Zero(), 1.0f, COLOR_GREEN_ACTIVE);
  draw_model_with_wireframe(sphere_model, sphere_body.p, 1.0f, COLOR_BLUE_STATIC);
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin_titled(ctx, "debug", "Debug", nk_rect(50, 50, 350, 550), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_label(ctx, cylinder_graphics.label, NK_TEXT_ALIGN_LEFT);

    draw_property_float(ctx, "#impulse_strength", &input_impulse_strength, 0, 100, 1, 0.5);
    draw_property_float(ctx, "#damping", &velocity_damping, 0.990, 0.999, 0.001, 0.001);
    draw_property_float(ctx, "#baumgarde", &baumgarde_coeff, 0, 1, 0.1, 0.05);
    draw_property_float(ctx, "#restitution", &restitution_coeff, 0, 1, 0.1, 0.05);
    draw_property_int(ctx, "#solver_iterations", &solver_iterations, 0, 50, 1, 1);
  }

  nk_end(ctx);
}

void save_state() { }

