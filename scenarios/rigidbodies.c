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
float velocity_damping = 0.993;

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
  sphere_body = rb_new((Vector3) {-5, 4, 0}, cylinder_mass);
  sphere_body.i0_inv = Vector3Invert(inertia);

  register_gizmo(&cylinder_body.p, &cylinder_body.r);
  register_gizmo(&sphere_body.p, &sphere_body.r);
}

void reset() {
  cylinder_body.l = Vector3Zero();
}

void on_input(Camera *camera) {}

typedef struct {
  float inv_mass;
  Vector3 j1, j2;
  Vector3 v, omega;
  Matrix I;
    
} constraint;

static constraint constraint_new(Vector3 n, Vector3 v, Vector3 ra, Vector3 omega, Matrix I, float inv_mass) {
  constraint c;
  c.j1 = negate(n);
  c.j2 = negate(cross(ra, n));
  c.inv_mass = inv_mass;
  c.v = v;
  c.omega = omega;
  c.I = I;

  return c;
}

static void constraint_update(constraint *c, Vector3 n, Vector3 ra) {
  c->j1 = negate(n);
  c->j2 = negate(cross(ra, n));
}

static void constraint_calculate_pre_lambda(const constraint *c, float *effective_mass, float *v_proj) {
  float m1 = c->I.m0 * c->j2.x + c->I.m1 * c->j2.y + c->I.m2 * c->j2.z;
  float m2 = c->I.m4 * c->j2.x + c->I.m5 * c->j2.y + c->I.m6 * c->j2.z;   
  float m3 = c->I.m8 * c->j2.x + c->I.m9 * c->j2.y + c->I.m10 *c-> j2.z;
  Vector3 m = { m1, m2, m3 };

  *effective_mass = c->inv_mass * dot(c->j1, c->j1) + dot(m, c->j2);
  *v_proj = dot(c->j1, c->v) + dot(c->j2, c->omega);
}

static void constraint_calculate_velocities(const constraint *c, float lambda, Vector3 *dv, Vector3 *dl) {
  *dv = add(*dv, scale(c->j1, c->inv_mass * lambda));
  Vector3 mm1 = { c->I.m0, c->I.m4, c->I.m8 };
  Vector3 mm2 = { c->I.m1, c->I.m5, c->I.m9 };
  Vector3 mm3 = { c->I.m2, c->I.m6, c->I.m10 };
  Vector3 mm = { dot(mm1, c->j2), dot(mm2, c->j2), dot(mm3, c->j2) };
  *dl = add(*dl, scale(mm, lambda));
}

void simulate(float dt) {
  const int num_bodies = 2;
  const int max_collisions = 2;
  
  rigidbody *bodies[] = { &sphere_body, &cylinder_body };
  Vector3 omegas[num_bodies];
  float inv_masses[num_bodies];
  Matrix inertias[num_bodies];
  collision collisions[max_collisions];

  for (int i = 0; i < num_bodies; ++i) {
    rigidbody *rb = bodies[i];

    rb_apply_force(rb, GRAVITY_V);

    rb->l = add(rb->l, rb->ti);
    rb->l = add(rb->l, scale(rb->t, dt));

    float inv_mass = 1.0 / rb->mass;
    Vector3 acc = scale(add(rb->fi, scale(rb->f, dt)), inv_mass);
    rb->v = add(rb->v, acc);

    Matrix inv_i0 = { 0 };
    inv_i0.m0 = rb->i0_inv.x;
    inv_i0.m5 = rb->i0_inv.y;
    inv_i0.m10 = rb->i0_inv.z;
    Matrix orientation = as_matrix(rb->r);
    Matrix inertia = mul(mul(orientation, inv_i0), transpose(orientation));
    Vector3 omega = transform(rb->l, inertia);

    inv_masses[i] = inv_mass;
    inertias[i] = inertia;
    omegas[i] = omega;
  }

  collisions[0] = sphere_plane_check_collision(&sphere_body, sphere_radius, zero(), up());
  collisions[1] = cylinder_plane_check_collision(&cylinder_body, cylinder_shape.height, cylinder_shape.radius, zero(), up());

  for(int i = 0; i < num_bodies; ++i) {
    collision col = collisions[i];
    if (!col.valid) {
      continue;
    }

    const float restitution_coeff = 0.7;
    const float baumgarde_coeff = 0.2;
    const float friction_coeff = 0.3;

    Vector3 n = negate(col.normal);
    constraint c = constraint_new(n, bodies[i]->v, col.local_contact_a, omegas[i], inertias[i], inv_masses[i]);

    Vector3 dv = zero();
    Vector3 dl = zero();
    float effective_mass, v_proj;
    constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

    float restitution = restitution_coeff * (dot(sub(negate(bodies[i]->v), cross(omegas[i], col.local_contact_a)), n));
    float bias = -baumgarde_coeff * col.depth / dt + restitution;
    float lambda_normal = fmaxf(-(v_proj + bias) / effective_mass, 0.0f);

    constraint_calculate_velocities(&c, lambda_normal, &dv, &dl);

    constraint_update(&c, col.tangent, col.local_contact_a);
    constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

    float limit_friction = fabs(friction_coeff * lambda_normal);
    float lambda_tangent = -v_proj / effective_mass;
    lambda_tangent = fmaxf(-limit_friction, fminf(limit_friction, lambda_tangent));

    constraint_calculate_velocities(&c, lambda_tangent, &dv, &dl);
    
    constraint_update(&c, col.bitangent, col.local_contact_a);
    constraint_calculate_pre_lambda(&c, &effective_mass, &v_proj);

    float lambda_bitangent = -v_proj / effective_mass;
    lambda_bitangent = fmaxf(-limit_friction, fmin(limit_friction, lambda_bitangent));

    constraint_calculate_velocities(&c, lambda_bitangent, &dv, &dl);

    rigidbody *rb = bodies[i];
    rb->v = add(rb->v, dv);
    rb->l = add(rb->l, dl);

    omegas[i] = transform(bodies[i]->l, inertias[i]);
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

    draw_stat_float3(ctx, "I0", Vector3Invert(cylinder_body.i0_inv));
    draw_stat_float3(ctx, "Omega", rb_angular_velocity(&cylinder_body));
    draw_stat_float3(ctx, "Momentum", cylinder_body.l);

    float det = MatrixDeterminant(QuaternionToMatrix(cylinder_body.r));
    draw_stat_float(ctx, "Det", det);
    draw_stat_matrix(ctx, "Inertia", rb_inertia_world(&cylinder_body));
  }

  nk_end(ctx);
}

void save_state() { }

