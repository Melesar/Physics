#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include "gizmos.h"

const float sphere_radius = 1;
const float cylinder_mass = 3;
const cylinder cylinder_shape = { .height = 2.5, .radius = 1 };

rigidbody sphere_body;
rigidbody cylinder_body;
struct object cylinder_graphics;
Mesh cylinder_mesh;

float input_impulse_strength = 15;
float velocity_damping = 0.993;

collision cylinder_collision = { 0 };
collision sphere_collision = { 0 };

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
  m.maps[MATERIAL_MAP_DIFFUSE].color = LIME;
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

  inertia = sphere_inertia_tensor(sphere_radius, cylinder_mass);
  sphere_body = rb_new((Vector3) {-5, 4, 0}, cylinder_mass);
  sphere_body.i0_inv = Vector3Invert(inertia);

  register_gizmo(&cylinder_body.p, &cylinder_body.r);
  register_gizmo(&sphere_body.p, &sphere_body.r);
}

void reset() {
  sphere_collision = (collision) { 0 };
  cylinder_collision = (collision) { 0 };

  cylinder_body.l = Vector3Zero();
}

void on_input(Camera *camera) {}

void simulate(float dt) {
  rb_apply_force(&cylinder_body, GRAVITY_V);

  rigidbody *rb = &cylinder_body;
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
  Matrix transform = mul(mul(orientation, inv_i0), transpose(orientation));
  Vector3 omega = transform(rb->l, transform);

  cylinder_collision = cylinder_plane_check_collision(&cylinder_body, cylinder_shape.height, cylinder_shape.radius, Vector3Zero(), (Vector3) { 0, 1, 0 });
  if (cylinder_collision.valid) {
    for (int i = 0; i < 1; i++) {
      Vector3 n = negate(cylinder_collision.normal);
      Vector3 j1 = negate(n);
      Vector3 j2 = negate(cross(cylinder_collision.local_contact_a, n));
    
      float m1 = transform.m0 * j2.x + transform.m1 * j2.y + transform.m2 * j2.z;
      float m2 = transform.m4 * j2.x + transform.m5 * j2.y + transform.m6 * j2.z;   
      float m3 = transform.m8 * j2.x + transform.m9 * j2.y + transform.m10 * j2.z;
      Vector3 m = { m1, m2, m3 };

      float effective_mass = inv_mass * dot(j1, j1) + dot(m, j2);

      float restitution = 0.7 * (dot(sub(negate(rb->v), cross(omega, cylinder_collision.local_contact_a)), n));
      float bias = -0.2 * cylinder_collision.depth / dt + restitution;
      float v_proj = dot(j1, rb->v) + dot(j2, omega);
      float lambda = -(v_proj + bias) / effective_mass; 

      Vector3 dv = scale(j1, inv_mass * lambda);
      Vector3 d_omega = scale(m, lambda);
                           
      rb->v = add(rb->v, dv);
      omega = add(omega, d_omega);
    }
  }


  Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
  Quaternion dq = qscale(qmul(q_omega, rb->r), 0.5 * dt);
  Quaternion q_orientation = qadd(rb->r, dq);
  rb->r = qnormalize(q_orientation);
  rb->t = rb->ti = zero();

  rb->p = add(rb->p, scale(rb->v, dt));
  rb->f = rb->fi = zero();
}

static void draw_collision(const collision *c) {
  if (!c->valid) return;

  DrawSphere(c->world_contact_a, 0.03f, GREEN);

  draw_arrow(c->world_contact_a, c->normal, RED);
  draw_arrow(c->world_contact_a, c->tangent, BLUE);
  draw_arrow(c->world_contact_a, c->bitangent, GREEN);
}

void draw(float interpolation) {
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, rb_transformation(&cylinder_body));
  DrawSphere(sphere_body.p, sphere_radius, GRAY);

  draw_collision(&cylinder_collision);
  draw_collision(&sphere_collision);
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

