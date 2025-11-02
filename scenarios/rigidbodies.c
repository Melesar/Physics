#include "config.h"
#include "core.h"
#include "external/par_shapes.h"
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
bool is_collision = false;

Vector3 simplex[4];
int simplex_points_count = 0;
Vector3 direction = (Vector3) { 1, 0, 1 };
bool gjk_stop = false;

const Vector3 initial_position = { 0, 1.25, 0 };
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
  cylinder_body.i0_inv = Vector3Invert(inertia);
  cylinder_body.l = initial_moment_of_inertia;

  cylinder_graphics = generate_cylinder_graphics(shader);

  inertia = sphere_inertia_tensor(sphere_radius, cylinder_mass);
  sphere_body = rb_new((Vector3) {-2, 1, 0}, cylinder_mass);
  sphere_body.i0_inv = Vector3Invert(inertia);

  register_gizmo(&cylinder_body.p, &cylinder_body.r);
  register_gizmo(&sphere_body.p, &sphere_body.r);
}

void reset() {
  cylinder_body.p = initial_position;
  cylinder_body.r = QuaternionIdentity();
  cylinder_body.v = Vector3Zero();
  cylinder_body.l = initial_moment_of_inertia;
  gjk_stop = false;
}

typedef enum {
  SHAPE_CYLINDER,
  SHAPE_SPHERE,
} shape_type;


Vector3 shape_support(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b, Vector3 direction);
bool gjk_update_simplex(Vector3 *points, int *count, Vector3 *direction);

void on_input(Camera *camera) {
  if (gjk_stop || !IsKeyPressed(KEY_Y)) {
    return;
  }

  Vector3 support = shape_support(SHAPE_SPHERE, SHAPE_CYLINDER, &sphere_body, &cylinder_body, (Vector2) {sphere_radius, 0}, (Vector2) { cylinder_shape.height, cylinder_shape.radius }, direction);
  simplex[simplex_points_count++] = support;
  if (Vector3DotProduct(support, direction) < 0.001) {
    TraceLog(LOG_DEBUG, "GJK finished without collision");
    gjk_stop = true;
    return;    
  }

  if (gjk_update_simplex(simplex, &simplex_points_count, &direction)) {
    TraceLog(LOG_DEBUG, "GJK finished with collision");
    gjk_stop = true;
    return;
  }
  
}

void simulate(float dt) {
  // collision c = cylinder_sphere_check_collision(&cylinder_body, &sphere_body, cylinder_shape.height, cylinder_shape.radius, sphere_radius);
  // is_collision = c.valid;
  // rb_apply_force(&cylinder_body, GRAVITY_V);
  // rb_simulate(&cylinder_body, dt);

  // cylinder_body.l = Vector3Scale(cylinder_body.l, velocity_damping);
}

void draw(float interpolation) {
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, rb_transformation(&cylinder_body));
  DrawSphere(sphere_body.p, sphere_radius, is_collision ? GREEN : GRAY);

  for (int i = 0; i < simplex_points_count; ++i) {
    DrawSphere(simplex[i], 0.05, RED);
  }

  for (int i = 0; i < simplex_points_count - 1; ++i) {
    DrawLine3D(simplex[i], simplex[i + 1], GREEN);
  }

  if (simplex_points_count > 2) {
    DrawLine3D(simplex[simplex_points_count - 1], simplex[0], GREEN);
  }
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

