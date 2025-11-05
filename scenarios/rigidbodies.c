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
  rb_simulate(&cylinder_body, dt);

  cylinder_collision = cylinder_plane_check_collision(&cylinder_body, cylinder_shape.height, cylinder_shape.radius, Vector3Zero(), (Vector3) { 0, 1, 0 });
  if (cylinder_collision.valid) {
    cylinder_body.p = add(cylinder_body.p, scale(cylinder_collision.normal, cylinder_collision.depth)); 
    rb_apply_impulse_at(&cylinder_body, cylinder_collision.world_contact_a, scale(cylinder_collision.normal, len(sphere_body.v)));
  }

  rb_apply_force(&sphere_body, GRAVITY_V);
  rb_simulate(&sphere_body, dt);

  sphere_collision = sphere_plane_check_collision(&sphere_body, sphere_radius, Vector3Zero(), (Vector3) { 0, 1, 0 });
  if (sphere_collision.valid) {
    sphere_body.p = add(sphere_body.p, scale(sphere_collision.normal, sphere_collision.depth)); 
    rb_apply_impulse_at(&sphere_body, sphere_collision.world_contact_a, scale(sphere_collision.normal, len(sphere_body.v)));
  }
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

