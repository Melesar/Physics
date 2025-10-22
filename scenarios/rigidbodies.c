#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"

const float cylinder_mass = 3;
const cylinder cylinder_shape = { .height = 2.5, .radius = 1 };

rigidbody cylinder_body;
struct object cylinder_graphics;

const Vector3 initial_position = { 0, 1.25, 0 };
const Vector3 initial_moment_of_inertia = { 0, 2, 5 };
const Vector3 mesh_origin_offset = { 0, -0.5 * cylinder_shape.height, 0 };

static struct object generate_cylinder_graphics(Shader shader) {
  Mesh cylinder_mesh = GenMeshCylinder(cylinder_shape.radius, cylinder_shape.height, 32);
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
  config->camera_mode = CAMERA_CUSTOM; 
  config->camera_position = (Vector3) { 22.542, 11.645, 20.752 };
  config->camera_target = (Vector3) { 0, 0, 0 };
}

void setup_scene(Shader shader) {
  Vector3 inertia = cylinder_inertia_tensor(cylinder_shape, cylinder_mass);
  cylinder_body = rb_new(initial_position, cylinder_mass);
  cylinder_body.i0_inv = Vector3Invert(inertia);
  cylinder_body.l = initial_moment_of_inertia;

  cylinder_graphics = generate_cylinder_graphics(shader);
}

void reset() {
  cylinder_body.p = initial_position;
  cylinder_body.r = QuaternionIdentity();
  cylinder_body.v = Vector3Zero();
  cylinder_body.l = initial_moment_of_inertia;
}

void simulate(float dt) {
  Vector3 omega = rb_angular_velocity(&cylinder_body);
  Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
  Quaternion dq = QuaternionScale(QuaternionMultiply(q_omega, cylinder_body.r), 0.5 * dt);

  cylinder_body.r = QuaternionNormalize(QuaternionAdd(cylinder_body.r, dq));
}

void draw(float interpolation) {
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, rb_transformation(&cylinder_body));

  draw_arrow(cylinder_body.p, cylinder_body.l, SKYBLUE);
  draw_arrow(cylinder_body.p, rb_angular_velocity(&cylinder_body), MAROON);
}

void draw_ui(struct nk_context* ctx) {
  
}

void save_state() { }

void on_input(Camera *camera) { }
