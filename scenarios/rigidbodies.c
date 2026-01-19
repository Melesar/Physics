#include "collisions.h"
#include "core.h"
#include "gizmos.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

Color colors[] = { BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE,  BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE  };

const size_t num_materials = sizeof(colors) / sizeof(Color);
Material materials[20];

Mesh box_mesh;
Mesh sphere_mesh;

physics_world* world = NULL;

void initialize_program(program_config* config) {
  config->window_title = "Rigidbodies";
  config->camera_position = (v3) { 22.542, 11.645, 20.752 };
  config->camera_target = (v3) { 0, 0, 0 };
}

void setup_scene(Shader shader) {
  reset();

  box_mesh = GenMeshCube(1, 1, 1);
  sphere_mesh = GenMeshSphere(1, 16, 16);

  for (size_t i = 0; i < num_materials; ++i) {
    Material m = LoadMaterialDefault();
    m.shader = shader;
    m.maps[MATERIAL_MAP_ALBEDO].color = colors[i];

    materials[i] = m;
  }
}

void reset() {
  if (world)
    physics_teardown(world);

  physics_config config = physics_default_config();
  world = physics_init(&config);

  physics_add_body(world, BODY_STATIC, (body_shape){ .type = SHAPE_PLANE, .plane = { .normal = { 0, 1, 0 } } }, (body_initial_state){ 0 });
  physics_add_body(world, BODY_DYNAMIC, (body_shape) { .type = SHAPE_BOX, .box = { .size = { 1, 1, 1 } } }, (body_initial_state) { .position = { 0, 6, 0 }, .rotation = qidentity(), .angular_momentum = { 1, 1, 1 }, .mass = 3});
  physics_add_body(world, BODY_DYNAMIC, (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = 1 } }, (body_initial_state) { .position = { -1, 5, 4 }, .rotation = qidentity(), .mass = 2, .angular_momentum = zero() });
}

void on_input(Camera *camera) {}

void simulate(float dt) {
  physics_step(world, dt);
}

void draw(float interpolation) {
  size_t dynamic_count = physics_body_count(world, BODY_DYNAMIC);

  for (size_t i = 0; i < dynamic_count; ++i) {
    body_snapshot snapshot;
    if (!physics_body(world, BODY_DYNAMIC, i, &snapshot))
      continue;

    m4 scale;
    m4 transform = MatrixMultiply(QuaternionToMatrix(snapshot.rotation), MatrixTranslate(snapshot.position.x, snapshot.position.y, snapshot.position.z));
    Material material = materials[i % num_materials];

    switch (snapshot.shape.type) {
      case SHAPE_BOX:
        scale = MatrixScale(snapshot.shape.box.size.x, snapshot.shape.box.size.y, snapshot.shape.box.size.z);
        DrawMesh(box_mesh, material, mul(scale, transform));
        break;

      case SHAPE_SPHERE:
        scale = MatrixScale(snapshot.shape.sphere.radius, snapshot.shape.sphere.radius, snapshot.shape.sphere.radius);
        DrawMesh(sphere_mesh, material, mul(scale, transform));
        break;

      default:
        break;
    }
  }
}

void draw_ui(struct nk_context* ctx) {

}
