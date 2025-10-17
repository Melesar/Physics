#include "core.h"
#include "physics.h"
#include "raylib.h"

void initialize_program(program_config* config) {
  config->window_title = "Rigidbodies";  
  config->camera_mode = CAMERA_CUSTOM; 
  config->camera_position = (Vector3) { 22.542, 11.645, 20.752 };
  config->camera_target = (Vector3) { 0, 0, 0 };
}

rigidbody cylinder;
struct object cylinder_graphics;

void setup_scene(Shader shader) {
  Mesh cylinder_mesh = GenMeshCylinder(1, 2.5, 32);
  Material m = LoadMaterialDefault();
  m.maps[MATERIAL_MAP_DIFFUSE].color = LIME;
  m.shader = shader;

  cylinder = rb_new((Vector3) { 0, 1, 0 }, 3);
  cylinder_graphics = (struct object) { .mesh = cylinder_mesh, .material = m, .label = "Cylinder" };
}

void save_state() {
  
}

void on_input() {
  
}

void reset() {}

void simulate(float dt) {
}

void draw(float interpolation) {
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, rb_transformation(&cylinder));
}

void draw_ui(struct nk_context* ctx) {
  
}
