#include "raylib.h"
#include "physics.h"
#include "core.h"
#include "math.h"
#include "string.h"

struct object {
  Mesh mesh;
  Material material;
};

const int num_bodies = 3;

const float stiffness = 2;
const float initial_offset = 5;

struct object graphics[num_bodies];
rigidbody prev_state[num_bodies];
rigidbody masses[num_bodies];

float total_time;

void initialize_program(program_config* config) {
 
  config->camera_mode = CAMERA_CUSTOM;
  config->window_title = "Mass on a spring";
  config->camera_position = (Vector3) { 2, 7, 10 };
  config->camera_target = (Vector3) { 2, 0.5, 0 };
}

void setup_scene() {
  Mesh cubeMesh = GenMeshCube(1, 1, 1);

  Color colors[num_bodies] = { YELLOW, GREEN, BLUE }; 
  float offsets[num_bodies] = { -2, 0, 2 };

  for (int i = 0; i < num_bodies; ++i) {
    Material m = LoadMaterialDefault();
    m.maps[MATERIAL_MAP_DIFFUSE].color = colors[i];

    graphics[i] = (struct object) { cubeMesh, m };
    masses[i] = rb_new((Vector3){ initial_offset, 0.5, offsets[i] }, 1);
  }

  save_state();
}

void save_state() {
  memcpy(prev_state, masses, sizeof(masses));
}

void simulate(float dt) {
  rigidbody* exact = &masses[0];
  float x = initial_offset * cosf(sqrtf(stiffness / exact->mass) * total_time);
  exact->position.x = x;

  total_time += dt;
}

void draw(float interpolation) {
  for (int i = 0; i < num_bodies; ++i) {
    const rigidbody* current = &masses[i];
    const rigidbody* prev = &prev_state[i];

    rigidbody body = rb_interpolate(current, prev, interpolation);
    struct object obj = graphics[i];
    
    DrawMesh(obj.mesh, obj.material, rb_transformation(&body));
  }
}
