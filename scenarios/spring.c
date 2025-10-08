#include "raylib.h"
#include "physics.h"
#include "raymath.h"

struct object {
  Mesh mesh;
  Material material;
  rigidbody body;
};

const int num_bodies = 3;

const float stiffness = 2;
const float initial_offset = 5;

struct object masses[num_bodies];

void setup() {
  Mesh cubeMesh = GenMeshCube(1, 1, 1);
  Image yellow = GenImageColor(32, 32, YELLOW);
  Image green = GenImageColor(32, 32, GREEN);
  Image blue = GenImageColor(32, 32, BLUE);

  Material m1 = LoadMaterialDefault();
  SetMaterialTexture(&m1, MATERIAL_MAP_DIFFUSE, LoadTextureFromImage(yellow));

  Material m2 = LoadMaterialDefault();
  SetMaterialTexture(&m2, MATERIAL_MAP_DIFFUSE, LoadTextureFromImage(green));

  Material m3 = LoadMaterialDefault();
  SetMaterialTexture(&m3, MATERIAL_MAP_DIFFUSE, LoadTextureFromImage(blue));
 
  masses[0] = (struct object) { cubeMesh, m1, rb_new((Vector3){initial_offset, 0.5, -2}, 1) };
  masses[1] = (struct object) { cubeMesh, m2, rb_new((Vector3){initial_offset, 0.5, 0}, 1) };
  masses[2] = (struct object) { cubeMesh, m3, rb_new((Vector3){initial_offset, 0.5, 2}, 1) };
}

void simulate(float dt) {
}

void draw() {
  for (int i = 0; i < num_bodies; ++i) {
    struct object m = masses[i];
    DrawMesh(m.mesh, m.material, rb_transformation(&m.body));
  }
}
