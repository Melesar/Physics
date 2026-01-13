#include "raylib.h"
#include "physics.h"
#include "raymath.h"
#include "stdlib.h"
#include "string.h"
#include <stdlib.h>

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  Vector3* positions; \
  Quaternion* rotations; \
  body_shape* shapes; \
  float *inv_masses;

typedef struct {
  COMMON_FIELDS
} common_data;

struct physics_world {
  struct {
    COMMON_FIELDS

    Vector3 *angular_momenta;
    Matrix *inv_inertia_tensors;
  } dynamics;

  struct {
    COMMON_FIELDS
  } statics;
};

static Matrix inertia_tensor_matrix(Vector3 inertia) {
  Matrix tensor = { 0 };
  tensor.m0 = inertia.x;
  tensor.m5 = inertia.y;
  tensor.m10 = inertia.z;
  return tensor;
}

static Vector3 cylinder_inertia(float radius, float height, float mass) {
  float principal =  mass * (3 * radius * radius + height * height) / 12.0;
  return (Vector3){ principal, mass * radius * radius / 2.0, principal };
}

static Vector3 sphere_inertia(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;
  return scale(one(), scale);
}

static Vector3 box_inertia(Vector3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  Vector3 i = { yy + zz, xx + zz, xx + yy };
  return scale(i, m);
}

static common_data* as_common(physics_world *world, body_type type) {
  switch (type) {
    case BODY_DYNAMIC:
      return (common_data*) &world->dynamics;

      case BODY_STATIC:
      return (common_data*) &world->statics;
  }
}
static const common_data* as_common_const(const physics_world *world, body_type type) {
  return as_common((physics_world*) world, type);
}

physics_config default_physics_config() {
  return (physics_config) { .dynamics_capacity = 32, .statics_capacity = 8 };
}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  #define INIT_COMMONS(type, cap) \
    world->type.capacity = cap; \
    world->type.count = 0; \
    world->type.positions = malloc(sizeof(Vector3) * cap); \
    world->type.rotations = malloc(sizeof(Quaternion) * cap); \
    world->type.shapes = malloc(sizeof(body_shape) * cap); \
    world->type.inv_masses = malloc(sizeof(float) * cap);

  INIT_COMMONS(dynamics, config->dynamics_capacity);
  INIT_COMMONS(statics, config->statics_capacity);

  #undef INIT_COMMONS

  world->dynamics.angular_momenta = malloc(sizeof(Vector3) * config->dynamics_capacity);
  world->dynamics.inv_inertia_tensors = malloc(sizeof(Matrix) * config->dynamics_capacity);

  return world;
}

void add_physics_body(physics_world* world, body_type type, body_shape shape, body_initial_state state) {
  common_data *commons = as_common(world, type);
  if (commons->capacity < commons->count + 1) {
    commons->capacity = commons->capacity << 1;
    commons->positions = realloc(commons->positions, sizeof(Vector3) * commons->capacity);
    commons->rotations = realloc(commons->rotations, sizeof(Quaternion) * commons->capacity);
    commons->shapes = realloc(commons->shapes, sizeof(body_shape) * commons->capacity);
    commons->inv_masses = realloc(commons->inv_masses, sizeof(float) * commons->capacity);

    if (type == BODY_DYNAMIC) {
      world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(Vector3) * commons->capacity);
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(Matrix) * commons->capacity);
    }
  }

  count_t index = commons->count;
  commons->positions[index] = state.position;
  commons->rotations[index] = state.rotation;
  commons->inv_masses[index] = 1.0 / state.mass;

  if (type == BODY_DYNAMIC) {
    world->dynamics.angular_momenta[index] = state.angular_momentum;

    Matrix *inv_inertia_tensor = &world->dynamics.inv_inertia_tensors[index];
    switch (shape.type) {
      case SHAPE_BOX:
        *inv_inertia_tensor = MatrixInvert(inertia_tensor_matrix(box_inertia(shape.box.size, state.mass)));
        break;

      default:
        *inv_inertia_tensor = MatrixIdentity();
        break;
    }
  }

  commons->count += 1;
}

size_t body_count(const physics_world* world, body_type type) {
  return as_common_const(world, type)->count;
}

bool body(const physics_world* world, body_type type, size_t index, body_snapshot* body) {
  const common_data *commons = as_common_const(world, type);

  if (index >= commons->count)
    return false;

  body->position = commons->positions[index];
  body->rotation = commons->rotations[index];
  body->shape = commons->shapes[index];
  body->mass = 1.0 / commons->inv_masses[index];

  return true;
}

void physics_step(physics_world* world, float dt) {

}

void physics_teardown(physics_world* world) {
  #define TEARDOWN_COMMONS(type) \
    free(world->type.positions); \
    free(world->type.rotations); \
    free(world->type.shapes); \
    free(world->type.inv_masses);

  TEARDOWN_COMMONS(dynamics);
  TEARDOWN_COMMONS(statics);

  #undef TEARDOWN_COMMONS

  free(world->dynamics.angular_momenta);
  free(world->dynamics.inv_inertia_tensors);
  free(world);
}
