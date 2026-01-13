#include "core.h"
#include "collisions.h"
#include "raylib.h"
#include "physics.h"
#include "raymath.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include <stdlib.h>

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  Vector3* positions; \
  Quaternion* rotations; \
  body_shape* shapes;

typedef struct {
  COMMON_FIELDS
} common_data;

typedef struct {
  COMMON_FIELDS

  float *inv_masses;
  Vector3 *velocities;
  Vector3 *angular_momenta;
  Matrix *inv_inertia_tensors;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world {
  dynamic_bodies dynamics;
  static_bodies statics;
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

physics_config physics_default_config() {
  return (physics_config) { .dynamics_capacity = 32, .statics_capacity = 8, .linear_damping = 0.997, .angular_damping = 0.997 };
}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  #define INIT_COMMONS(type, cap) \
    world->type.capacity = cap; \
    world->type.count = 0; \
    world->type.positions = malloc(sizeof(Vector3) * cap); \
    world->type.rotations = malloc(sizeof(Quaternion) * cap); \
    world->type.shapes = malloc(sizeof(body_shape) * cap); \

  INIT_COMMONS(dynamics, config->dynamics_capacity);
  INIT_COMMONS(statics, config->statics_capacity);

  #undef INIT_COMMONS

  world->dynamics.inv_masses = malloc(sizeof(float) * config->dynamics_capacity);
  world->dynamics.velocities = malloc(sizeof(Vector3) * config->dynamics_capacity);
  world->dynamics.angular_momenta = malloc(sizeof(Vector3) * config->dynamics_capacity);
  world->dynamics.inv_inertia_tensors = malloc(sizeof(Matrix) * config->dynamics_capacity);

  return world;
}

void physics_add_body(physics_world* world, body_type type, body_shape shape, body_initial_state state) {
  common_data *commons = as_common(world, type);
  if (commons->capacity < commons->count + 1) {
    commons->capacity = commons->capacity << 1;
    commons->positions = realloc(commons->positions, sizeof(Vector3) * commons->capacity);
    commons->rotations = realloc(commons->rotations, sizeof(Quaternion) * commons->capacity);
    commons->shapes = realloc(commons->shapes, sizeof(body_shape) * commons->capacity);

    if (type == BODY_DYNAMIC) {
      world->dynamics.inv_masses = realloc(world->dynamics.inv_masses, sizeof(float) * commons->capacity);
      world->dynamics.velocities = realloc(world->dynamics.velocities, sizeof(Vector3) * commons->capacity);
      world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(Vector3) * commons->capacity);
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(Matrix) * commons->capacity);
    }
  }

  count_t index = commons->count++;
  commons->positions[index] = state.position;
  commons->rotations[index] = state.rotation;

  if (type == BODY_DYNAMIC) {
    world->dynamics.inv_masses[index] = state.mass;
    world->dynamics.velocities[index] = zero();
    world->dynamics.angular_momenta[index] = state.angular_momentum;

    Matrix *inv_inertia_tensor = &world->dynamics.inv_inertia_tensors[index];
    switch (shape.type) {
      case SHAPE_BOX:
        *inv_inertia_tensor = inertia_tensor_matrix(Vector3Invert(box_inertia(shape.box.size, state.mass)));
        break;

      default:
        *inv_inertia_tensor = MatrixIdentity();
        break;
    }
  }
}

size_t physics_body_count(const physics_world* world, body_type type) {
  return as_common_const(world, type)->count;
}

bool physics_body(const physics_world* world, body_type type, size_t index, body_snapshot* body) {
  const common_data *commons = as_common_const(world, type);

  if (index >= commons->count)
    return false;

  body->position = commons->positions[index];
  body->rotation = commons->rotations[index];
  body->shape = commons->shapes[index];
  body->mass = type == BODY_DYNAMIC ? 1.0 / world->dynamics.inv_masses[index] : INFINITY;

  return true;
}

void physics_step(physics_world* world, float dt) {
  Vector3 gravity_acc = scale(GRAVITY_V, dt);
  float linear_damping = powf(world->linear_damping, dt);
  float angular_damping = powf(world->angular_damping, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->count; ++i) {
    dynamics->velocities[i] = add(dynamics->velocities[i], gravity_acc);
    dynamics->velocities[i] = scale(dynamics->velocities[i], linear_damping);

    dynamics->angular_momenta[i] = scale(dynamics->angular_momenta[i], angular_damping);

    Quaternion rotation = dynamics->rotations[i];
    Matrix orientation = as_matrix(rotation);
    Matrix inertia = mul(mul(orientation, dynamics->inv_inertia_tensors[i]), transpose(orientation));
    Vector3 omega = transform(dynamics->angular_momenta[i], inertia);

    Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
    Quaternion dq = qscale(qmul(q_omega, rotation), 0.5 * dt);

    Quaternion q_orientation = qadd(rotation, dq);

    dynamics->rotations[i] = qnormalize(q_orientation);
    dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
  }
}

void physics_teardown(physics_world* world) {
  #define TEARDOWN_COMMONS(type) \
    free(world->type.positions); \
    free(world->type.rotations); \
    free(world->type.shapes);

  TEARDOWN_COMMONS(dynamics);
  TEARDOWN_COMMONS(statics);

  #undef TEARDOWN_COMMONS

  free(world->dynamics.inv_masses);
  free(world->dynamics.velocities);
  free(world->dynamics.angular_momenta);
  free(world->dynamics.inv_inertia_tensors);

  free(world);
}
