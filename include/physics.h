#ifndef PHYSICS_H
#define PHYSICS_H

#include "stdbool.h"
#include "pmath.h"
#include "core.h"
#include <stddef.h>
#include <stdint.h>

#define GRAVITY 9.81f
#define GRAVITY_V (v3) { 0, -9.81f, 0 }

// ====== PHYSICS WORLD =======

typedef enum {
  SHAPE_BOX,
  SHAPE_SPHERE,
  SHAPE_PLANE,
} shape_type;

typedef struct {
  shape_type type;

  union {
    struct { v3 size; } box;
    struct { v3 normal; } plane;
    struct { float radius; } sphere;
  };

} body_shape;

typedef enum {
    BODY_DYNAMIC,
    BODY_STATIC,
} body_type;

typedef struct {
  v3 position;
  quat rotation;
  v3 angular_momentum;
  float mass;
} body_initial_state;

typedef struct {
  v3 position;
  quat rotation;
  body_shape shape;
  float mass;
} body_snapshot;

typedef struct {
  count_t dynamics_capacity;
  count_t statics_capacity;
  count_t collisions_capacity;
  float linear_damping;
  float angular_damping;
  float restitution;

  count_t max_resolution_iterations;
} physics_config;

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  v3* positions; \
  quat* rotations; \
  body_shape* shapes;

typedef struct {
  COMMON_FIELDS
} common_data;

struct physics_world;

typedef struct physics_world physics_world;

physics_config physics_default_config();

physics_world* physics_init(const physics_config *config);

void physics_add_body(physics_world* world, body_type type, body_shape shape, body_initial_state state);

size_t physics_body_count(const physics_world* world, body_type type);
bool physics_body(const physics_world* world, body_type type, size_t index, body_snapshot* body);

void physics_step(physics_world* world, float dt);

void physics_draw_collisions(const physics_world *world);
bool physics_has_collisions(const physics_world *world);

void physics_teardown(physics_world* world);

#endif
