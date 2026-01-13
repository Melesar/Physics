#ifndef PHYSICS_H
#define PHYSICS_H

#include "raylib.h"
#include "stdbool.h"
#include "core.h"
#include <stddef.h>
#include <stdint.h>

#define GRAVITY 9.81f
#define GRAVITY_V (Vector3) { 0, -9.81f, 0 }

// ==== MATH  ======

#define cross(x, y) Vector3CrossProduct(x, y)
#define dot(x, y) Vector3DotProduct(x, y)
#define add(x, y) Vector3Add(x, y)
#define scale(x, y) Vector3Scale(x, y)
#define normalize(x) Vector3Normalize(x)
#define sub(x, y) Vector3Subtract(x, y)
#define len(x) Vector3Length(x)
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3) { 0, 1, 0 })
#define right() ((Vector3) { 1, 0, 0 })
#define forward() ((Vector3) { 0, 0, 1 })
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)

#define qadd(x, y) QuaternionAdd(x, y)
#define qscale(x, y) QuaternionScale(x, y)
#define qmul(x, y) QuaternionMultiply(x, y)
#define qnormalize(x) QuaternionNormalize(x)
#define qinvert(x) QuaternionInvert(x)
#define as_matrix(x) QuaternionToMatrix(x)
#define qidentity() QuaternionIdentity()

#define mul(x, y) MatrixMultiply(x, y)
#define transpose(x) MatrixTranspose(x)
#define translate(x, y, z) MatrixTranslate(x, y, z)

#define vlerp(x, y, t) Vector3Lerp(x, y, t)
#define lerp(x, y, t) Lerp(x, y, t)
#define slerp(x, y, t) QuaternionSlerp(x, y, t)

// ====== PHYSICS WORLD =======

typedef enum {
  SHAPE_BOX,
  SHAPE_PLANE,
} shape_type;

typedef struct {
  shape_type type;

  union {
    struct { Vector3 size; } box;
    struct { Vector3 normal; } plane;
  };

} body_shape;

typedef enum {
    BODY_DYNAMIC,
    BODY_STATIC,
} body_type;

typedef struct {
  Vector3 position;
  Quaternion rotation;
  Vector3 angular_momentum;
  float mass;
} body_initial_state;

typedef struct {
  Vector3 position;
  Quaternion rotation;
  body_shape shape;
  float mass;
} body_snapshot;

typedef struct {
  count_t dynamics_capacity;
  count_t statics_capacity;
  count_t collisions_capacity;
  float linear_damping;
  float angular_damping;
} physics_config;

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  Vector3* positions; \
  Quaternion* rotations; \
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
