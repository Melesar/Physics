#ifndef BANDURA_H
#define BANDURA_H

#include <stdbool.h>
#include <stdint.h>

#define RAYMATH_DISABLE_CPP_OPERATORS
#include "raymath.h"

#define cross(x, y) Vector3CrossProduct(x, y)
#define dot(x, y) Vector3DotProduct(x, y)
#define add(x, y) Vector3Add(x, y)
#define scale(x, y) Vector3Scale(x, y)
#define normalize(x) Vector3Normalize(x)
#define sub(x, y) Vector3Subtract(x, y)
#define len(x) Vector3Length(x)
#define lensq(x) Vector3LengthSqr(x)
#define distance(x, y) Vector3Distance(x, y)
#define distancesqr(x, y) Vector3DistanceSqr(x, y)
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3) { 0, 1, 0 })
#define right() ((Vector3) { 1, 0, 0 })
#define forward() ((Vector3) { 0, 0, 1 })
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)
#define invert(x) Vector3Invert(x)

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
#define inverse(x) MatrixInvert(x)
#define m4identity(x) MatrixIdentity(x)

#define vlerp(x, y, t) Vector3Lerp(x, y, t)
#define lerp(x, y, t) Lerp(x, y, t)
#define slerp(x, y, t) QuaternionSlerp(x, y, t)

typedef Vector3 v3;
typedef Vector4 v4;
typedef Quaternion quat;
typedef Matrix m4;

typedef struct {
  float m0[3];  // Row 0
  float m1[3];  // Row 1
  float m2[3];  // Row 2
} m3;

m3 matrix_identity();
m3 matrix_transpose(m3 m);
m3 matrix_inverse(m3 m);
m3 matrix_add(m3 a, m3 b);
m3 matrix_multiply(m3 a, m3 b);
m3 matrix_negate(m3 m);
v3 matrix_rotate(v3 v, m3 m);
v3 matrix_rotate_inverse(v3 v, m3 m);
m3 matrix_from_basis(v3 x, v3 y, v3 z);
m3 matrix_skew_symmetric(v3 v);
m3 matrix_initial_inertia(v3 inertia);
m3 matrix_inertia(m3 initial_inertia, quat rotation);

typedef uint32_t count_t;

typedef enum {
  SHAPE_BOX,
  SHAPE_SPHERE,
  SHAPE_PLANE,

  SHAPES_COUNT
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
  count_t type: 1;
  count_t index: 31;
} body_handle;

typedef struct {
  v3* position;
  quat* rotation;
  v3* velocity;
  v3* angular_momentum;

  body_handle handle;
} body;

typedef struct {
  v3 point;
  v3 normal;
  float distance;
  body_handle body;
} raycast_hit;

typedef struct {
  v3 gravity;

  count_t dynamics_capacity;
  count_t statics_capacity;
  count_t collisions_capacity;

  float linear_damping;
  float angular_damping;
  float restitution;
  float friction;

  count_t max_penentration_iterations;
  count_t max_velocity_iterations;

  float penetration_epsilon;
  float velocity_epsilon;

  float sleep_base_bias;
  float sleep_threshold;

  float restitution_damping_limit;
} physics_config;

typedef void physics_world;

physics_config physics_default_config();

physics_world* physics_init(const physics_config *config);

void physics_add_plane(physics_world *world, v3 point, v3 normal);
body physics_add_box(physics_world *world, body_type type, float mass, v3 size);
body physics_add_sphere(physics_world *world, body_type type, float mass, float radius);

void physics_apply_force(physics_world *world, body_handle handle, v3 force);
void physics_apply_force_at(physics_world *world, body_handle handle, v3 force, v3 position);
void physics_apply_impulse(physics_world *world, body_handle handle, v3 impulse);
void physics_apply_impulse_at(physics_world *world, body_handle handle, v3 impulse, v3 position);

bool physics_get_shape(physics_world *world, body_handle handle, body_shape *shape);
bool physics_get_velocity(physics_world *world, body_handle handle, v3 *velocity);
bool physics_get_angular_velocity(physics_world *world, body_handle handle, v3 *angular_velocity);
bool physics_get_motion_avg(physics_world *world, body_handle handle, float *motion_avg);

void physics_step(physics_world* world, float dt);
void physics_awaken_body(physics_world* world, body_handle handle);
void physics_reset(physics_world *world);

count_t physics_raycast(physics_world *world, v3 origin, v3 direction, float max_distance, count_t max_hits, raycast_hit *hits);

void physics_teardown(physics_world* world);

#define CDBG_MAX_CONTACTS 64

typedef enum {
  CDBG_IDLE,
  CDBG_PENETRATION_RESOLVE,
  CDBG_DEPTH_UPDATE,
  CDBG_VELOCITY_RESOLVE,
  CDBG_VELOCITY_UPDATE,
  CDBG_DONE,
} collision_debug_phase;

typedef struct {
  count_t index;
  float before;
  float after;
} depth_update_record;

typedef struct {
  count_t index;
  v3 local_vel_before;
  v3 local_vel_after;
  float ddv_before;
  float ddv_after;
} velocity_update_record;

typedef struct {
  bool active;
  collision_debug_phase prev_phase;
  collision_debug_phase phase;
  count_t iteration;
  bool is_dynamic;

  count_t current_collision_index;
  count_t current_contact_index;

  // Deltas from resolve steps (body1 linear, body1 angular, body2 linear, body2 angular)
  v3 deltas[4];

  // Depth update records
  count_t depth_update_count;
  depth_update_record depth_updates[CDBG_MAX_CONTACTS];

  // Velocity update records
  count_t velocity_update_count;
  velocity_update_record velocity_updates[CDBG_MAX_CONTACTS];

  // Internal state carried between steps
  float dt;
} collision_debug_state;

void physics_debug_state_init(collision_debug_state *state);
void physics_step_debug(physics_world *world, float dt, collision_debug_state *state);
#endif
