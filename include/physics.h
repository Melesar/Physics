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
  v3* position;
  quat* rotation;
  v3* velocity;
  v3* angular_momentum;
} body;

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
  float friction;

  count_t max_resolution_iterations;

  float restitution_damping_limit;
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

void physics_add_plane(physics_world *world, v3 point, v3 normal);
body physics_add_box(physics_world *world, body_type type, float mass, v3 size);
body physics_add_sphere(physics_world *world, body_type type, float mass, float radius);

bool physics_has_collisions(const physics_world *world);
size_t physics_body_count(const physics_world* world, body_type type);
bool physics_body(const physics_world* world, body_type type, size_t index, body_snapshot* body);

void physics_step(physics_world* world, float dt);
void physics_awaken_body(physics_world* world, count_t index);
void physics_reset(physics_world *world);

void physics_draw_collisions(const physics_world *world);
void physics_draw_stats(const physics_world *world, struct nk_context* ctx);
void physics_draw_config_widget(physics_world *world, struct nk_context* ctx);

void physics_teardown(physics_world* world);

// ====== COLLISION DEBUGGING =======

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
  bool penetration_done;
  bool velocity_done;
  bool needs_integration; // true when a new frame needs physics integration
} collision_debug_state;

void physics_debug_state_init(collision_debug_state *state);
void physics_step_debug(physics_world *world, float dt, collision_debug_state *state);
void physics_draw_debug_widget(const physics_world *world, const collision_debug_state *state, struct nk_context *ctx);

#endif
