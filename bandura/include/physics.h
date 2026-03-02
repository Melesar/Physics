#ifndef PHYSICS_H
#define PHYSICS_H

typedef struct physics_world_t physics_world;
typedef struct collision_debug_state_t collision_debug_state;

#define LIB_BUILD

#include "bandura.h"
#include "stdbool.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
  count_t index_a, index_b;
  count_t contacts_offset, contacts_count;
} collision;

typedef struct {
  v3 point;
  v3 normal;
  float depth;

  m3 basis;
  v3 relative_position[2];
  v3 local_velocity;
  float desired_delta_velocity;
} contact;

#define ARRAY(type) \
  count_t type##s_capacity; \
  count_t type##s_count; \
  type* type##s;

typedef struct {
  ARRAY(collision)
  ARRAY(contact)

  count_t dynamic_collisions_count;
} collisions;

#undef ARRAY

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  v3* positions; \
  quat* rotations; \
  body_shape* shapes; \
  count_t *outer_lookup; \
  count_t *inner_lookup;

typedef struct {
  COMMON_FIELDS
} common_data;

typedef struct {
  COMMON_FIELDS

  // Forces
  v3 *forces;
  v3 *torques;
  v3 *impulses;
  v3 *angular_impulses;

  // Dynamics
  float *inv_masses;
  v3 *velocities;
  v3 *angular_momenta;
  m3 *inv_inertia_tensors;

  // Derived values.
  v3 *accelerations;
  m3 *inv_intertias;

  // Sleeping
  count_t awake_count;
  float *motion_avgs;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world_t {
  dynamic_bodies dynamics;
  static_bodies statics;

  collisions *collisions;

  physics_config config;

  count_t generation;
};

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

struct collision_debug_state_t {
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
};

body_handle make_body_handle(const physics_world *world, body_type type, count_t index);
count_t handle_to_inner_index(const physics_world *world, body_handle handle);

common_data* as_common(physics_world *world, body_type type);
const common_data* as_common_const(const physics_world *world, body_type type);

collisions* collisions_init(const physics_config *config);
void collisions_teardown(collisions *collisions);

void clear_forces(physics_world *world);
void integrate_bodies(physics_world *world, float dt);
void prepare_contacts(physics_world *world, float dt);
void resolve_interpenetration_contact(physics_world *world, count_t collision_index, const contact *contact, v3 *deltas);
void update_penetration_depths_ex(physics_world *world, count_t collision_index, const v3 *deltas, depth_update_record *records, count_t *record_count);
void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas);
bool find_worst_penetration(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
bool find_worst_velocity(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
void update_awake_status_for_collision(physics_world *world, count_t collision_index);
void update_velocity_deltas_ex(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt, velocity_update_record *records, count_t *record_count);
void resolve_collisions(physics_world *world, float dt);
void collisions_detect(collisions* collisions, const common_data *dynamics, const common_data *statics);
void update_awake_statuses(physics_world *world, float dt);

#endif
