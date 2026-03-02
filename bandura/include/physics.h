#ifndef PHYSICS_H
#define PHYSICS_H

typedef struct physics_world_t physics_world;

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

body_handle make_body_handle(const physics_world *world, body_type type, count_t index);
count_t handle_to_inner_index(const physics_world *world, body_handle handle);

common_data* as_common(physics_world *world, body_type type);
const common_data* as_common_const(const physics_world *world, body_type type);
#endif
