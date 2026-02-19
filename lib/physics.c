#include "physics.h"
#include "pmath.h"
#include "stdlib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "assert.h"

const float min_velocity_threshold = 0.07f;
const float min_velocity_threshold_sqr = min_velocity_threshold * min_velocity_threshold;
const float sleep_threshold = 0.1f;
const float rwa_base_bias = 0.3f;

// static v3 cylinder_inertia(float radius, float height, float mass) {
//   float principal =  mass * (3 * radius * radius + height * height) / 12.0;
//   return (v3){ principal, mass * radius * radius / 2.0, principal };
// }

static v3 sphere_inertia(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;
  return scale(one(), scale);
}

static v3 box_inertia(v3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  v3 i = { yy + zz, xx + zz, xx + yy };
  return scale(i, m);
}

common_data* as_common(physics_world *world, body_type type) {
  switch (type) {
    case BODY_DYNAMIC:
      return (common_data*) &world->dynamics;

    case BODY_STATIC:
      return (common_data*) &world->statics;

    default:
      return NULL;
  }
}

// static void move_body(physics_world *world, count_t src_index, count_t dst_index);
static void swap_bodies(physics_world *world, count_t index_a, count_t index_b);

extern void clear_forces(physics_world *world);
extern void update_awake_statuses(physics_world *world, float dt);
extern void prepare_contacts(physics_world *world, float dt);
extern void resolve_interpenetration_contact(physics_world *world, count_t collision_index, const contact *contact, v3 *deltas);
extern void update_penetration_depths_ex(physics_world *world, count_t collision_index, const v3 *deltas, depth_update_record *records, count_t *record_count);
extern void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas);
extern bool find_worst_penetration(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
extern bool find_worst_velocity(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
extern void update_awake_status_for_collision(physics_world *world, count_t collision_index);
extern void update_velocity_deltas_ex(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt, velocity_update_record *records, count_t *record_count);
extern void resolve_collisions(physics_world *world, float dt);
extern collisions* collisions_init(const physics_config *config);
extern void collisions_detect(collisions* collisions, const common_data *dynamics, const common_data *statics);
extern void collisions_teardown(collisions *collisions);

physics_config physics_default_config() {
  return (physics_config) {
    .gravity = (v3) { 0, -9.81f, 0 },
    .dynamics_capacity = 32,
    .statics_capacity = 8,
    .collisions_capacity = 64,
    .linear_damping = 0.997,
    .angular_damping = 0.99,
    .restitution = 0.3,
    .friction = 0.3,
    .max_penentration_iterations = 10,
    .max_velocity_iterations = 20,
    .penetration_epsilon = 0.005,
    .velocity_epsilon = 0.1,
    .restitution_damping_limit = 0.2,
  };
}

body_handle make_body_handle(const physics_world *world, body_type type, count_t index) {
  return (body_handle) {
    .type = type,
    .index = type == BODY_DYNAMIC ? world->dynamics.inner_lookup[index] : index,
  };
}

count_t handle_to_inner_index(const physics_world *world, body_handle handle) {
  return handle.type == BODY_DYNAMIC ? world->dynamics.outer_lookup[handle.index] : handle.index;
}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  #define INIT_COMMONS(type, cap) \
    world->type.capacity = cap; \
    world->type.count = 0; \
    world->type.positions = malloc(sizeof(v3) * cap); \
    world->type.rotations = malloc(sizeof(quat) * cap); \
    world->type.shapes = malloc(sizeof(body_shape) * cap); \
    world->type.outer_lookup = malloc(sizeof(count_t) * cap); \
    world->type.inner_lookup = malloc(sizeof(count_t) * cap); \

  INIT_COMMONS(dynamics, config->dynamics_capacity);
  INIT_COMMONS(statics, config->statics_capacity);

  #undef INIT_COMMONS

  const count_t vectors = sizeof(v3) * config->dynamics_capacity;
  const count_t floats = sizeof(float) * config->dynamics_capacity;
  const count_t matrices = sizeof(m3) * config->dynamics_capacity;

  world->dynamics.forces = malloc(vectors);
  world->dynamics.torques = malloc(vectors);
  world->dynamics.impulses = malloc(vectors);
  world->dynamics.angular_impulses = malloc(vectors);
  world->dynamics.accelerations = malloc(vectors);

  world->dynamics.inv_masses = malloc(floats);
  world->dynamics.velocities = malloc(vectors);
  world->dynamics.angular_momenta = malloc(vectors);
  world->dynamics.inv_inertia_tensors = malloc(matrices);
  world->dynamics.inv_intertias = malloc(matrices);
  world->dynamics.motion_avgs = malloc(floats);
  world->dynamics.awake_count = 0;

  world->config = *config;
  world->collisions = collisions_init(config);

#ifdef DIAGNOSTICS
  const count_t percentiles_buffer_size = 10000;
  world->diagnostics.penetration_depth = percentiles_init(percentiles_buffer_size);
  world->diagnostics.velocity_deltas = percentiles_init(percentiles_buffer_size);
  world->diagnostics.unresolved_penetrations = 0;
  world->diagnostics.unresolved_velocities = 0;
  world->diagnostics.frames_simulated = 0;
#endif

  return world;
}

static body physics_add_body(physics_world* world, body_type type, body_shape shape, float mass) {
  common_data *commons = as_common(world, type);
  if (commons->capacity < commons->count + 1) {
    commons->capacity = commons->capacity << 1;
    commons->positions = realloc(commons->positions, sizeof(v3) * commons->capacity);
    commons->rotations = realloc(commons->rotations, sizeof(quat) * commons->capacity);
    commons->shapes = realloc(commons->shapes, sizeof(body_shape) * commons->capacity);

    if (type == BODY_DYNAMIC) {
      world->dynamics.forces = realloc(world->dynamics.forces, sizeof(v3) * commons->capacity);
      world->dynamics.torques = realloc(world->dynamics.torques, sizeof(v3) * commons->capacity);
      world->dynamics.impulses = realloc(world->dynamics.impulses, sizeof(v3) * commons->capacity);
      world->dynamics.angular_impulses = realloc(world->dynamics.angular_impulses, sizeof(v3) * commons->capacity);

      world->dynamics.inv_masses = realloc(world->dynamics.inv_masses, sizeof(float) * commons->capacity);
      world->dynamics.velocities = realloc(world->dynamics.velocities, sizeof(v3) * commons->capacity);
      world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(v3) * commons->capacity);
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(m3) * commons->capacity);
      world->dynamics.inv_intertias = realloc(world->dynamics.inv_intertias, sizeof(m3) * commons->capacity);
      world->dynamics.motion_avgs = realloc(world->dynamics.motion_avgs, sizeof(float) * commons->capacity);
    }
  }

  count_t index = commons->count++;
  commons->shapes[index] = shape;
  commons->positions[index] = zero();
  commons->rotations[index] = qidentity();
  commons->outer_lookup[index] = index;
  commons->inner_lookup[index] = index;

  if (type == BODY_DYNAMIC) {
    world->dynamics.inv_masses[index] = 1.0 / mass;
    world->dynamics.velocities[index] = zero();
    world->dynamics.angular_momenta[index] = zero();
    world->dynamics.motion_avgs[index] = 0;
    world->dynamics.forces[index] = zero();
    world->dynamics.torques[index] = zero();
    world->dynamics.impulses[index] = zero();
    world->dynamics.angular_impulses[index] = zero();
    world->dynamics.accelerations[index] = zero();

    v3 inertia_vector = one();
    switch (shape.type) {
      case SHAPE_BOX:
        inertia_vector = box_inertia(shape.box.size, mass);
        break;

      case SHAPE_SPHERE:
        inertia_vector = sphere_inertia(shape.sphere.radius, mass);
        break;

      default:
        break;
    }
    world->dynamics.inv_inertia_tensors[index] = matrix_initial_inertia(invert(inertia_vector));
  }

  return (body) {
    .position = &commons->positions[index],
    .rotation = &commons->rotations[index],
    .velocity = type == BODY_DYNAMIC ? &world->dynamics.velocities[index] : NULL,
    .angular_momentum = type == BODY_DYNAMIC ? &world->dynamics.angular_momenta[index] : NULL,
    .handle = make_body_handle(world, type, index),
  };
}

void physics_add_plane(physics_world *world, v3 point, v3 normal) {
  physics_add_body(world, BODY_STATIC, (body_shape) { .type = SHAPE_PLANE, .plane = { .normal = normal } }, INFINITY);
  world->statics.positions[world->statics.count - 1] = point;
}

body physics_add_box(physics_world *world, body_type type, float mass, v3 size) {
  return physics_add_body(world, type, (body_shape) { .type = SHAPE_BOX, .box = { .size = size } }, mass);
}

body physics_add_sphere(physics_world *world, body_type type, float mass, float radius) {
  return physics_add_body(world, type, (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = radius } }, mass);
}

void physics_apply_force(physics_world *world, body_handle handle, v3 force) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.forces[index];

  world->dynamics.forces[index] = add(prev_force, force);
}

void physics_apply_force_at(physics_world *world, body_handle handle, v3 force, v3 position) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.forces[index];
  v3 prev_torque = world->dynamics.torques[index];

  v3 torque = cross(position, force);

  world->dynamics.forces[index] = add(prev_force, force);
  world->dynamics.torques[index] = add(prev_torque, torque);
}

void physics_apply_impulse(physics_world *world, body_handle handle, v3 impulse) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_impulse = world->dynamics.impulses[index];

  world->dynamics.impulses[index] = add(prev_impulse, impulse);
}

void physics_apply_impulse_at(physics_world *world, body_handle handle, v3 impulse, v3 position) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.impulses[index];
  v3 prev_angular_impulse = world->dynamics.angular_impulses[index];

  v3 angular_impulse = cross(position, impulse);

  world->dynamics.impulses[index] = add(prev_force, impulse);
  world->dynamics.angular_impulses[index] = add(prev_angular_impulse, angular_impulse);
}


bool physics_get_shape(physics_world *world, body_handle handle, body_shape *shape) {
  body_type type = handle.type;
  common_data *data = as_common(world, type);

  if (handle.index >= data->count)
    return false;

  *shape = data->shapes[handle.index];
  return true;
}

bool physics_get_velocity(physics_world *world, body_handle handle, v3 *velocity) {
  if (handle.type != BODY_DYNAMIC) {
    *velocity = zero();
    return true;
  }

  dynamic_bodies *dynamics = &world->dynamics;
  if (handle.index >= dynamics->count)
    return false;

  *velocity = dynamics->velocities[handle_to_inner_index(world, handle)];
  return true;
}

bool physics_get_angular_velocity(physics_world *world, body_handle handle, v3 *angular_velocity) {
  if (handle.type != BODY_DYNAMIC) {
    *angular_velocity = zero();
    return true;
  }

  dynamic_bodies *dynamics = &world->dynamics;
  if (handle.index >= dynamics->count)
    return false;

  count_t index = handle_to_inner_index(world, handle);
  v3 angular_momentum = dynamics->angular_momenta[index];
  m3 inv_inertia = dynamics->inv_intertias[index];

  *angular_velocity = matrix_rotate(angular_momentum, inv_inertia);
  return true;
}

bool physics_get_motion_avg(physics_world *world, body_handle handle, float *motion_avg) {
  if (handle.type != BODY_DYNAMIC) {
    *motion_avg = 0.0f;
    return true;
  }

  dynamic_bodies *dynamics = &world->dynamics;
  if (handle.index >= dynamics->count)
    return false;

  *motion_avg = dynamics->motion_avgs[handle_to_inner_index(world, handle)];
  return true;
}

void integrate_bodies(physics_world *world, float dt) {
  v3 gravity_acc = scale(world->config.gravity, dt);
  float linear_damping = world->config.linear_damping;
  float angular_damping = world->config.angular_damping;

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    float inv_mass = dynamics->inv_masses[i];

    v3 acceleration = scale(dynamics->forces[i], inv_mass * dt);
    acceleration = add(acceleration, scale(dynamics->impulses[i], inv_mass));
    acceleration = add(acceleration, gravity_acc);

    v3 velocity = dynamics->velocities[i];
    velocity = add(velocity, acceleration);
    velocity = scale(velocity, linear_damping);

    quat rotation = dynamics->rotations[i];
    m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);

    v3 momentum_delta = scale(dynamics->torques[i], dt);
    momentum_delta = add(momentum_delta, dynamics->angular_impulses[i]);

    v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = add(angular_momentum, momentum_delta);
    angular_momentum = scale(angular_momentum, angular_damping);

    v3 omega = matrix_rotate(angular_momentum, inertia);

    if (lensq(omega) > min_velocity_threshold_sqr) {
      quat q_omega = { omega.x, omega.y, omega.z, 0 };
      quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
      quat q_orientation = qadd(rotation, dq);
      rotation = qnormalize(q_orientation);
    }

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->rotations[i] = rotation;

    if (lensq(velocity) > min_velocity_threshold_sqr) {
      dynamics->positions[i] = add(dynamics->positions[i], scale(velocity, dt));
    }
  }
}

void physics_step(physics_world* world, float dt) {
  integrate_bodies(world, dt);
  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);
  resolve_collisions(world, dt);
  update_awake_statuses(world, dt);
  clear_forces(world);

#ifdef DIAGNOSTICS
  world->diagnostics.frames_simulated += 1;
#endif

 }

void physics_awaken_body(physics_world* world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count)
    return;

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index)
    swap_bodies(world, index, target_index);

  dynamics->motion_avgs[target_index] = 2.0 * sleep_threshold;
  dynamics->awake_count += 1;
}

void physics_reset(physics_world *world) {
  world->dynamics.count = 0;
  world->dynamics.awake_count = 0;

  world->statics.count = 0;

  world->collisions->dynamic_collisions_count = 0;
  world->collisions->collisions_count = 0;
  world->collisions->contacts_count = 0;
}


void physics_teardown(physics_world* world) {
  #define TEARDOWN_COMMONS(type) \
    free(world->type.positions); \
    free(world->type.rotations); \
    free(world->type.shapes); \
    free(world->type.outer_lookup); \
    free(world->type.inner_lookup);

  TEARDOWN_COMMONS(dynamics);
  TEARDOWN_COMMONS(statics);

  #undef TEARDOWN_COMMONS

  free(world->dynamics.forces);
  free(world->dynamics.torques);
  free(world->dynamics.impulses);
  free(world->dynamics.angular_impulses);
  free(world->dynamics.accelerations);

  free(world->dynamics.inv_masses);
  free(world->dynamics.velocities);
  free(world->dynamics.angular_momenta);
  free(world->dynamics.inv_inertia_tensors);
  free(world->dynamics.inv_intertias);
  free(world->dynamics.motion_avgs);

  collisions_teardown(world->collisions);

#ifdef DIAGNOSTICS
  percentiles_free(world->diagnostics.penetration_depth);
  percentiles_free(world->diagnostics.velocity_deltas);
#endif

  free(world);
}

void clear_forces(physics_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;

  const count_t size = sizeof(v3) * dynamics->count;
  memset(dynamics->forces, 0, size);
  memset(dynamics->torques, 0, size);
  memset(dynamics->impulses, 0, size);
  memset(dynamics->angular_impulses, 0, size);
  memset(dynamics->accelerations, 0, size);
}

void update_awake_statuses(physics_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (dynamics->count == 0)
    return;

  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    v3 angular_velocity = matrix_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = lensq(dynamics->velocities[i]) + lensq(angular_velocity);
    float bias = powf(rwa_base_bias, dt);

    dynamics->motion_avgs[i] = current_motion * bias + new_motion * (1 - bias);
  }

  count_t left = 0;
  count_t right = dynamics->count - 1;
  while(left < awake_count && right >= awake_count) {
    while(dynamics->motion_avgs[left] > sleep_threshold) {
      left += 1;
    }

    while (dynamics->motion_avgs[right] <= sleep_threshold && right >= awake_count) {
      right -= 1;
    }

    if (left >= awake_count || right <= awake_count - 1)
      break;

    swap_bodies(world, left, right);
  }

  for (count_t i = awake_count - 1; i >= left && i != (count_t)-1; --i) {
    if (dynamics->motion_avgs[i] >= sleep_threshold)
      continue;

    count_t target_index = awake_count - 1;
    if (i != target_index)
      swap_bodies(world, i, target_index);

    dynamics->velocities[target_index] = dynamics->angular_momenta[target_index] = zero();
    awake_count -= 1;
  }

  for (count_t i = awake_count; i <= right; ++i) {
    if (dynamics->motion_avgs[i] < sleep_threshold)
      continue;

    count_t target_index = awake_count;
    if (i != target_index)
      swap_bodies(world, i, target_index);

    awake_count += 1;
  }

  dynamics->awake_count = awake_count;
}



// static void move_body(physics_world *world, count_t src_index, count_t dst_index) {
//   world->dynamics.positions[dst_index] = world->dynamics.positions[src_index];
//   world->dynamics.rotations[dst_index] = world->dynamics.rotations[src_index];
//   world->dynamics.shapes[dst_index] = world->dynamics.shapes[src_index];
//   world->dynamics.inv_masses[dst_index] = world->dynamics.inv_masses[src_index];
//   world->dynamics.velocities[dst_index] = world->dynamics.velocities[src_index];
//   world->dynamics.angular_momenta[dst_index] = world->dynamics.angular_momenta[src_index];
//   world->dynamics.inv_inertia_tensors[dst_index] = world->dynamics.inv_inertia_tensors[src_index];
//   world->dynamics.inv_intertias[dst_index] = world->dynamics.inv_intertias[src_index];
//   world->dynamics.motion_avgs[dst_index] = world->dynamics.motion_avgs[src_index];
// }

static void swap_bodies(physics_world *world, count_t index_a, count_t index_b) {
  #define SWAP(type, arr) \
    type tmp_##arr = world->dynamics.arr[index_a]; \
    world->dynamics.arr[index_a] = world->dynamics.arr[index_b]; \
    world->dynamics.arr[index_b] = tmp_##arr;

  SWAP(v3, positions)
  SWAP(quat, rotations)
  SWAP(body_shape, shapes)
  SWAP(float, inv_masses)
  SWAP(v3, velocities)
  SWAP(v3, angular_momenta)
  SWAP(m3, inv_inertia_tensors)
  SWAP(m3, inv_intertias)
  SWAP(float, motion_avgs)

  SWAP(v3, forces)
  SWAP(v3, torques)
  SWAP(v3, impulses)
  SWAP(v3, angular_impulses)
  SWAP(v3, accelerations)

  SWAP(count_t, inner_lookup)

  world->dynamics.outer_lookup[world->dynamics.inner_lookup[index_b]] = index_b;
  world->dynamics.outer_lookup[world->dynamics.inner_lookup[index_a]] = index_a;

  #undef SWAP
}
