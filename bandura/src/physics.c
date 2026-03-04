#include "physics.h"
#include "bandura.h"
#include <stdlib.h>
#include <string.h>

#define SHAPE_BRACKET_BLOCK_CAPACITY 64
<<<<<<< HEAD

const float min_velocity_threshold = 0.07f;
const float min_velocity_threshold_sqr = min_velocity_threshold * min_velocity_threshold;
||||||| d540963
const float min_velocity_threshold = 0.07f;
const float min_velocity_threshold_sqr = min_velocity_threshold * min_velocity_threshold;
=======
>>>>>>> 0a6db1174e69ac340c0cfe4378cc202486624de3

static v3 cylinder_inertia(float radius, float height, float mass) {
  float principal =  mass * (3 * radius * radius + height * height) / 12.0;
  return (v3){ principal, mass * radius * radius / 2.0, principal };
}

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

static bool find_empty_shape_slot(const physics_world *world, shape_dimension_bracket bracket, count_t *index) {

}

static void expand_shapes_bracket(physics_world *world, shape_dimension_bracket bracket) {

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

const common_data* as_common_const(const physics_world *world, body_type type) {
  return as_common((physics_world*)world, type);
}

static void swap_bodies(physics_world *world, count_t index_a, count_t index_b);

physics_config physics_default_config() {
  return (physics_config) {
    .gravity = (v3) { 0, -9.81f, 0 },
    .dynamics_capacity = 32,
    .statics_capacity = 8,
    .collisions_capacity = 64,
    .shapes_brackets_capacity = { 64, 1, 1, 1, 1 },
    .linear_damping = 0.95,
    .angular_damping = 0.8,
    .restitution = 0.2,
    .friction = 0.9,
    .max_penentration_iterations = 50,
    .max_velocity_iterations = 50,
    .sleep_base_bias = 0.5,
    .sleep_threshold = 0.3,
    .penetration_epsilon = 0.01,
    .velocity_epsilon = 0.01,
    .restitution_damping_limit = 0.25,
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

static void init_commons(common_data *data, count_t capacity) {
  data->capacity = capacity;
  data->count = 0;
  data->positions = malloc(sizeof(v3) * capacity);
  data->rotations = malloc(sizeof(quat) * capacity);
  data->shapes = malloc(sizeof(body_shapes) * capacity);
  data->outer_lookup = malloc(sizeof(count_t) * capacity);
  data->inner_lookup = malloc(sizeof(count_t) * capacity);
}

static void realloc_commons(common_data *data) {
  data->capacity = data->capacity << 1;
  data->positions = realloc(data->positions, sizeof(v3) * data->capacity);
  data->rotations = realloc(data->rotations, sizeof(quat) * data->capacity);
  data->shapes = realloc(data->shapes, sizeof(body_shapes) * data->capacity);
  data->outer_lookup = realloc(data->outer_lookup, sizeof(count_t) * data->capacity);
  data->inner_lookup = realloc(data->inner_lookup, sizeof(count_t) * data->capacity);
}

<<<<<<< HEAD
static void add_primitive_body_common(common_data *data, body_shape shape, count_t index) {
  data->positions[index] = zero();
  data->rotations[index] = qidentity();
  data->outer_lookup[index] = index;
  data->inner_lookup[index] = index;
||||||| d540963
=======
static void add_primitive_body_common(physics_world *world, common_data *data, body_shape shape, count_t index) {
  data->positions[index] = zero();
  data->rotations[index] = qidentity();
  data->outer_lookup[index] = index;
  data->inner_lookup[index] = index;

  count_t slot_index;
  if (!find_empty_shape_slot(world, BRACKET_PRIMITIVE, &slot_index)) {
    expand_shapes_bracket(world, BRACKET_PRIMITIVE);
  }
>>>>>>> 0a6db1174e69ac340c0cfe4378cc202486624de3


}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  init_commons((common_data*) &world->dynamics, config->dynamics_capacity);
  init_commons((common_data*) &world->statics, config->statics_capacity);

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

  world->generation = 0;

  return world;
}


static body add_primitive_body_static(physics_world* world, body_shape shape) {
  static_bodies *data = &world->statics;
  if (data->capacity < data->count + 1) {
    realloc_commons(data);
  }

  count_t index = data->count++;
<<<<<<< HEAD
  add_primitive_body_common(data, shape, index);
||||||| d540963
  init_commons(data, shape, index);
=======
  add_primitive_body_common(world, data, shape, index);
>>>>>>> 0a6db1174e69ac340c0cfe4378cc202486624de3

  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = NULL,
    .angular_momentum = NULL,
    .handle = make_body_handle(world, BODY_STATIC, index)
  };
}

static body add_primitive_body_dynamic(physics_world* world, body_shape shape, float mass) {
  dynamic_bodies *data = &world->dynamics;
  if (data->capacity < data->count + 1) {
    realloc_commons((common_data*) data);
    world->dynamics.forces = realloc(world->dynamics.forces, sizeof(v3) * data->capacity);
    world->dynamics.torques = realloc(world->dynamics.torques, sizeof(v3) * data->capacity);
    world->dynamics.impulses = realloc(world->dynamics.impulses, sizeof(v3) * data->capacity);
    world->dynamics.angular_impulses = realloc(world->dynamics.angular_impulses, sizeof(v3) * data->capacity);
    world->dynamics.accelerations = realloc(world->dynamics.accelerations, sizeof(v3) * data->capacity);

    world->dynamics.inv_masses = realloc(world->dynamics.inv_masses, sizeof(float) * data->capacity);
    world->dynamics.velocities = realloc(world->dynamics.velocities, sizeof(v3) * data->capacity);
    world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(v3) * data->capacity);
    world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(m3) * data->capacity);
    world->dynamics.inv_intertias = realloc(world->dynamics.inv_intertias, sizeof(m3) * data->capacity);
    world->dynamics.motion_avgs = realloc(world->dynamics.motion_avgs, sizeof(float) * data->capacity);
  }

  count_t index = data->count++;
<<<<<<< HEAD
  add_primitive_body_common((common_data*) data, shape, index);
||||||| d540963
  init_commons((common_data*) data, shape, index);
=======
  add_primitive_body_common(world, (common_data*) data, shape, index);
>>>>>>> 0a6db1174e69ac340c0cfe4378cc202486624de3

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

    case SHAPE_CYLINDER:
      inertia_vector = cylinder_inertia(shape.cylinder.radius, shape.cylinder.height, mass);
      break;

    default:
      break;
  }

  world->dynamics.inv_inertia_tensors[index] = matrix_initial_inertia(invert(inertia_vector));
  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = &world->dynamics.velocities[index],
    .angular_momentum = &world->dynamics.angular_momenta[index],
    .handle = make_body_handle(world, BODY_DYNAMIC, index),
  };
}

void physics_add_plane(physics_world *world, v3 point, v3 normal) {
  body plane = add_primitive_body_static(world, (body_shape) { .type = SHAPE_PLANE, .plane = { .normal = normal }, .offset = zero(), .rotation = qidentity() });
  world->statics.positions[handle_to_inner_index(world, plane.handle)] = point;
}

body physics_add_box_dynamic(physics_world *world, float mass, v3 size) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() }, mass);
}

body physics_add_box_static(physics_world *world, v3 size) {
  return add_primitive_body_static(world, (body_shape) { .type = SHAPE_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() });
}

body physics_add_sphere_dynamic(physics_world *world, float mass, float radius) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = radius }, .offset = zero(), .rotation = qidentity() }, mass);
}

body physics_add_cylinder_static(physics_world *world, float radius, float height) {
  return add_primitive_body_static(world, (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() });
}

body physics_add_cylinder_dynamic(physics_world *world, float mass, float radius, float height) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() }, mass);
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

physics_config* physics_edit_config(physics_world *world) {
  return &world->config;
}

count_t physics_body_count(const physics_world *world, body_type type) {
  return as_common_const(world, type)->count;
}

count_t physics_awake_count(const physics_world *world) {
  return world->dynamics.awake_count;
}

count_t physics_collisions_count(const physics_world *world) {
  return world->collisions->collisions_count;
}


v3 physics_get_position(const physics_world *world, body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->positions[handle_to_inner_index(world, handle)];
}

quat physics_get_rotation(const physics_world *world, body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->rotations[handle_to_inner_index(world, handle)];
}

body_shape physics_get_shape(const physics_world *world, body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->shapes[handle_to_inner_index(world, handle)];
}

v3 physics_get_velocity(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  return world->dynamics.velocities[handle_to_inner_index(world, handle)];
}

v3 physics_get_angular_velocity(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  v3 momentum = dynamics->angular_momenta[index];
  quat rotation = dynamics->rotations[index];
  m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return matrix_rotate(momentum, matrix_inertia(inv_inertia, rotation));
}



v3 physics_get_angular_momentum(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  return world->dynamics.angular_momenta[handle_to_inner_index(world, handle)];
}

float physics_get_motion_avg(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return 0;
  }

  return world->dynamics.motion_avgs[handle_to_inner_index(world, handle)];
}

const count_t sentinel_index = (count_t)~0 >> 1;

void physics_enumerate_bodies(const physics_world *world, body_enumerator *enumerator) {
  enumerator->handle = (body_handle) { .type = BODY_DYNAMIC, .index = sentinel_index & 0x7FFFFFFF };
  enumerator->generation = world->generation;
}

void physics_enumerate_bodies_typed(const physics_world *world, body_type type, body_enumerator *enumerator) {
  enumerator->handle = (body_handle) { .type = type, .index = sentinel_index & 0x7FFFFFFF };
  enumerator->generation = world->generation;
}

bool physics_body_next(const physics_world *world, body_enumerator *enumerator) {
  if (enumerator->generation != world->generation) {
    return false;
  }

  // Initial case. We assume that nobody messes with the enumerator after its initialization.
  if (enumerator->handle.index == sentinel_index) {
    if (world->dynamics.count > 0) {
      enumerator->handle.index = world->dynamics.outer_lookup[0];
      return true;
    }

    if (world->statics.count > 0) {
      enumerator->handle = (body_handle) { .type = BODY_STATIC, .index = 0 };
      return true;
    }

    return false;
  }

  count_t index;
  body_type type = enumerator->handle.type;

  switch (type) {
    case BODY_DYNAMIC:
      index = world->dynamics.inner_lookup[enumerator->handle.index];
      if (index < world->dynamics.count - 1) {
        index += 1;
        enumerator->handle.index = world->dynamics.outer_lookup[index];
        return true;
      }

      if (world->statics.count > 0) {
        enumerator->handle = (body_handle) { .type = BODY_STATIC, .index = 0 };
        return true;
      }

      return false;

    case BODY_STATIC:
      index = enumerator->handle.index;
      if (index < world->statics.count - 1) {
        index += 1;
        enumerator->handle.index = index;
        return true;
      }

      return false;
  }
}

bool physics_body_next_typed(const physics_world *world, body_enumerator_typed *enumerator) {
  if (enumerator->generation != world->generation) {
    return false;
  }

  const common_data *data = as_common_const(world, enumerator->handle.type);
  if (enumerator->handle.index == sentinel_index) {
    if (data->count == 0) {
      return false;
    }

    enumerator->handle.index = enumerator->handle.type == BODY_DYNAMIC ? world->dynamics.outer_lookup[0] : 0;
    return true;
  }

  count_t index;
  switch(enumerator->handle.type) {
    case BODY_DYNAMIC:
      index = world->dynamics.inner_lookup[enumerator->handle.index];
      if (index < data->count - 1) {
        index += 1;
        enumerator->handle.index = world->dynamics.outer_lookup[index];
        return true;
      }

      return false;

    case BODY_STATIC:
      index = enumerator->handle.index;
      if (index < data->count - 1) {
        index += 1;
        enumerator->handle.index = world->statics.outer_lookup[index];
        return true;
      }

      return false;
  }

  return false;
}

void integrate_bodies(physics_world *world, float dt) {
  v3 gravity_acc = world->config.gravity;
  float linear_damping = powf(world->config.linear_damping, dt);
  float angular_damping = powf(world->config.angular_damping, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    float inv_mass = dynamics->inv_masses[i];

    v3 acceleration = scale(dynamics->forces[i], inv_mass);
    acceleration = add(acceleration, gravity_acc);

    v3 impulse = scale(dynamics->impulses[i], inv_mass);

    v3 velocity = dynamics->velocities[i];
    velocity = add(velocity, scale(acceleration, dt));
    velocity = add(velocity, impulse);
    velocity = scale(velocity, linear_damping);

    quat rotation = dynamics->rotations[i];
    m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);

    v3 momentum_delta = scale(dynamics->torques[i], dt);
    momentum_delta = add(momentum_delta, dynamics->angular_impulses[i]);

    v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = add(angular_momentum, momentum_delta);
    angular_momentum = scale(angular_momentum, angular_damping);

    v3 omega = matrix_rotate(angular_momentum, inertia);

    quat q_omega = { omega.x, omega.y, omega.z, 0 };
    quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    quat q_orientation = qadd(rotation, dq);
    rotation = qnormalize(q_orientation);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = add(dynamics->positions[i], scale(velocity, dt));
  }
}

void physics_step(physics_world* world, float dt) {
  integrate_bodies(world, dt);
  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);
  resolve_collisions(world, dt);
  update_awake_statuses(world, dt);
  clear_forces(world);
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

  dynamics->motion_avgs[target_index] = 2.0 * world->config.sleep_threshold;
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

  const float sleep_threshold = world->config.sleep_threshold;
  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    v3 angular_velocity = matrix_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = lensq(dynamics->velocities[i]) + lensq(angular_velocity);
    float bias = powf(world->config.sleep_base_bias, dt);

    float motion = current_motion * bias + new_motion * (1 - bias);
    motion = fminf(motion, 10 * sleep_threshold);

    dynamics->motion_avgs[i] = motion;
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
  world->generation += 1;

  #undef SWAP
}
