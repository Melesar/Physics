#include "collisions.h"
#include "raylib.h"
#include "physics.h"
#include "raylib.h"
#include "stdlib.h"
#include "core.h"
#include "gizmos.h"
#include <math.h>
#include <stdlib.h>

typedef struct {
  COMMON_FIELDS

  float *inv_masses;
  v3 *velocities;
  v3 *angular_momenta;
  m4 *inv_inertia_tensors;

  // Derived values.
  m4 *inv_intertias;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world {
  dynamic_bodies dynamics;
  static_bodies statics;

  collisions *collisions;

  physics_config config;
};

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

static void resolve_collisions(physics_world *world, float dt);
static void prepare_contacts(physics_world *world, float dt);

physics_config physics_default_config() {
  return (physics_config) {
    .dynamics_capacity = 32,
    .statics_capacity = 8,
    .collisions_capacity = 64,
    .linear_damping = 0.997,
    .angular_damping = 0.997,
    .restitution = 0.3,
    .max_resolution_iterations = 10,
  };
}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  #define INIT_COMMONS(type, cap) \
    world->type.capacity = cap; \
    world->type.count = 0; \
    world->type.positions = malloc(sizeof(v3) * cap); \
    world->type.rotations = malloc(sizeof(quat) * cap); \
    world->type.shapes = malloc(sizeof(body_shape) * cap); \

  INIT_COMMONS(dynamics, config->dynamics_capacity);
  INIT_COMMONS(statics, config->statics_capacity);

  #undef INIT_COMMONS

  world->dynamics.inv_masses = malloc(sizeof(float) * config->dynamics_capacity);
  world->dynamics.velocities = malloc(sizeof(v3) * config->dynamics_capacity);
  world->dynamics.angular_momenta = malloc(sizeof(v3) * config->dynamics_capacity);
  world->dynamics.inv_inertia_tensors = malloc(sizeof(m4) * config->dynamics_capacity);

  world->dynamics.inv_intertias = malloc(sizeof(m4) * config->dynamics_capacity);

  world->config = *config;

  world->collisions = collisions_init(config);

  return world;
}

void physics_add_body(physics_world* world, body_type type, body_shape shape, body_initial_state state) {
  common_data *commons = as_common(world, type);
  if (commons->capacity < commons->count + 1) {
    commons->capacity = commons->capacity << 1;
    commons->positions = realloc(commons->positions, sizeof(v3) * commons->capacity);
    commons->rotations = realloc(commons->rotations, sizeof(quat) * commons->capacity);
    commons->shapes = realloc(commons->shapes, sizeof(body_shape) * commons->capacity);

    if (type == BODY_DYNAMIC) {
      world->dynamics.inv_masses = realloc(world->dynamics.inv_masses, sizeof(float) * commons->capacity);
      world->dynamics.velocities = realloc(world->dynamics.velocities, sizeof(v3) * commons->capacity);
      world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(v3) * commons->capacity);
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(m4) * commons->capacity);
      world->dynamics.inv_intertias = realloc(world->dynamics.inv_intertias, sizeof(m4) * commons->capacity);
    }
  }

  count_t index = commons->count++;
  commons->positions[index] = state.position;
  commons->rotations[index] = state.rotation;
  commons->shapes[index] = shape;

  if (type == BODY_DYNAMIC) {
    world->dynamics.inv_masses[index] = 1.0 / state.mass;
    world->dynamics.velocities[index] = zero();
    world->dynamics.angular_momenta[index] = state.angular_momentum;

    m4 *inv_inertia_tensor = &world->dynamics.inv_inertia_tensors[index];
    switch (shape.type) {
      case SHAPE_BOX:
        *inv_inertia_tensor = inertia_tensor_matrix(invert(box_inertia(shape.box.size, state.mass)));
        break;

      default:
        *inv_inertia_tensor = m4identity();
        break;
    }

    register_gizmo(&commons->positions[index], &commons->rotations[index]);
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
  v3 gravity_acc = scale(GRAVITY_V, dt);
  float linear_damping = powf(world->config.linear_damping, dt);
  float angular_damping = powf(world->config.angular_damping, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->count; ++i) {
    dynamics->velocities[i] = add(dynamics->velocities[i], gravity_acc);
    dynamics->velocities[i] = scale(dynamics->velocities[i], linear_damping);

    dynamics->angular_momenta[i] = scale(dynamics->angular_momenta[i], angular_damping);

    quat rotation = dynamics->rotations[i];
    m4 orientation = as_matrix(rotation);
    m4 inertia = mul(mul(orientation, dynamics->inv_inertia_tensors[i]), transpose(orientation));
    v3 omega = transform(dynamics->angular_momenta[i], inertia);

    quat q_omega = { omega.x, omega.y, omega.z, 0 };
    quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    quat q_orientation = qadd(rotation, dq);

    dynamics->rotations[i] = qnormalize(q_orientation);
    dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
  }

  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);

  // if (world->collisions->collisions_count > 0) {
  //   toggle_pause(true);
  // }

  resolve_collisions(world, dt);
}

void physics_draw_collisions(const physics_world *world) {
  count_t count = collisions_count(world->collisions);

  for (count_t i = 0; i < count; ++i) {
    collision c;
    collision_get(world->collisions, i, &c);

    for (count_t j = 0; j < c.contacts_count; ++j) {
      contact contact;
      contact_get(world->collisions, c.contacts_offset + j, &contact);


      draw_arrow(contact.point, contact.normal, RED);
    }
  }
}

bool physics_has_collisions(const physics_world *world) {
  return collisions_count(world->collisions);
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
  free(world->dynamics.inv_intertias);

  collisions_teardown(world->collisions);

  free(world);
}

static void update_desired_velocity_delta(physics_world *world, contact *contact, float dt) {
  const static float velocity_limit = 0.25f;
  float restitution = fabsf(contact->local_velocity.y) >= velocity_limit ? world->config.restitution : 0.0f;
  float acceleration_velocity = dot(GRAVITY_V, contact->normal) * dt;
  contact->desired_delta_velocity = -contact->local_velocity.y - restitution * (contact->local_velocity.y - acceleration_velocity);
}

static void prepare_contacts(physics_world *world, float dt) {
  for (count_t i = 0; i < world->collisions->collisions_count; ++i) {
    collision collision = world->collisions->collisions[i];
    count_t body_index = collision.index_a;

    m4 rotation_matrix = as_matrix(world->dynamics.rotations[body_index]);
    m4 inv_inertia = mul(mul(rotation_matrix, world->dynamics.inv_inertia_tensors[body_index]), transpose(rotation_matrix));
    v3 angular_velocity = transform(world->dynamics.angular_momenta[body_index], inv_inertia);

    world->dynamics.inv_intertias[body_index] = inv_inertia;

    for (count_t j = collision.contacts_offset; j < collision.contacts_offset + collision.contacts_count; ++j) {
      contact *contact = &world->collisions->contacts[j];

      contact->basis = contact_space_transform(contact);
      contact->relative_position = sub(contact->point, world->dynamics.positions[body_index]);
      contact->local_velocity = add(world->dynamics.velocities[body_index], cross(angular_velocity, contact->relative_position));
      contact->local_velocity = matrix_rotate_inverse(contact->local_velocity, contact->basis);

      update_desired_velocity_delta(world, contact, dt);
    }
  }
}

static void resolve_interpenetration_contact(physics_world *world, count_t body_index, const contact *contact, v3 *deltas) {
  v3 position = world->dynamics.positions[body_index];
  m4 inv_inertia_tensor = world->dynamics.inv_intertias[body_index];
  float inv_mass = world->dynamics.inv_masses[body_index];
  quat rotation = world->dynamics.rotations[body_index];

  v3 torque_per_impulse = cross(contact->relative_position, contact->normal);

  v3 angular_inertia_world = torque_per_impulse;
  angular_inertia_world = transform(angular_inertia_world, inv_inertia_tensor);
  angular_inertia_world = cross(angular_inertia_world, contact->relative_position);

  float angular_inertia_contact = dot(angular_inertia_world, contact->normal);
  float linear_inertia = inv_mass;
  float total_inertia = linear_inertia + angular_inertia_contact;
  float inv_inertia = 1 / total_inertia;
  float linear_move = contact->depth * linear_inertia * inv_inertia;
  float angular_move = contact->depth * angular_inertia_contact * inv_inertia;
  v3 linear_delta = scale(contact->normal, linear_move);

  world->dynamics.positions[body_index] = add(position, linear_delta);
  deltas[0] = linear_delta;

  if (fabsf(angular_inertia_contact) <= 0.0001) {
    deltas[1] = zero();
    return;
  }

  v3 impulse_per_move = transform(torque_per_impulse, inv_inertia_tensor);
  v3 rotation_per_move = scale(impulse_per_move, 1.0 / angular_inertia_contact);
  v3 rotation_delta = scale(rotation_per_move, angular_move);

  quat q_omega = { rotation_delta.x, rotation_delta.y, rotation_delta.z, 0 };
  quat dq = qmul(q_omega, rotation);
  world->dynamics.rotations[body_index] = qnormalize(qadd(rotation, dq));

  deltas[1] = rotation_delta;
}

static void update_penetration_depths(physics_world *world, count_t worst_body_index, count_t worst_contact_index, const v3 *deltas) {
  contact worst_contact, contact;
  collision collision;

  contact_get(world->collisions, worst_contact_index, &worst_contact);

  count_t count = collisions_count(world->collisions);
  for (count_t i = 0; i < count; ++i) {
    collision_get(world->collisions, i, &collision);

    if (collision.index_a != worst_body_index) {
      continue;
    }

    for (count_t j = 0; j < collision.contacts_count; ++j) {
      count_t index = collision.contacts_offset + j;
      contact_get(world->collisions, index, &contact);

      v3 delta_position = add(deltas[0], cross(deltas[1], sub(contact.point, world->dynamics.positions[worst_body_index])));
      float new_penetration = contact.depth - dot(delta_position, contact.normal);
      contact_update_penetration(world->collisions, index, new_penetration);
    }
  }
}

static void resolve_interpenetrations(physics_world *world) {
  const count_t count = collisions_count(world->collisions);

  if (count == 0)
    return;

  const float penetration_epsilon = 0.0001f;
  count_t iterations = 0;
  collision collision;
  contact contact;
  while (iterations < world->config.max_resolution_iterations) {
    float max_penetration = penetration_epsilon;
    count_t max_penetration_index = -1;
    count_t collision_index = -1;

    for (count_t i = 0; i < count; ++i) {
      collision_get(world->collisions, i, &collision);

      for (count_t j = 0; j < collision.contacts_count; ++j) {
        contact_get(world->collisions, collision.contacts_offset + j, &contact);

        if (contact.depth > max_penetration) {
          max_penetration = contact.depth;
          max_penetration_index = collision.contacts_offset + j;
          collision_index = i;
        }
      }
    }

    if (collision_index == -1)
      break;

    collision_get(world->collisions, collision_index, &collision);
    contact_get(world->collisions, max_penetration_index, &contact);

    count_t body_index = collision.index_a;
    v3 deltas[2];

    resolve_interpenetration_contact(world, body_index, &contact, deltas);
    update_penetration_depths(world, body_index, max_penetration_index, deltas);

    iterations += 1;
  }
}

static void resolve_velocity_contact(physics_world *world, count_t body_index, contact *contact, v3 *deltas, float dt) {
  v3 delta_velocity_world = cross(contact->relative_position, contact->normal);
  delta_velocity_world = transform(delta_velocity_world, world->dynamics.inv_intertias[body_index]);
  delta_velocity_world = cross(delta_velocity_world, contact->relative_position);

  float inv_mass = world->dynamics.inv_masses[body_index];
  float delta_velocity = dot(delta_velocity_world, contact->normal);
  delta_velocity += inv_mass;

  float desired_delta_velocity = contact->desired_delta_velocity; // Y-component of the contact space velocity is the velocity along the contact normal.
  v3 contact_space_impulse = { 0, desired_delta_velocity / delta_velocity, 0 };
  v3 world_space_impulse = matrix_rotate(contact_space_impulse, contact->basis);

  v3 linear_impulse_delta = scale(world_space_impulse, inv_mass);
  v3 angular_impulse_delta = cross(world_space_impulse, contact->relative_position);

  v3 *velocity = &world->dynamics.velocities[body_index];
  v3 *angular_momentum = &world->dynamics.angular_momenta[body_index];

  *velocity = add(*velocity, linear_impulse_delta);
  *angular_momentum = add(*angular_momentum, angular_impulse_delta);

  deltas[0] = linear_impulse_delta;
  deltas[1] = angular_impulse_delta;
}

static void update_velocity_deltas(physics_world *world, count_t worst_contact_index, count_t body_index, const v3 *deltas, float dt) {
  contact worst_contact;
  collision collision;

  contact_get(world->collisions, worst_contact_index, &worst_contact);

  count_t count = collisions_count(world->collisions);
  for (count_t i = 0; i < count; ++i) {
    collision_get(world->collisions, i, &collision);

    if (collision.index_a != body_index) {
      continue;
    }

    for (count_t j = 0; j < collision.contacts_count; ++j) {
      count_t index = collision.contacts_offset + j;
      contact *contact = &world->collisions->contacts[index];

      v3 delta_velocity = add(deltas[0], cross(deltas[1], contact->relative_position));
      contact->local_velocity = add(contact->local_velocity, matrix_rotate_inverse(delta_velocity, contact->basis));
      update_desired_velocity_delta(world, contact, dt);
    }
  }
}

static void resolve_velocities(physics_world *world, float dt) {
  const float velocity_epsilon = 0.00001f;
  const count_t count = collisions_count(world->collisions);
  if (count == 0)
    return;

  count_t iterations = 0;
  collision collision;
  contact contact;
  while (iterations < world->config.max_resolution_iterations) {
    float max_velocity = velocity_epsilon;
    count_t worst_contact_index = -1;
    count_t worst_collision_index = -1;
    for (count_t i = 0; i < count; ++i) {
      collision_get(world->collisions, i, &collision);

      for (count_t j = 0; j < collision.contacts_count; ++j) {
        contact_get(world->collisions, collision.contacts_offset + j, &contact);

        if (contact.desired_delta_velocity > max_velocity) {
          max_velocity = contact.desired_delta_velocity;
          worst_contact_index = collision.contacts_offset + j;
          worst_collision_index = i;
        }
      }
    }

    if (worst_contact_index == -1) {
      break;
    }

    collision_get(world->collisions, worst_collision_index, &collision);
    contact_get(world->collisions, worst_contact_index, &contact);

    v3 deltas[2];
    resolve_velocity_contact(world, collision.index_a, &contact, deltas, dt);
    update_velocity_deltas(world, worst_collision_index, collision.index_a, deltas, dt);

    iterations += 1;
  }
}

static void resolve_collisions(physics_world *world, float dt) {
  prepare_contacts(world, dt);
  resolve_interpenetrations(world);
  resolve_velocities(world, dt);
}
