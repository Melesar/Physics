#include "collisions.h"
#include "raylib.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include "stdlib.h"
#include "core.h"
#include "gizmos.h"
#include <math.h>
#include <stdlib.h>
#include "assert.h"

typedef struct {
  COMMON_FIELDS

  float *inv_masses;
  v3 *velocities;
  v3 *angular_momenta;
  m3 *inv_inertia_tensors;

  // Derived values.
  m3 *inv_intertias;
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
    .friction = 0.3,
    .max_resolution_iterations = 10,
    .restitution_damping_limit = 0.2,
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

static body physics_add_body(physics_world* world, body_type type, body_shape shape, float mass) {
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
  commons->shapes[index] = shape;
  commons->positions[index] = zero();
  commons->rotations[index] = qidentity();

  if (type == BODY_DYNAMIC) {
    world->dynamics.inv_masses[index] = 1.0 / mass;
    world->dynamics.velocities[index] = zero();
    world->dynamics.angular_momenta[index] = zero();

    m3 *inv_inertia_tensor = &world->dynamics.inv_inertia_tensors[index];
    switch (shape.type) {
      case SHAPE_BOX:
        *inv_inertia_tensor = matrix_initial_inertia(invert(box_inertia(shape.box.size, mass)));
        break;

      default:
        *inv_inertia_tensor = matrix_identity();
        break;
    }

    register_gizmo(&commons->positions[index], &commons->rotations[index]);
  }

  return (body) {
    .position = &commons->positions[index],
    .rotation = &commons->rotations[index],
    .velocity = type == BODY_DYNAMIC ? &world->dynamics.velocities[index] : NULL,
    .angular_momentum = type == BODY_DYNAMIC ? &world->dynamics.angular_momenta[index] : NULL,
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
    m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);
    v3 omega = matrix_rotate(dynamics->angular_momenta[i], inertia);

    m3 inv = matrix_inverse(inertia);
    m3 s = matrix_multiply(inv, inertia);

    quat q_omega = { omega.x, omega.y, omega.z, 0 };
    quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    quat q_orientation = qadd(rotation, dq);

    dynamics->rotations[i] = qnormalize(q_orientation);
    dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
  }

  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);
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
  float acceleration_velocity = dot(GRAVITY_V, contact->normal) * dt;

  float restitution = fabsf(contact->local_velocity.y) >= world->config.restitution_damping_limit ? world->config.restitution : 0.0f;
  contact->desired_delta_velocity = -contact->local_velocity.y - restitution * (contact->local_velocity.y - acceleration_velocity);
}

static void prepare_contacts(physics_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;

  for (count_t i = 0; i < world->collisions->collisions_count; ++i) {
    collision collision = world->collisions->collisions[i];
    count_t body_ids[] = { collision.index_a, collision.index_b };
    count_t body_count = i < world->collisions->dynamic_collisions_count ? 2 : 1;
    v3 angular_velocity[2];

    for (count_t k = 0; k < body_count; ++k) {
      m3 inv_inertia = matrix_inertia(dynamics->inv_inertia_tensors[body_ids[k]], dynamics->rotations[body_ids[k]]);
      angular_velocity[k] = matrix_rotate(dynamics->angular_momenta[body_ids[k]], inv_inertia);

      dynamics->inv_intertias[body_ids[k]] = inv_inertia;
    }

    for (count_t j = collision.contacts_offset; j < collision.contacts_offset + collision.contacts_count; ++j) {
      contact *contact = &world->collisions->contacts[j];

      contact->basis = contact_space_transform(contact);
      m3 world_to_contact = matrix_transpose(contact->basis);

      for (count_t k = 0; k < body_count; ++k) {
        contact->relative_position[k] = sub(contact->point, dynamics->positions[body_ids[k]]);
      }

      v3 acceleration_velocity = scale(GRAVITY_V, dt);
      acceleration_velocity = matrix_rotate(acceleration_velocity, world_to_contact);
      acceleration_velocity.y = 0;

      v3 local_velocity[2] = { 0 };
      for (count_t k = 0; k < body_count; ++k) {
        v3 vel = add(dynamics->velocities[body_ids[k]], cross(angular_velocity[k], contact->relative_position[k]));
        vel = matrix_rotate(vel, world_to_contact);
        local_velocity[k] = add(vel, acceleration_velocity);
      }

      contact->local_velocity = sub(local_velocity[0], local_velocity[1]);

      update_desired_velocity_delta(world, contact, dt);
    }
  }
}

static void resolve_interpenetration_contact(physics_world *world, count_t collision_index, const contact *contact, v3 *deltas) {
  count_t body_count = collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;
  count_t body_ids[] = { world->collisions->collisions[collision_index].index_a, world->collisions->collisions[collision_index].index_b };

  float total_inertia = 0;
  float linear_inertia[2];
  float angular_inertia_contact[2];
  v3 torque_per_impulse[2];
  v3 position[2];
  m3 inv_inertia_tensor[2];
  quat rotation[2];
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];

    position[k] = world->dynamics.positions[body_index];
    inv_inertia_tensor[k] = world->dynamics.inv_intertias[body_index];
    rotation[k] = world->dynamics.rotations[body_index];
    float inv_mass = world->dynamics.inv_masses[body_index];

    torque_per_impulse[k] = cross(contact->relative_position[k], contact->normal);

    v3 angular_inertia_world = torque_per_impulse[k];
    angular_inertia_world = matrix_rotate(angular_inertia_world, inv_inertia_tensor[k]);
    angular_inertia_world = cross(angular_inertia_world, contact->relative_position[k]);

    angular_inertia_contact[k] = dot(angular_inertia_world, contact->normal);
    linear_inertia[k] = inv_mass;
    total_inertia += linear_inertia[k] + angular_inertia_contact[k];
  }

  float inv_inertia = 1 / total_inertia;
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    float sign = k ? -1 : 1;
    float linear_move = sign * contact->depth * linear_inertia[k] * inv_inertia;
    float angular_move = sign * contact->depth * angular_inertia_contact[k] * inv_inertia;
    v3 linear_delta = scale(contact->normal, linear_move);

    world->dynamics.positions[body_index] = add(position[k], linear_delta);
    deltas[2 * k] = linear_delta;

    if (fabsf(angular_inertia_contact[k]) <= 0.0001) {
      deltas[2 * k + 1] = zero();
      continue;
    }

    v3 impulse_per_move = matrix_rotate(torque_per_impulse[k], inv_inertia_tensor[k]);
    v3 rotation_per_move = scale(impulse_per_move, 1.0 / angular_inertia_contact[k]);
    v3 rotation_delta = scale(rotation_per_move, angular_move);

    quat q_omega = { rotation_delta.x, rotation_delta.y, rotation_delta.z, 0 };
    quat dq = qmul(q_omega, rotation[k]);
    world->dynamics.rotations[body_index] = qnormalize(qadd(rotation[k], dq));

    deltas[2 * k + 1] = rotation_delta;
  }
}

static void update_penetration_depths(physics_world *world, count_t collision_index, count_t worst_contact_index, const v3 *deltas) {
  contact *worst_contact = &world->collisions->contacts[worst_contact_index];
  collision *worst_collision = &world->collisions->collisions[collision_index];

  count_t worst_body_ids[] = { worst_collision->index_a, worst_collision->index_b };
  count_t worst_body_count = collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;

  count_t count = world->collisions->collisions_count;
  for (count_t i = 0; i < count; ++i) {
    collision *collision = &world->collisions->collisions[i];
    count_t body_count = i < world->collisions->dynamic_collisions_count ? 2 : 1;
    count_t body_ids[] = { collision->index_a, collision->index_b };

    for (count_t j = 0; j < collision->contacts_count; ++j) {
      count_t index = collision->contacts_offset + j;
      contact *contact = &world->collisions->contacts[index];

      for (count_t k = 0; k < body_count; ++k) {
        count_t body_index = body_ids[k];

        for (count_t m = 0; m < worst_body_count; ++m) {
          count_t worst_body_index = worst_body_ids[m];

          if (body_index == worst_body_index) {
            v3 delta_position = add(deltas[2 * m], cross(deltas[2 * m + 1], contact->relative_position[k]));
            contact->depth += (k ? 1 : -1) * dot(delta_position, contact->normal);
          }
        }
      }
    }
  }
}

static void resolve_interpenetrations(physics_world *world) {
  const count_t count = collisions_count(world->collisions);

  if (count == 0)
    return;

  const float penetration_epsilon = 0.0001f; // TODO move to config
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

    v3 deltas[4];
    resolve_interpenetration_contact(world, collision_index, &contact, deltas);
    update_penetration_depths(world, collision_index, max_penetration_index, deltas);

    iterations += 1;
  }
}

static void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas, float dt) {
  count_t body_count = worst_collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;
  count_t body_ids[] = { world->collisions->collisions[worst_collision_index].index_a, world->collisions->collisions[worst_collision_index].index_b };

  m3 contact_to_world = contact->basis;
  m3 world_to_contact = matrix_transpose(contact_to_world);

  m3 delta_velocity = { 0 };
  float inv_mass = 0;
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    m3 impulse_to_torque = matrix_skew_symmetric(contact->relative_position[k]);

    m3 delta_velocity_world = matrix_multiply(world->dynamics.inv_intertias[body_index], impulse_to_torque);
    delta_velocity_world = matrix_multiply(delta_velocity_world, impulse_to_torque);
    delta_velocity_world = matrix_negate(delta_velocity_world);

    inv_mass += world->dynamics.inv_masses[body_index];
    delta_velocity = matrix_add(delta_velocity, delta_velocity_world);
  }

  delta_velocity = matrix_multiply(world_to_contact, delta_velocity);
  delta_velocity = matrix_multiply(delta_velocity, contact_to_world);
  delta_velocity.m0[0] += inv_mass;
  delta_velocity.m1[1] += inv_mass;
  delta_velocity.m2[2] += inv_mass;

  m3 impulse_matrix = matrix_inverse(delta_velocity);
  v3 velocity_to_kill = { -contact->local_velocity.x, contact->desired_delta_velocity, -contact->local_velocity.z };
  v3 contact_space_impulse = matrix_rotate(velocity_to_kill, impulse_matrix);
  float planar_impulse = sqrtf(contact_space_impulse.x * contact_space_impulse.x + contact_space_impulse.z * contact_space_impulse.z);

  if (planar_impulse > contact_space_impulse.y * world->config.friction) {
    contact_space_impulse.x /= planar_impulse;
    contact_space_impulse.z /= planar_impulse;

    float desired_delta_velocity = contact->desired_delta_velocity;
    contact_space_impulse.y =
      delta_velocity.m1[0] * world->config.friction * contact_space_impulse.x +
      delta_velocity.m1[1] +
      delta_velocity.m1[2] * world->config.friction * contact_space_impulse.z;
    contact_space_impulse.y = desired_delta_velocity / contact_space_impulse.y;
    contact_space_impulse.x *= world->config.friction * contact_space_impulse.y;
    contact_space_impulse.z *= world->config.friction * contact_space_impulse.y;
  }

  v3 world_space_impulse = matrix_rotate(contact_space_impulse, contact->basis);

  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    float inv_mass = world->dynamics.inv_masses[body_index];

    v3 linear_impulse_delta = scale(world_space_impulse, inv_mass);
    v3 angular_impulse_delta = cross(contact->relative_position[k], world_space_impulse);

    v3 *velocity = &world->dynamics.velocities[body_index];
    v3 *angular_momentum = &world->dynamics.angular_momenta[body_index];

    *velocity = add(*velocity, linear_impulse_delta);
    *angular_momentum = add(*angular_momentum, angular_impulse_delta);

    deltas[2 * k] = linear_impulse_delta;
    deltas[2 * k + 1] = angular_impulse_delta;

    world_space_impulse = scale(world_space_impulse, -1);
  }
}

static void update_velocity_deltas(physics_world *world, count_t worst_contact_index, count_t worst_collision_index, const v3 *deltas, float dt) {
  collision *worst_collision = &world->collisions->collisions[worst_collision_index];
  contact *worst_contact = &world->collisions->contacts[worst_contact_index];
  count_t worst_body_ids[] = { worst_collision->index_a, worst_collision->index_b };
  count_t worst_body_count = worst_collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;

  count_t count = world->collisions->collisions_count;
  for (count_t i = 0; i < count; ++i) {
    collision *collision = &world->collisions->collisions[i];
    count_t body_ids[] = { collision->index_a, collision->index_b };
    count_t body_count = i < world->collisions->dynamic_collisions_count ? 2 : 1;

    for (count_t j = 0; j < collision->contacts_count; ++j) {
      count_t index = collision->contacts_offset + j;
      contact *contact = &world->collisions->contacts[index];

      for (count_t k = 0; k < body_count; ++k) {
        count_t body_index = body_ids[k];

        for (count_t m = 0; m < worst_body_count; ++m) {
          count_t worst_body_index = worst_body_ids[m];

          if (body_index == worst_body_index) {
            v3 angular_velocity_delta = matrix_rotate(deltas[2 * m + 1], world->dynamics.inv_intertias[worst_body_index]);
            v3 delta_velocity = add(deltas[2 * m], cross(angular_velocity_delta, contact->relative_position[k]));
            delta_velocity = matrix_rotate_inverse(delta_velocity, contact->basis);

            contact->local_velocity = add(contact->local_velocity, scale(delta_velocity, (k ? -1 : 1)));

            update_desired_velocity_delta(world, contact, dt);
          }
        }
      }
    }
  }
}

static void resolve_velocities(physics_world *world, float dt) {
  const float velocity_epsilon = 0.00001f;
  const count_t count = world->collisions->collisions_count;
  if (count == 0)
    return;

  count_t iterations = 0;
  collision *collision;
  contact *contact;
  while (iterations < world->config.max_resolution_iterations) {
    float max_velocity = velocity_epsilon;
    count_t worst_contact_index = -1;
    count_t worst_collision_index = -1;
    for (count_t i = 0; i < count; ++i) {
      collision = &world->collisions->collisions[i];

      for (count_t j = 0; j < collision->contacts_count; ++j) {
        contact = &world->collisions->contacts[collision->contacts_offset + j];

        if (fabsf(contact->desired_delta_velocity) > max_velocity) {
          max_velocity = contact->desired_delta_velocity;
          worst_contact_index = collision->contacts_offset + j;
          worst_collision_index = i;
        }
      }
    }

    if (worst_contact_index == -1) {
      break;
    }

    contact = &world->collisions->contacts[worst_contact_index];

    v3 deltas[4];
    resolve_velocity_contact(world, worst_collision_index, contact, deltas, dt);
    update_velocity_deltas(world, worst_collision_index, worst_collision_index, deltas, dt);

    iterations += 1;
  }
}

static void resolve_collisions(physics_world *world, float dt) {
  prepare_contacts(world, dt);
  resolve_interpenetrations(world);
  resolve_velocities(world, dt);
}
