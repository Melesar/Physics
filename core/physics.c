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

typedef struct {
  COMMON_FIELDS

  float *inv_masses;
  Vector3 *velocities;
  Vector3 *angular_momenta;
  Matrix *inv_inertia_tensors;

  // Derived values.
  Matrix *inv_intertias;
  Vector3 *angular_velocities;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world {
  dynamic_bodies dynamics;
  static_bodies statics;

  collisions *collisions;

  float linear_damping, angular_damping;
  float restitution;

  count_t max_resolution_iterations;
};

static Matrix inertia_tensor_matrix(Vector3 inertia) {
  Matrix tensor = { 0 };
  tensor.m0 = inertia.x;
  tensor.m5 = inertia.y;
  tensor.m10 = inertia.z;
  return tensor;
}

static Vector3 cylinder_inertia(float radius, float height, float mass) {
  float principal =  mass * (3 * radius * radius + height * height) / 12.0;
  return (Vector3){ principal, mass * radius * radius / 2.0, principal };
}

static Vector3 sphere_inertia(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;
  return scale(one(), scale);
}

static Vector3 box_inertia(Vector3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  Vector3 i = { yy + zz, xx + zz, xx + yy };
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
static void calculate_derivatives(physics_world *world);

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
    world->type.positions = malloc(sizeof(Vector3) * cap); \
    world->type.rotations = malloc(sizeof(Quaternion) * cap); \
    world->type.shapes = malloc(sizeof(body_shape) * cap); \

  INIT_COMMONS(dynamics, config->dynamics_capacity);
  INIT_COMMONS(statics, config->statics_capacity);

  #undef INIT_COMMONS

  world->dynamics.inv_masses = malloc(sizeof(float) * config->dynamics_capacity);
  world->dynamics.velocities = malloc(sizeof(Vector3) * config->dynamics_capacity);
  world->dynamics.angular_momenta = malloc(sizeof(Vector3) * config->dynamics_capacity);
  world->dynamics.inv_inertia_tensors = malloc(sizeof(Matrix) * config->dynamics_capacity);

  world->dynamics.inv_intertias = malloc(sizeof(Matrix) * config->dynamics_capacity);
  world->dynamics.angular_velocities = malloc(sizeof(Vector3) * config->dynamics_capacity);

  world->linear_damping = config->linear_damping;
  world->angular_damping = config->angular_damping;
  world->restitution = config->restitution;

  world->max_resolution_iterations = config->max_resolution_iterations;

  world->collisions = collisions_init(config);

  return world;
}

void physics_add_body(physics_world* world, body_type type, body_shape shape, body_initial_state state) {
  common_data *commons = as_common(world, type);
  if (commons->capacity < commons->count + 1) {
    commons->capacity = commons->capacity << 1;
    commons->positions = realloc(commons->positions, sizeof(Vector3) * commons->capacity);
    commons->rotations = realloc(commons->rotations, sizeof(Quaternion) * commons->capacity);
    commons->shapes = realloc(commons->shapes, sizeof(body_shape) * commons->capacity);

    if (type == BODY_DYNAMIC) {
      world->dynamics.inv_masses = realloc(world->dynamics.inv_masses, sizeof(float) * commons->capacity);
      world->dynamics.velocities = realloc(world->dynamics.velocities, sizeof(Vector3) * commons->capacity);
      world->dynamics.angular_momenta = realloc(world->dynamics.angular_momenta, sizeof(Vector3) * commons->capacity);
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(Matrix) * commons->capacity);
      world->dynamics.inv_intertias = realloc(world->dynamics.inv_intertias, sizeof(Matrix) * commons->capacity);
      world->dynamics.angular_velocities = realloc(world->dynamics.angular_velocities, sizeof(Vector3) * commons->capacity);
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

    Matrix *inv_inertia_tensor = &world->dynamics.inv_inertia_tensors[index];
    switch (shape.type) {
      case SHAPE_BOX:
        *inv_inertia_tensor = inertia_tensor_matrix(Vector3Invert(box_inertia(shape.box.size, state.mass)));
        break;

      default:
        *inv_inertia_tensor = MatrixIdentity();
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
  Vector3 gravity_acc = scale(GRAVITY_V, dt);
  float linear_damping = powf(world->linear_damping, dt);
  float angular_damping = powf(world->angular_damping, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->count; ++i) {
    dynamics->velocities[i] = add(dynamics->velocities[i], gravity_acc);
    dynamics->velocities[i] = scale(dynamics->velocities[i], linear_damping);

    dynamics->angular_momenta[i] = scale(dynamics->angular_momenta[i], angular_damping);

    Quaternion rotation = dynamics->rotations[i];
    Matrix orientation = as_matrix(rotation);
    Matrix inertia = mul(mul(orientation, dynamics->inv_inertia_tensors[i]), transpose(orientation));
    Vector3 omega = transform(dynamics->angular_momenta[i], inertia);

    Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
    Quaternion dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    Quaternion q_orientation = qadd(rotation, dq);

    dynamics->rotations[i] = qnormalize(q_orientation);
    dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
  }

  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);

  calculate_derivatives(world);
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
  free(world->dynamics.angular_velocities);

  collisions_teardown(world->collisions);

  free(world);
}

static void calculate_derivatives(physics_world *world) {
  count_t count = world->dynamics.count;

  for (count_t i = 0; i < count; ++i) {
    Quaternion rotation = world->dynamics.rotations[i];
    Matrix rotation_matrix = as_matrix(rotation);
    Matrix inv_inertia = mul(mul(rotation_matrix, world->dynamics.inv_inertia_tensors[i]), transpose(rotation_matrix));
    Vector3 angular_velocity = transform(world->dynamics.angular_momenta[i], inv_inertia);

    world->dynamics.inv_intertias[i] = inv_inertia;
    world->dynamics.angular_velocities[i] = angular_velocity;
  }
}

static void resolve_interpenetration_contact(physics_world *world, count_t body_index, const contact *contact, Vector3 *deltas) {
  Vector3 position = world->dynamics.positions[body_index];
  Vector3 angular_velocity = world->dynamics.angular_velocities[body_index];
  Matrix inv_inertia_tensor = world->dynamics.inv_intertias[body_index];
  float inv_mass = world->dynamics.inv_masses[body_index];
  Quaternion rotation = world->dynamics.rotations[body_index];

  Matrix contact_to_world = contact_space_transform(contact);
  Matrix world_to_contact = transpose(contact_to_world);

  Vector3 point_relative_position = sub(contact->point, position);
  Vector3 point_rotational_velocity = cross(angular_velocity, point_relative_position);
  Vector3 torque_per_impulse = cross(point_relative_position, contact->normal);

  Vector3 angular_inertia_world = torque_per_impulse;
  angular_inertia_world = transform(angular_inertia_world, inv_inertia_tensor);
  angular_inertia_world = cross(angular_inertia_world, point_relative_position);

  float angular_inertia_contact = dot(angular_inertia_world, contact->normal);
  float linear_inertia = inv_mass;
  float total_inertia = linear_inertia + angular_inertia_contact;
  float inv_inertia = 1 / total_inertia;
  float linear_move = contact->depth * linear_inertia * inv_inertia;
  float angular_move = contact->depth * angular_inertia_contact * inv_inertia;
  Vector3 linear_delta = scale(contact->normal, linear_move);

  world->dynamics.positions[body_index] = add(position, linear_delta);
  deltas[0] = linear_delta;

  if (fabsf(angular_inertia_contact) <= 0.0001) {
    deltas[1] = zero();
    return;
  }

  Vector3 impulse_per_move = transform(torque_per_impulse, inv_inertia_tensor);
  Vector3 rotation_per_move = scale(impulse_per_move, 1.0 / angular_inertia_contact);
  Vector3 rotation_delta = scale(rotation_per_move, angular_move);

  Quaternion q_omega = { rotation_delta.x, rotation_delta.y, rotation_delta.z, 0 };
  Quaternion dq = qmul(q_omega, rotation);
  world->dynamics.rotations[body_index] = qnormalize(qadd(rotation, dq));

  deltas[1] = rotation_delta;
}

static void update_penetration_depths(physics_world *world, count_t worst_body_index, count_t worst_contact_index, const Vector3 *deltas) {
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
      contact_get(world->collisions, collision.contacts_offset + j, &contact);

      Vector3 delta_position = add(deltas[0], cross(deltas[1], sub(contact.point, world->dynamics.positions[worst_body_index])));
      float new_penetration = contact.depth - dot(delta_position, contact.normal);
      contact_update_penetration(world->collisions, collision.contacts_offset + j, new_penetration);
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
  while (iterations < world->max_resolution_iterations) {
    float max_penetration = -INFINITY;
    count_t max_penetration_index = -1;
    count_t collision_index = -1;

    for (count_t i = 0; i < count; ++i) {
      collision_get(world->collisions, i, &collision);

      for (count_t j = 0; j < collision.contacts_count; ++j) {
        contact_get(world->collisions, collision.contacts_offset + j, &contact);

        if (contact.depth > max_penetration) {
          max_penetration = contact.depth;
          max_penetration_index = j;
          collision_index = i;
        }
      }
    }

    if (max_penetration < penetration_epsilon)
      break;

    collision_get(world->collisions, collision_index, &collision);
    contact_get(world->collisions, collision.contacts_offset + max_penetration_index, &contact);

    count_t body_index = collision.index_a;
    Vector3 deltas[2];

    resolve_interpenetration_contact(world, body_index, &contact, deltas);
    update_penetration_depths(world, body_index, max_penetration_index, deltas);

    iterations += 1;
  }
}

static void resolve_velocities(physics_world *world) {
  const float velocity_epsilon = 0.00001f;
  const count_t count = collisions_count(world->collisions);
  if (count == 0)
    return;

  count_t iterations = 0;
  collision collision;
  contact contact;
  while (iterations < world->max_resolution_iterations) {
    float max_velocity = -INFINITY;
    count_t worst_contact_index = -1;
    count_t worst_collision_index = -1;
    for (count_t i = 0; i < count; ++i) {
      collision_get(world->collisions, i, &collision);

      for (count_t j = 0; j < count; ++j) {
        Vector3 angular_velocity =
        Vector3 closing_velocity = add(world->dynamics.velocities[collision.index_a], cross());
      }
    }
    iterations += 1;
  }
}

static void resolve_collisions(physics_world *world, float dt) {
  resolve_interpenetrations(world);
  resolve_velocities(world);
  // collisions *collisions = world->collisions;
  // count_t count = collisions_count(collisions);

  // collision c;
  // contact contact;
  // for (count_t i = 0; i < count; ++i) {
  //   collision_get(collisions, i, &c);

  //   count_t body_index = c.index_a;
  //   float inv_mass = world->dynamics.inv_masses[body_index];
  //   Vector3 position = world->dynamics.positions[body_index];
  //   Vector3 velocity = world->dynamics.velocities[body_index];
  //   Quaternion rotation = world->dynamics.rotations[body_index];
  //   Vector3 angular_momentum = world->dynamics.angular_momenta[body_index];
  //   Vector3 angular_velocity = world->dynamics.angular_velocities[body_index];
  //   Matrix inv_inertia_tensor = world->dynamics.inv_intertias[body_index];

  //   for (count_t j = 0; j < c.contacts_count; ++j) {
  //     contact_get(collisions, c.contacts_offset + j, &contact);

  //     Matrix contact_to_world = contact_space_transform(&contact);
  //     Matrix world_to_contact = transpose(contact_to_world);

  //     Vector3 point_relative_position = sub(contact.point, position);
  //     Vector3 point_rotational_velocity = cross(angular_velocity, point_relative_position);
  //     Vector3 torque_per_impulse = cross(point_relative_position, contact.normal);


  //     // Resolve velocity
  //     float velocity_change_per_unit_impulse = inv_mass; // Linear component (second body is static, mass = INF).
  //     velocity_change_per_unit_impulse += dot(point_rotational_velocity, contact.normal); // Velocity of the contact point along the contact normal.

  //     Vector3 closing_velocity = add(velocity, point_rotational_velocity);
  //     Vector3 contact_space_velocity = transform(closing_velocity, world_to_contact);

  //     float delta_velocity = -contact_space_velocity.y * (1 + world->restitution); // Y-component of the contact space velocity is the velocity along the contact normal.
  //     Vector3 contact_space_impulse = { 0, delta_velocity / velocity_change_per_unit_impulse, 0 };
  //     Vector3 world_space_impulse = transform(contact_space_impulse, contact_to_world);

  //     Vector3 linear_impulse_delta = scale(world_space_impulse, inv_mass);
  //     Vector3 angular_impulse_delta = cross(world_space_impulse, point_relative_position);

  //     velocity = add(velocity, linear_impulse_delta);
  //     angular_momentum = add(angular_momentum, angular_impulse_delta);
  //   }

  //   world->dynamics.positions[body_index] = position;
  //   world->dynamics.rotations[body_index] = rotation;
  //   world->dynamics.velocities[body_index] = velocity;
  //   world->dynamics.angular_momenta[body_index] = angular_momentum;
  // }
}
