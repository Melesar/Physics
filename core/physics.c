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

const float sleep_threshold = 0.1f;
const float rwa_base_bias = 0.7f;

typedef struct {
  COMMON_FIELDS

  float *inv_masses;
  v3 *velocities;
  v3 *angular_momenta;
  m3 *inv_inertia_tensors;

  // Derived values.
  m3 *inv_intertias;

  // Sleeping
  count_t awake_count;
  float *motion_avgs;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world {
  dynamic_bodies dynamics;
  static_bodies statics;

  collisions *collisions;

  physics_config config;
};

// static v3 cylinder_inertia(float radius, float height, float mass) {
//   float principal =  mass * (3 * radius * radius + height * height) / 12.0;
//   return (v3){ principal, mass * radius * radius / 2.0, principal };
// }

// static v3 sphere_inertia(float radius, float mass) {
//   float scale = 2.0 * mass * radius * radius / 5.0;
//   return scale(one(), scale);
// }

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

    default:
      return NULL;
  }
}

static const common_data* as_common_const(const physics_world *world, body_type type) {
  return as_common((physics_world*) world, type);
}

// static void move_body(physics_world *world, count_t src_index, count_t dst_index);
static void swap_bodies(physics_world *world, count_t index_a, count_t index_b);

static void resolve_collisions(physics_world *world, float dt);
static void prepare_contacts(physics_world *world, float dt);
static void update_awake_statuses(physics_world *world, float dt);
static void update_awake_status_for_collision(physics_world *world, count_t collision_index);

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
  world->dynamics.inv_inertia_tensors = malloc(sizeof(m3) * config->dynamics_capacity);
  world->dynamics.inv_intertias = malloc(sizeof(m3) * config->dynamics_capacity);
  world->dynamics.motion_avgs = malloc(sizeof(float) * config->dynamics_capacity);
  world->dynamics.awake_count = 0;

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
      world->dynamics.inv_inertia_tensors = realloc(world->dynamics.inv_inertia_tensors, sizeof(m3) * commons->capacity);
      world->dynamics.inv_intertias = realloc(world->dynamics.inv_intertias, sizeof(m3) * commons->capacity);
      world->dynamics.motion_avgs = realloc(world->dynamics.motion_avgs, sizeof(float) * commons->capacity);
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
    world->dynamics.motion_avgs[index] = 0;

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
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    dynamics->velocities[i] = add(dynamics->velocities[i], gravity_acc);
    dynamics->velocities[i] = scale(dynamics->velocities[i], linear_damping);
    dynamics->angular_momenta[i] = scale(dynamics->angular_momenta[i], angular_damping);

    quat rotation = dynamics->rotations[i];
    m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);
    v3 omega = matrix_rotate(dynamics->angular_momenta[i], inertia);

    quat q_omega = { omega.x, omega.y, omega.z, 0 };
    quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    quat q_orientation = qadd(rotation, dq);

    dynamics->rotations[i] = qnormalize(q_orientation);
    dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
  }

  collisions_detect(world->collisions, (common_data*) &world->dynamics, (common_data*)&world->statics);
  resolve_collisions(world, dt);
  update_awake_statuses(world, dt);
 }

void physics_awaken_body(physics_world* world, count_t index) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count)
    return;

  dynamics->motion_avgs[index] = 2.0 * sleep_threshold;
}

void physics_reset(physics_world *world) {
  world->dynamics.count = 0;
  world->dynamics.awake_count = 0;

  world->statics.count = 0;

  world->collisions->dynamic_collisions_count = 0;
  world->collisions->collisions_count = 0;
  world->collisions->contacts_count = 0;
}

void physics_draw_collisions(const physics_world *world) {
  count_t count = world->collisions->collisions_count;

  for (count_t i = 0; i < count; ++i) {
    collision c = world->collisions->collisions[i];

    for (count_t j = 0; j < c.contacts_count; ++j) {
      contact contact = world->collisions->contacts[c.contacts_offset + j];

      draw_arrow(contact.point, contact.normal, RED);
    }
  }
}

bool physics_has_collisions(const physics_world *world) {
  return collisions_count(world->collisions);
}

static bool begin_widget_window(
  struct nk_context* ctx,
  const char* window_name,
  const char* title,
  float x,
  float y,
  float width,
  float row_height,
  int row_count
) {
  const nk_flags window_flags = NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_MINIMIZABLE |
    NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_TITLE;

  float header_height = ctx->style.font->height + ctx->style.window.header.padding.y * 2.0f;
  float padding_y = ctx->style.window.padding.y;
  float spacing_y = ctx->style.window.spacing.y;
  float content_height = (row_height * row_count) + (spacing_y * (row_count - 1));
  float window_height = header_height + (padding_y * 2.0f) + content_height + 25.0;

  if (nk_begin_titled(ctx, window_name, title, nk_rect(x, y, width, window_height), window_flags)) {
    if (!nk_window_is_collapsed(ctx, window_name)) {
      nk_window_set_size(ctx, window_name, nk_vec2(width, window_height));
      return true;
    }
  }

  return false;
}

void physics_draw_stats(const physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_world_stats";
  const float row_height = 18.0f;
  const float window_width = 240.0f;
  const int row_count = 4;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics world stats", 20.0f, 200.0f, window_width, row_height, row_count);

  if (draw_content) {
    count_t dynamic_count = (count_t) physics_body_count(world, BODY_DYNAMIC);
    count_t static_count = (count_t) physics_body_count(world, BODY_STATIC);
    count_t collisions_total = (count_t) collisions_count(world->collisions);
    count_t awake_total = (count_t) world->dynamics.awake_count;

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_label(ctx, "Body count:", NK_TEXT_ALIGN_LEFT);

    nk_layout_row_begin(ctx, NK_DYNAMIC, row_height, 2);
    nk_layout_row_push(ctx, 0.5f);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Dynamic %u", dynamic_count);
    nk_layout_row_push(ctx, 0.5f);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Static %u", static_count);
    nk_layout_row_end(ctx);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Collisions count: %u", collisions_total);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Awake bodies: %u", awake_total);
  }

  nk_end(ctx);
}

void physics_draw_config_widget(physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_config_widget";
  const float row_height = 15.0f;
  const float window_width = 260.0f;
  const int row_count = 6;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics config", 20.0f, 360.0f, window_width, row_height, row_count);

  if (draw_content) {
    int max_iterations = (int) world->config.max_resolution_iterations;

    draw_edit_float(ctx, "Linear damping", &world->config.linear_damping);
    draw_edit_float(ctx, "Angular damping", &world->config.angular_damping);
    draw_edit_float(ctx, "Restitution", &world->config.restitution);
    draw_edit_float(ctx, "Friction", &world->config.friction);
    draw_edit_int(ctx, "Resolution iterations", &max_iterations);
    draw_edit_float(ctx, "Restitution damp limit", &world->config.restitution_damping_limit);

    if (max_iterations < 0)
      max_iterations = 0;

    world->config.max_resolution_iterations = (count_t) max_iterations;
  }

  nk_end(ctx);
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
  free(world->dynamics.motion_avgs);

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

static void update_penetration_depths_ex(physics_world *world, count_t collision_index, const v3 *deltas, depth_update_record *records, count_t *record_count) {
  collision *worst_collision = &world->collisions->collisions[collision_index];

  count_t worst_body_ids[] = { worst_collision->index_a, worst_collision->index_b };
  count_t worst_body_count = collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;

  if (record_count) *record_count = 0;

  count_t count = world->collisions->collisions_count;
  for (count_t i = 0; i < count; ++i) {
    collision *collision = &world->collisions->collisions[i];
    count_t body_count = i < world->collisions->dynamic_collisions_count ? 2 : 1;
    count_t body_ids[] = { collision->index_a, collision->index_b };

    for (count_t j = 0; j < collision->contacts_count; ++j) {
      count_t index = collision->contacts_offset + j;
      contact *contact = &world->collisions->contacts[index];
      float depth_before = contact->depth;

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

      if (records && record_count && contact->depth != depth_before && *record_count < CDBG_MAX_CONTACTS) {
        records[*record_count] = (depth_update_record){ .index = index, .before = depth_before, .after = contact->depth };
        (*record_count)++;
      }
    }
  }
}

static void update_penetration_depths(physics_world *world, count_t collision_index, const v3 *deltas) {
  update_penetration_depths_ex(world, collision_index, deltas, NULL, NULL);
}

static void resolve_interpenetrations(physics_world *world) {
  const count_t count = collisions_count(world->collisions);

  if (count == 0)
    return;

  const float penetration_epsilon = 0.0001f; // TODO move to config
  count_t iterations = 0;
  collision *collision;
  contact *contact;
  while (iterations < world->config.max_resolution_iterations) {
    float max_penetration = penetration_epsilon;
    count_t max_penetration_index = -1;
    count_t collision_index = -1;

    for (count_t i = 0; i < count; ++i) {
      collision = &world->collisions->collisions[i];

      for (count_t j = 0; j < collision->contacts_count; ++j) {
        contact = &world->collisions->contacts[j];

        if (contact->depth > max_penetration) {
          max_penetration = contact->depth;
          max_penetration_index = collision->contacts_offset + j;
          collision_index = i;
        }
      }
    }

    if (collision_index == (count_t)-1)
      break;

    contact = &world->collisions->contacts[max_penetration_index];

    update_awake_status_for_collision(world, collision_index);

    v3 deltas[4];
    resolve_interpenetration_contact(world, collision_index, contact, deltas);
    update_penetration_depths(world, collision_index, deltas);

    iterations += 1;
  }
}

static void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas) {
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

static void update_velocity_deltas_ex(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt, velocity_update_record *records, count_t *record_count) {
  collision *worst_collision = &world->collisions->collisions[worst_collision_index];
  count_t worst_body_ids[] = { worst_collision->index_a, worst_collision->index_b };
  count_t worst_body_count = worst_collision_index < world->collisions->dynamic_collisions_count ? 2 : 1;

  if (record_count) *record_count = 0;

  count_t count = world->collisions->collisions_count;
  for (count_t i = 0; i < count; ++i) {
    collision *collision = &world->collisions->collisions[i];
    count_t body_ids[] = { collision->index_a, collision->index_b };
    count_t body_count = i < world->collisions->dynamic_collisions_count ? 2 : 1;

    for (count_t j = 0; j < collision->contacts_count; ++j) {
      count_t index = collision->contacts_offset + j;
      contact *contact = &world->collisions->contacts[index];
      v3 local_vel_before = contact->local_velocity;
      float ddv_before = contact->desired_delta_velocity;
      bool changed = false;

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
            changed = true;
          }
        }
      }

      if (records && record_count && changed && *record_count < CDBG_MAX_CONTACTS) {
        records[*record_count] = (velocity_update_record){
          .index = index,
          .local_vel_before = local_vel_before,
          .local_vel_after = contact->local_velocity,
          .ddv_before = ddv_before,
          .ddv_after = contact->desired_delta_velocity,
        };
        (*record_count)++;
      }
    }
  }
}

static void update_velocity_deltas(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt) {
  update_velocity_deltas_ex(world, worst_collision_index, deltas, dt, NULL, NULL);
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

    if (worst_contact_index == (count_t)-1) {
      break;
    }

    contact = &world->collisions->contacts[worst_contact_index];

    update_awake_status_for_collision(world, worst_collision_index);

    v3 deltas[4];
    resolve_velocity_contact(world, worst_collision_index, contact, deltas);
    update_velocity_deltas(world, worst_collision_index, deltas, dt);

    iterations += 1;
  }
}

static void resolve_collisions(physics_world *world, float dt) {
  prepare_contacts(world, dt);
  resolve_interpenetrations(world);

  if (world->collisions->dynamic_collisions_count > 0) {
    float largest_penetration = 0;
    for (count_t i = 0; i < world->collisions->contacts_count; ++i) {
      float penetration = fabsf(world->collisions->contacts[i].depth);

      if (penetration > largest_penetration)
        largest_penetration = penetration;
    }

    TraceLog(LOG_DEBUG, "Largest penetration: %f", largest_penetration);
  }

  resolve_velocities(world, dt);
}

static void update_awake_statuses(physics_world *world, float dt) {
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

static void update_awake_status_for_collision(physics_world *world, count_t collision_index) {
  if (collision_index >= world->collisions->dynamic_collisions_count)
    return;

  collision *collision = &world->collisions->collisions[collision_index];

  bool body_a_awake = collision->index_a < world->dynamics.awake_count;
  bool body_b_awake = collision->index_b < world->dynamics.awake_count;
  if (body_a_awake == body_b_awake)
    return;

  if (!body_a_awake)
    world->dynamics.motion_avgs[collision->index_a] = 2.0 * sleep_threshold;

  if (!body_b_awake)
    world->dynamics.motion_avgs[collision->index_b] = 2.0 * sleep_threshold;
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

  #undef SWAP
}

// ====== COLLISION DEBUGGING =======

void physics_debug_state_init(collision_debug_state *state) {
  state->active = false;
  state->phase = CDBG_IDLE;
  state->iteration = 0;
  state->is_dynamic = false;
  state->current_contact_index = (count_t)-1;
  state->dt = 0;
  state->penetration_done = false;
  state->velocity_done = false;
  state->needs_integration = true;
  state->depth_update_count = 0;
  state->velocity_update_count = 0;
  for (int i = 0; i < 4; ++i) state->deltas[i] = zero();
}

// Find the worst penetration contact. Returns false if none above threshold.
static bool find_worst_penetration(physics_world *world, count_t *out_collision_index, count_t *out_contact_index) {
  const float penetration_epsilon = 0.0001f;
  float max_penetration = penetration_epsilon;
  count_t best_contact = (count_t)-1;
  count_t best_collision = (count_t)-1;
  count_t count = collisions_count(world->collisions);

  for (count_t i = 0; i < count; ++i) {
    collision *c = &world->collisions->collisions[i];
    for (count_t j = 0; j < c->contacts_count; ++j) {
      contact *ct = &world->collisions->contacts[c->contacts_offset + j];
      if (ct->depth > max_penetration) {
        max_penetration = ct->depth;
        best_contact = c->contacts_offset + j;
        best_collision = i;
      }
    }
  }

  if (best_collision == (count_t)-1)
    return false;

  *out_collision_index = best_collision;
  *out_contact_index = best_contact;
  return true;
}

// Find the worst velocity contact. Returns false if none above threshold.
static bool find_worst_velocity(physics_world *world, count_t *out_collision_index, count_t *out_contact_index) {
  const float velocity_epsilon = 0.00001f;
  float max_velocity = velocity_epsilon;
  count_t best_contact = (count_t)-1;
  count_t best_collision = (count_t)-1;
  count_t count = world->collisions->collisions_count;

  for (count_t i = 0; i < count; ++i) {
    collision *c = &world->collisions->collisions[i];
    for (count_t j = 0; j < c->contacts_count; ++j) {
      contact *ct = &world->collisions->contacts[c->contacts_offset + j];
      if (fabsf(ct->desired_delta_velocity) > max_velocity) {
        max_velocity = ct->desired_delta_velocity;
        best_contact = c->contacts_offset + j;
        best_collision = i;
      }
    }
  }

  if (best_collision == (count_t)-1)
    return false;

  *out_collision_index = best_collision;
  *out_contact_index = best_contact;
  return true;
}

void physics_step_debug(physics_world *world, float dt, collision_debug_state *state) {
  // Phase: integration + collision detection (runs once per frame)
  if (state->needs_integration) {
    v3 gravity_acc = scale(GRAVITY_V, dt);
    float linear_damping = powf(world->config.linear_damping, dt);
    float angular_damping = powf(world->config.angular_damping, dt);

    dynamic_bodies *dynamics = &world->dynamics;
    for (count_t i = 0; i < dynamics->awake_count; ++i) {
      dynamics->velocities[i] = add(dynamics->velocities[i], gravity_acc);
      dynamics->velocities[i] = scale(dynamics->velocities[i], linear_damping);
      dynamics->angular_momenta[i] = scale(dynamics->angular_momenta[i], angular_damping);

      quat rotation = dynamics->rotations[i];
      m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);
      v3 omega = matrix_rotate(dynamics->angular_momenta[i], inertia);

      quat q_omega = { omega.x, omega.y, omega.z, 0 };
      quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
      quat q_orientation = qadd(rotation, dq);

      dynamics->rotations[i] = qnormalize(q_orientation);
      dynamics->positions[i] = add(dynamics->positions[i], scale(dynamics->velocities[i], dt));
    }

    collisions_detect(world->collisions, (common_data*)&world->dynamics, (common_data*)&world->statics);

    // No dynamic collisions — run normal resolution and finish
    if (world->collisions->dynamic_collisions_count == 0) {
      resolve_collisions(world, dt);
      update_awake_statuses(world, dt);
      state->active = false;
      state->phase = CDBG_IDLE;
      return;
    }

    // Dynamic collision detected — enter debug mode
    prepare_contacts(world, dt);
    state->active = true;
    state->dt = dt;
    state->iteration = 0;
    state->penetration_done = false;
    state->velocity_done = false;
    state->needs_integration = false;

    // Try to find first penetration contact
    count_t ci, cti;
    if (find_worst_penetration(world, &ci, &cti)) {
      state->iteration = 1;
      state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
      state->current_contact_index = cti;

      update_awake_status_for_collision(world, ci);

      contact *ct = &world->collisions->contacts[cti];
      resolve_interpenetration_contact(world, ci, ct, state->deltas);
      state->phase = CDBG_PENETRATION_RESOLVE;
    } else {
      state->penetration_done = true;
      // Skip to velocity
      if (find_worst_velocity(world, &ci, &cti)) {
        state->iteration = 1;
        state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
        state->current_contact_index = cti;

        update_awake_status_for_collision(world, ci);

        contact *ct = &world->collisions->contacts[cti];
        resolve_velocity_contact(world, ci, ct, state->deltas);
        state->phase = CDBG_VELOCITY_RESOLVE;
      } else {
        state->phase = CDBG_DONE;
      }
    }

    toggle_pause(true);
    return;
  }

  // State machine: advance one sub-step per call
  count_t ci, cti;

  switch (state->phase) {
    case CDBG_PENETRATION_RESOLVE: {
      // The resolve just happened, now do the depth update
      // We need to find which collision was just resolved — reconstruct from the last search
      // Re-search to find the collision that was resolved (it was the worst before resolve)
      // Actually, the deltas are already applied. We need the collision index.
      // Let's store it. But we don't have it in state... Let me re-find the worst
      // post-resolution to do the update. Actually, the update needs the collision index
      // of what was JUST resolved, which we can find by re-running the search on the
      // pre-update state. Since resolve already moved bodies, we need to track the index.
      //
      // Workaround: we need to store the collision index. Let me add an approach where
      // we do the depth update immediately after resolve in the same step, then show
      // both results. Actually the spec says they're separate steps. Let me rethink.
      //
      // The depth update function needs the collision_index and deltas from the resolve.
      // We have deltas in state. We need collision_index. Let me search again for the
      // contact that was just resolved — but depths have not been updated yet, and the
      // body positions have moved. The depth values in contacts are stale.
      //
      // Actually looking more carefully: resolve_interpenetration_contact only modifies
      // positions/rotations. It does NOT update contact->depth. So the worst penetration
      // contact is still the same one. We can re-find it.

       // Re-find worst penetration (depths not yet updated, so same contact is still worst)
       if (find_worst_penetration(world, &ci, &cti)) {
         update_penetration_depths_ex(world, ci, state->deltas, state->depth_updates, &state->depth_update_count);
         state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
         state->current_contact_index = cti;
         state->phase = CDBG_DEPTH_UPDATE;
       } else {
         // Shouldn't happen since we just resolved something, but handle gracefully
         state->penetration_done = true;
         state->phase = CDBG_DEPTH_UPDATE;
         state->depth_update_count = 0;
       }
      break;
    }

     case CDBG_DEPTH_UPDATE: {
       // Depth update shown. Try next penetration iteration.
       if (state->iteration < world->config.max_resolution_iterations && find_worst_penetration(world, &ci, &cti)) {
         state->iteration++;
         state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
         state->current_contact_index = cti;

         update_awake_status_for_collision(world, ci);

         contact *ct = &world->collisions->contacts[cti];
         resolve_interpenetration_contact(world, ci, ct, state->deltas);
         state->phase = CDBG_PENETRATION_RESOLVE;
       } else {
         // Penetration done, switch to velocity resolution
         state->penetration_done = true;
         state->iteration = 0;

         if (find_worst_velocity(world, &ci, &cti)) {
           state->iteration = 1;
           state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
           state->current_contact_index = cti;

           update_awake_status_for_collision(world, ci);

           contact *ct = &world->collisions->contacts[cti];
           resolve_velocity_contact(world, ci, ct, state->deltas);
           state->phase = CDBG_VELOCITY_RESOLVE;
         } else {
           state->phase = CDBG_DONE;
         }
       }
       break;
     }

     case CDBG_VELOCITY_RESOLVE: {
       // Velocity resolve just happened. Now do the velocity update.
       // Similar to penetration: resolve_velocity_contact modifies velocities/angular_momenta
       // but does NOT update desired_delta_velocity on other contacts. So worst is still findable.
       if (find_worst_velocity(world, &ci, &cti)) {
         update_velocity_deltas_ex(world, ci, state->deltas, state->dt, state->velocity_updates, &state->velocity_update_count);
         state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
         state->current_contact_index = cti;
         state->phase = CDBG_VELOCITY_UPDATE;
       } else {
         state->velocity_done = true;
         state->phase = CDBG_VELOCITY_UPDATE;
         state->velocity_update_count = 0;
       }
       break;
     }

     case CDBG_VELOCITY_UPDATE: {
       // Velocity update shown. Try next velocity iteration.
       if (state->iteration < world->config.max_resolution_iterations && find_worst_velocity(world, &ci, &cti)) {
         state->iteration++;
         state->is_dynamic = ci < world->collisions->dynamic_collisions_count;
         state->current_contact_index = cti;

         update_awake_status_for_collision(world, ci);

         contact *ct = &world->collisions->contacts[cti];
         resolve_velocity_contact(world, ci, ct, state->deltas);
         state->phase = CDBG_VELOCITY_RESOLVE;
       } else {
         state->phase = CDBG_DONE;
       }
       break;
     }

    case CDBG_DONE: {
      // All done — finish the frame
      update_awake_statuses(world, state->dt);
      state->active = false;
      state->phase = CDBG_IDLE;
      state->needs_integration = true;
      break;
    }

    case CDBG_IDLE:
      break;
  }
}

static const char* debug_phase_label(collision_debug_phase phase) {
  switch (phase) {
    case CDBG_PENETRATION_RESOLVE: return "Penetration resolve";
    case CDBG_DEPTH_UPDATE:        return "Depth update";
    case CDBG_VELOCITY_RESOLVE:    return "Velocity resolve";
    case CDBG_VELOCITY_UPDATE:     return "Velocity update";
    case CDBG_DONE:                return "Done";
    case CDBG_IDLE:                return "Idle";
  }
  return "Unknown";
}

void physics_draw_debug_widget(const physics_world *world, const collision_debug_state *state, struct nk_context *ctx) {
  if (!state->active)
    return;

  static const char *window_name = "collision_debug_widget";
  const float row_height = 18.0f;
  const float window_width = 420.0f;

  // Estimate row count based on phase
  int row_count = 8; // header rows: iteration, phase, collision type + contact info (index, depth, local vel, desired delta)
  switch (state->phase) {
    case CDBG_PENETRATION_RESOLVE:
    case CDBG_VELOCITY_RESOLVE:
      // "Deltas:" label + body headers + linear/angular rows
      row_count += 1 + 1 + 2;
      break;
    case CDBG_DEPTH_UPDATE:
      row_count += 1 + (int)state->depth_update_count;
      break;
    case CDBG_VELOCITY_UPDATE:
      row_count += 1 + (int)state->velocity_update_count * 3;
      break;
    default:
      break;
  }

  bool draw_content = begin_widget_window(ctx, window_name, "Collision debug", 20.0f, 500.0f, window_width, row_height, row_count);

  if (draw_content) {
    // Header
    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Iteration: %u", state->iteration);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Phase: %s", debug_phase_label(state->phase));

     nk_layout_row_dynamic(ctx, row_height, 1);
     nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Collision type: %s", state->is_dynamic ? "dynamic" : "static");

     // Display contact information
     if (state->current_contact_index != (count_t)-1 && state->current_contact_index < world->collisions->contacts_count) {
       const contact *current_contact = &world->collisions->contacts[state->current_contact_index];

       nk_layout_row_dynamic(ctx, row_height, 1);
       nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Contact index: %u", state->current_contact_index);

       nk_layout_row_dynamic(ctx, row_height, 1);
       nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Depth: %.3f", current_contact->depth);

       nk_layout_row_dynamic(ctx, row_height, 1);
       nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Local velocity: (%.2f, %.2f, %.2f)",
         current_contact->local_velocity.x, current_contact->local_velocity.y, current_contact->local_velocity.z);

       nk_layout_row_dynamic(ctx, row_height, 1);
       nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Desired delta velocity: %.3f", current_contact->desired_delta_velocity);
     }

     // Body
     switch (state->phase) {
      case CDBG_PENETRATION_RESOLVE:
      case CDBG_VELOCITY_RESOLVE: {
        nk_layout_row_dynamic(ctx, row_height, 1);
        nk_label(ctx, "Deltas:", NK_TEXT_ALIGN_LEFT);

        if (state->is_dynamic) {
          nk_layout_row_begin(ctx, NK_DYNAMIC, row_height, 2);
          nk_layout_row_push(ctx, 0.5f);
          nk_label(ctx, "Body 1", NK_TEXT_ALIGN_LEFT);
          nk_layout_row_push(ctx, 0.5f);
          nk_label(ctx, "Body 2", NK_TEXT_ALIGN_LEFT);
          nk_layout_row_end(ctx);

          nk_layout_row_begin(ctx, NK_DYNAMIC, row_height, 2);
          nk_layout_row_push(ctx, 0.5f);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Lin: (%.3f, %.3f, %.3f)", state->deltas[0].x, state->deltas[0].y, state->deltas[0].z);
          nk_layout_row_push(ctx, 0.5f);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Lin: (%.3f, %.3f, %.3f)", state->deltas[2].x, state->deltas[2].y, state->deltas[2].z);
          nk_layout_row_end(ctx);

          nk_layout_row_begin(ctx, NK_DYNAMIC, row_height, 2);
          nk_layout_row_push(ctx, 0.5f);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Ang: (%.3f, %.3f, %.3f)", state->deltas[1].x, state->deltas[1].y, state->deltas[1].z);
          nk_layout_row_push(ctx, 0.5f);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Ang: (%.3f, %.3f, %.3f)", state->deltas[3].x, state->deltas[3].y, state->deltas[3].z);
          nk_layout_row_end(ctx);
        } else {
          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_label(ctx, "Body 1", NK_TEXT_ALIGN_LEFT);

          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Linear:  (%.3f, %.3f, %.3f)", state->deltas[0].x, state->deltas[0].y, state->deltas[0].z);

          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Angular: (%.3f, %.3f, %.3f)", state->deltas[1].x, state->deltas[1].y, state->deltas[1].z);
        }
        break;
      }

      case CDBG_DEPTH_UPDATE: {
        nk_layout_row_dynamic(ctx, row_height, 1);
        nk_label(ctx, "Depth updates:", NK_TEXT_ALIGN_LEFT);

        for (count_t i = 0; i < state->depth_update_count; ++i) {
          const depth_update_record *r = &state->depth_updates[i];
          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Contact #%u: %.3f -> %.3f", r->index, r->before, r->after);
        }
        break;
      }

      case CDBG_VELOCITY_UPDATE: {
        nk_layout_row_dynamic(ctx, row_height, 1);
        nk_label(ctx, "Velocity updates:", NK_TEXT_ALIGN_LEFT);

        for (count_t i = 0; i < state->velocity_update_count; ++i) {
          const velocity_update_record *r = &state->velocity_updates[i];
          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Contact #%u:", r->index);

          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "  Local vel: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)",
            r->local_vel_before.x, r->local_vel_before.y, r->local_vel_before.z,
            r->local_vel_after.x, r->local_vel_after.y, r->local_vel_after.z);

          nk_layout_row_dynamic(ctx, row_height, 1);
          nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "  Desired delta: %.3f -> %.3f", r->ddv_before, r->ddv_after);
        }
        break;
      }

      default:
        break;
    }
  }

  nk_end(ctx);
}
