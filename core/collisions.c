#include "collisions.h"
#include "physics.h"
#include "raymath.h"
#include <math.h>
#include <stdlib.h>

#define ARRAY_INIT(base, type, capacity) \
  base->type##s_capacity = capacity; \
  base->type##s_count = 0; \
  base->type##s = malloc(base->type##s_capacity * sizeof(type));

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type) \
  while (count >= capacity) { \
    capacity <<= 1; \
    if (count <= capacity) { \
      array = realloc(array, capacity * sizeof(type)); \
      break; \
    } \
  }

static count_t box_box_collision(collisions* collisions, count_t index_a, count_t index_b, const common_data *data_a, const common_data *data_b) {
  return 0;
}

static count_t box_plane_collision(collisions* collisions, count_t index_a, count_t index_b, const common_data *data_a, const common_data *data_b) {
  v3 extents = scale(data_a->shapes[index_a].box.size, 0.5);
  v3 plane_normal = data_b->shapes[index_b].plane.normal;

  v3 corners[] = {
    { extents.x, extents.y, extents.z },
    { extents.x, -extents.y, extents.z },
    { extents.x, -extents.y, -extents.z },
    { extents.x, extents.y, -extents.z },
    { -extents.x, extents.y, extents.z },
    { -extents.x, -extents.y, extents.z },
    { -extents.x, -extents.y, -extents.z },
    { -extents.x, extents.y, -extents.z },
  };

  const count_t max_contacts = 4;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->contacts_count + max_contacts, collisions->contacts_capacity, contact)

  count_t contact_count = 0;
  contact *contacts = collisions->contacts + collisions->contacts_count;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    v3 corner = add(data_a->positions[index_a], rotate(corners[i], data_a->rotations[index_a]));
    float distance = dot(sub(corner, data_b->positions[index_b]), plane_normal);
    if (distance > 0)
      continue;

    contact *new_contact = &contacts[contact_count++];
    new_contact->normal = plane_normal;
    new_contact->point = add(corner, scale(plane_normal, -0.5 * distance));
    new_contact->depth = -distance;
  }

  if (contact_count == 0)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);

  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return 1;
}

count_t box_sphere_collision(collisions *collisions, count_t i, count_t j, const common_data *data_a, const common_data *data_b) {
  return 0;
}

count_t sphere_sphere_collision(collisions *collisions, count_t i, count_t j, const common_data *data_a, const common_data *data_b) {
  v3 p1 = data_a->positions[i];
  v3 p2 = data_b->positions[j];
  float r1 = data_a->shapes[i].sphere.radius;
  float r2 = data_b->shapes[j].sphere.radius;

  v3 offset = sub(p1, p2);
  float distance = len(offset);
  float radii = r1 + r2;
  if (distance > radii) {
    return 0;
  }

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision col = { .index_a = i, .index_b = j, .contacts_count = 1, .contacts_offset = collisions->contacts_count };
  contact contact = {
    .point = add(p1, scale(offset, 0.5)),
    .normal = normalize(offset),
    .depth = radii - distance
  };

  collisions->collisions[collisions->collisions_count++] = col;
  collisions->contacts[collisions->contacts_count++] = contact;

  return 1;
}

static count_t sphere_plane_collision(collisions *collisions, count_t index_a, count_t index_b, const common_data *data_a, const common_data *data_b) {
  v3 plane_point = data_b->positions[index_b];
  v3 plane_normal = data_b->shapes[index_b].plane.normal;
  v3 sphere_center = data_a->positions[index_a];
  float sphere_radius = data_a->shapes[index_a].sphere.radius;

  float plane_sphere_distance = dot(sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision *collision = &collisions->collisions[collisions->collisions_count++];
  contact *contact = &collisions->contacts[collisions->contacts_count];

  collision->index_a = index_a;
  collision->index_b = index_b;
  collision->contacts_offset = collisions->contacts_count;
  collision->contacts_count = 1;

  contact->normal = plane_normal;
  contact->point = add(sphere_center, scale(plane_normal, -plane_sphere_distance));
  contact->depth = sphere_radius - plane_sphere_distance;

  collisions->contacts_count++;

  return 1;
}

m3 contact_space_transform(const contact *contact) {
  v3 y_axis = contact->normal;
  v3 x_axis, z_axis;

  if (fabsf(y_axis.y) > fabsf(y_axis.z)) {
    // Take (1, 0, 0) as initial guess
    const float s = 1.0 / sqrtf(y_axis.y * y_axis.y + y_axis.z * y_axis.z);

    z_axis.x = 0;
    z_axis.y = s * y_axis.z;
    z_axis.z = -s * y_axis.y;

    x_axis.x = z_axis.y * y_axis.z - y_axis.y * z_axis.z;
    x_axis.y = y_axis.x * z_axis.z;
    x_axis.z = y_axis.x * z_axis.y;
  } else {
    // Take (0, 0, 1) as initial guess
    const float s = 1.0 / sqrtf(y_axis.x * y_axis.x + y_axis.y * y_axis.y);

    x_axis.x = -s * y_axis.y;
    x_axis.y = s * y_axis.x;
    x_axis.z = 0;

    z_axis.x = -y_axis.z * x_axis.y;
    z_axis.y = x_axis.x * y_axis.z;
    z_axis.z = y_axis.x * x_axis.y - x_axis.x * y_axis.y;
  }

  return matrix_from_basis(x_axis, y_axis, z_axis);
}

collisions* collisions_init(const physics_config *config) {
  collisions* result =  malloc(sizeof(collisions));

  ARRAY_INIT(result, collision, config->collisions_capacity);
  ARRAY_INIT(result, contact, config->collisions_capacity * 4);

  return result;
}

count_t collisions_count(collisions *collisions) {
  return collisions->collisions_count;
}

bool collision_get(collisions *collisions, count_t index, collision *collision) {
  if (index >= collisions->collisions_count)
    return false;

  *collision = collisions->collisions[index];
  return true;
}

bool contact_get(collisions *collisions, count_t index, contact *contact) {
  if (index >= collisions->contacts_count)
    return false;

  *contact = collisions->contacts[index];
  return true;
}

void contact_update_penetration(collisions *collisions,count_t index, float penetration) {
  if (index >= collisions->contacts_count)
    return;

  collisions->contacts[index].depth = penetration;
}

void collisions_detect(collisions *collisions, const common_data *dynamics, const common_data *statics) {
  collisions->collisions_count = 0;
  collisions->contacts_count = 0;
  collisions->dynamic_collisions_count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; ++j) {
      body_shape shape_a = dynamics->shapes[i];
      body_shape shape_b = dynamics->shapes[j];

      switch(shape_a.type) {
        case SHAPE_BOX:
          switch(shape_b.type) {
            case SHAPE_BOX:
              collisions->dynamic_collisions_count += box_box_collision(collisions, i, j, dynamics, dynamics);
              break;

            case SHAPE_SPHERE:
              collisions->dynamic_collisions_count += box_sphere_collision(collisions, i, j, dynamics, dynamics);
              break;

            default:
              break;
          }

        case SHAPE_SPHERE:
          switch(shape_b.type) {
            case SHAPE_BOX:
              collisions->dynamic_collisions_count += box_sphere_collision(collisions, i, j, dynamics, dynamics);
              break;

            case SHAPE_SPHERE:
              collisions->dynamic_collisions_count += sphere_sphere_collision(collisions, i, j, dynamics, dynamics);
              break;
          }
          break;
      }
    }
  }

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < statics->count; ++j) {
      body_shape shape_a = dynamics->shapes[i];
      body_shape shape_b = statics->shapes[j];

      if (shape_b.type == SHAPE_PLANE) {
        switch (shape_a.type) {
          case SHAPE_BOX:
            box_plane_collision(collisions, i, j, dynamics, statics);
            break;

          case SHAPE_SPHERE:
            sphere_plane_collision(collisions, i, j, dynamics, statics);
            break;

          default:
            break;
        }
      }
    }
  }
}

void collisions_teardown(collisions *collisions) {
  free(collisions->collisions);
  free(collisions->contacts);
  free(collisions);
}
