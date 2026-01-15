#include "collisions.h"
#include "physics.h"
#include "raymath.h"
#include <math.h>
#include <stdlib.h>

#define ARRAY(type) \
  count_t type##s_capacity; \
  count_t type##s_count; \
  type* type##s;

#define ARRAY_INIT(base, type, capacity) \
  base->type##s_capacity = capacity; \
  base->type##s_count = 0; \
  base->type##s = malloc(base->type##s_capacity * sizeof(type));

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type) \
  if (count >= capacity) { \
    capacity *= 2; \
    array = realloc(array, capacity * sizeof(type)); \
  }

struct collisions {
  ARRAY(collision)
  ARRAY(contact)
};

static count_t box_plane_collision(collisions* collisions, count_t index_a, count_t index_b, const common_data *data_a, const common_data *data_b) {

  Vector3 extents = scale(data_a->shapes[index_a].box.size, 0.5);
  Vector3 plane_normal = data_b->shapes[index_b].plane.normal;

  Vector3 corners[] = {
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
    Vector3 corner = add(data_a->positions[index_a], rotate(corners[i], data_a->rotations[index_a]));
    float distance = dot(sub(corner, data_b->positions[index_b]), plane_normal);
    if (distance > 0)
      continue;

    contact *new_contact = &contacts[contact_count++];
    new_contact->normal = plane_normal;
    new_contact->point = add(corner, scale(plane_normal, -0.5 * distance));
    new_contact->depth = -distance;
  }

  if (contact_count == 0)
    return contact_count;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);

  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return contact_count;
}

static Matrix contact_basis(const contact *contact) {
  Vector3 y_axis = contact->normal;
  Vector3 x_axis, z_axis;

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

  Vector4 basis[] = {
    { x_axis.x, y_axis.x, z_axis.x, 0 },
    { x_axis.y, y_axis.y, z_axis.y, 0 },
    { x_axis.z, y_axis.z, z_axis.z, 0 },
    { 0 }
  };

  return *(Matrix*)basis;
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

bool contact_get(collisions *collisions, count_t index, const collision* collision, contact *contact) {
  count_t i = collision->contacts_offset + index;

  if (i >= collisions->contacts_count || index >= collision->contacts_count)
    return false;

  *contact = collisions->contacts[i];
  return true;
}

void collisions_detect(collisions* collisions, const common_data *data_a, const common_data *data_b) {
  collisions->collisions_count = 0;
  collisions->contacts_count = 0;

  for (count_t i = 0; i < data_a->count; ++i) {
    for (count_t j = 0; j < data_b->count; ++j) {
      if (data_a == data_b && i == j)
        continue;

      body_shape shape_a = data_a->shapes[i];
      body_shape shape_b = data_b->shapes[j];

      if (shape_a.type == SHAPE_BOX && shape_b.type == SHAPE_PLANE) {
        box_plane_collision(collisions, i, j, data_a, data_b);
      }
    }
  }
}

void collisions_teardown(collisions *collisions) {
  free(collisions->collisions);
  free(collisions->contacts);
  free(collisions);
}
