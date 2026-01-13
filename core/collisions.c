#include "collisions.h"
#include "physics.h"
#include "raymath.h"
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


  bool collision_found = false;
  count_t contact_count = 0;
  for (count_t i = 0; i < 8; ++i) {
    Vector3 corner = add(data_a->positions[index_a], rotate(corners[i], data_a->rotations[index_a]));
    float distance = dot(sub(corner, data_b->positions[index_b]), data_b->shapes[index_b].plane.normal);
    if (distance > 0)
      continue;

    collision_found = true;
  }

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return contact_count;
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

void collisions_detect(collisions* collisions, const common_data *dynamics, const common_data *statics) {
  collisions->collisions_count = 0;
  collisions->contacts_count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < statics->count; ++j) {
      body_shape dynamic_shape = dynamics->shapes[i];
      body_shape static_shape = statics->shapes[j];

      if (dynamic_shape.type == SHAPE_BOX && static_shape.type == SHAPE_PLANE) {
        box_plane_collision(collisions, i, j, dynamics, statics);
      }
    }
  }
}

void collisions_teardown(collisions *collisions) {
  free(collisions->collisions);
  free(collisions->contacts);
  free(collisions);
}
