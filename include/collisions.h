#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "core.h"
#include "physics.h"

typedef struct {
  count_t index_a, index_b;
  count_t contacts_offset, contacts_count;
} collision;

typedef struct {
  Vector3 point;
  Vector3 normal;
  float depth;
} contact;

typedef struct collisions collisions;

collisions* collisions_init(const physics_config *config);

count_t collisions_count(collisions *collisions);
bool collision_get(collisions *collisions, count_t index, collision *collision);

bool contact_get(collisions *collisions, count_t index, const collision* collision, contact *contact);
Matrix contact_space_transform(const contact* contact);

void collisions_detect(collisions* collisions, const common_data *dynamics, const common_data *statics);

void collisions_teardown(collisions *collisions);

#endif
