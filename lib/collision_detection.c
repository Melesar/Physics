#include "physics.h"
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

typedef struct {
  v3 center;
  v3 size;
  v3 axis[3];
} collision_box;

collision_box collision_box_make(const common_data *data, count_t index) {
  quat rotation = data->rotations[index];

  return (collision_box) {
    .center = data->positions[index],
    .size = data->shapes[index].box.size,
    .axis = { rotate(right(), rotation), rotate(up(), rotation), rotate(forward(), rotation) }
  };
}

m4 collision_box_transform(const collision_box *box) {
  m4 transform = { 0 };

  transform.m0 = box->axis[0].x;
  transform.m1 = box->axis[0].y;
  transform.m2 = box->axis[0].z;

  transform.m4 = box->axis[1].x;
  transform.m5 = box->axis[1].y;
  transform.m6 = box->axis[1].z;

  transform.m8 = box->axis[2].x;
  transform.m9 = box->axis[2].y;
  transform.m10 = box->axis[2].z;

  transform.m12 = box->center.x;
  transform.m13 = box->center.y;
  transform.m14 = box->center.z;

  transform.m15 = 1;

  return transform;
}

static v3 contact_point(
  v3 point_a, v3 axis_a, float half_size_a,
  v3 point_b, v3 axis_b, float half_size_b,
  bool use_a
) {
  float len_a = lensq(axis_a);
  float len_b = lensq(axis_b);
  float dp = dot(axis_a, axis_b);

  v3 offset = sub(point_a, point_b);

  float dp_a = dot(axis_a, offset);
  float dp_b = dot(axis_b, offset);

  float denom = len_a * len_b - dp * dp;

  if (fabsf(denom) < 0.0001f)
    return use_a ? point_a : point_b;

  float a = (dp * dp_b - len_b * dp_a) / denom;
  float b = (len_a * dp_b - dp * dp_a) / denom;

  if (a > half_size_a || a < -half_size_a || b > half_size_b || b < -half_size_b)
    return use_a ? point_a : point_b;

  v3 c_a = add(point_a, scale(axis_a, a));
  v3 c_b = add(point_b, scale(axis_b, b));

  return add(scale(c_a, 0.5), scale(c_b, 0.5));
}


static void fill_point_face_box_box(
  const collision_box *box_a,
  const collision_box *box_b,
  v3 offset,
  count_t best_axis,
  float penetration,
  collisions *collisions) {

  v3 normal = box_a->axis[best_axis];
  if (dot(box_a->axis[best_axis], offset) > 0)
    normal = scale(normal, -1);

  v3 vertex = scale(box_b->size, 0.5);
  if (dot(box_b->axis[0], normal) < 0) vertex.x = -vertex.x;
  if (dot(box_b->axis[1], normal) < 0) vertex.y = -vertex.y;
  if (dot(box_b->axis[2], normal) < 0) vertex.z = -vertex.z;

  contact *contact = &collisions->contacts[collisions->contacts_count++];
  contact->point = transform(vertex, collision_box_transform(box_b));
  contact->normal = normal;
  contact->depth = penetration;
}

static float transform_to_axis(const collision_box *box, v3 axis) {
  v3 half_size = scale(box->size, 0.5);
  return
    half_size.x * fabsf(dot(axis, box->axis[0])) +
    half_size.y * fabsf(dot(axis, box->axis[1])) +
    half_size.z * fabsf(dot(axis, box->axis[2]));
}

static bool try_axis(const collision_box *box_a, const collision_box *box_b, v3 axis, count_t index, v3 offset, float *min_penetration, count_t *min_index) {
  if (lensq(axis) < 0.0001)
    return true;

  axis = normalize(axis);

  float project_a = transform_to_axis(box_a, axis);
  float project_b = transform_to_axis(box_b, axis);

  float distance = fabsf(dot(offset, axis));
  float penetration = project_a + project_b - distance;

  if (penetration < 0)
    return false;

  if (penetration < *min_penetration) {
    *min_penetration = penetration;
    *min_index = index;
  }

  return true;
}

#define CHECK_OVERLAP(axis, index) \
  if (!try_axis(&box_a, &box_b, (axis), (index), offset, &penetration, &best_axis)) return 0;

static count_t box_box_collision(collisions* collisions, count_t index_a, count_t index_b, const common_data *data_a, const common_data *data_b) {
  collision_box box_a = collision_box_make(data_a, index_a);
  collision_box box_b = collision_box_make(data_b, index_b);

  v3 offset = sub(box_b.center, box_a.center);

  float penetration = INFINITY;
  count_t best_axis = -1;

  CHECK_OVERLAP(box_a.axis[0], 0)
  CHECK_OVERLAP(box_a.axis[1], 1)
  CHECK_OVERLAP(box_a.axis[2], 2)

  CHECK_OVERLAP(box_b.axis[0], 3)
  CHECK_OVERLAP(box_b.axis[1], 4)
  CHECK_OVERLAP(box_b.axis[2], 5)

  count_t best_single_axis = best_axis;

  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[0]), 6)
  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[1]), 7)
  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[2]), 8)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[0]), 9)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[1]), 10)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[2]), 11)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[0]), 12)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[1]), 13)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[2]), 14)

  if (best_axis == (count_t)-1) {
    return 0;
  }

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision *collision = &collisions->collisions[collisions->collisions_count++];
  collision->index_a = index_a;
  collision->index_b = index_b;
  collision->contacts_count = 1;
  collision->contacts_offset = collisions->contacts_count;

  if (best_axis < 3) {
    // We've got a vertex of box two on a face of box one.
    fill_point_face_box_box(&box_a, &box_b, offset, best_axis, penetration, collisions);
    return 1;
  } else if (best_axis < 6) {
    // We've got a vertex of box one on a face of box two.
    // We use the same algorithm as above, but swap around
    // one and two (and therefore also the vector between their
    // centres).
    fill_point_face_box_box(&box_b, &box_a, scale(offset, -1), best_axis - 3, penetration, collisions);
    return 1;
  } else {
    // We've got an edge-edge contact. Find out which axes
    best_axis -= 6;
    count_t axis_a_index = best_axis / 3;
    count_t axis_b_index = best_axis % 3;
    v3 axis_a = box_a.axis[axis_a_index];
    v3 axis_b = box_b.axis[axis_b_index];
    v3 axis = normalize(cross(axis_a, axis_b));

    if (dot(axis, offset) > 0)
      axis = scale(axis, -1);

    v3 half_size_a = scale(box_a.size, 0.5);
    v3 half_size_b = scale(box_b.size, 0.5);
    v3 edge_point_a = half_size_a;
    v3 edge_point_b = half_size_b;
    for (count_t i = 0; i < 3; ++i) {
      float *p_a = (float*)&edge_point_a;
      float *p_b = (float*)&edge_point_b;

      if (i == axis_a_index)
        p_a[i] = 0;
      else if (dot(box_a.axis[i], axis) > 0)
        p_a[i] = -p_a[i];

      if (i == axis_b_index)
        p_b[i] = 0;
      else if (dot(box_b.axis[i], axis) < 0)
        p_b[i] = -p_b[i];
    }

    edge_point_a = transform(edge_point_a, collision_box_transform(&box_a));
    edge_point_b = transform(edge_point_b, collision_box_transform(&box_b));

    v3 vertex = contact_point(
      edge_point_a, axis_a, *((float*)&half_size_a + axis_a_index),
      edge_point_b, axis_b, *((float*)&half_size_b + axis_b_index),
      best_single_axis > 2);


    contact *contact = &collisions->contacts[collisions->contacts_count++];

    contact->point = vertex;
    contact->normal = axis;
    contact->depth = penetration;

    return 1;
  }

  return 0;
}

#undef CHECK_OVERLAP

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

  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + max_contacts, collisions->contacts_capacity, contact)

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
  v3 box_center = data_a->positions[i];
  quat box_rotation = data_a->rotations[i];
  v3 box_half_extents = scale(data_a->shapes[i].box.size, 0.5);

  v3 sphere_center = data_b->positions[j];
  float sphere_radius = data_b->shapes[j].sphere.radius;

  m4 box_transform = mul(as_matrix(box_rotation), MatrixTranslate(box_center.x, box_center.y, box_center.z));
  v3 relative_sphere_center = transform(sphere_center, inverse(box_transform));

  if (fabsf(relative_sphere_center.x) > box_half_extents.x + sphere_radius ||
      fabsf(relative_sphere_center.y) > box_half_extents.y + sphere_radius ||
      fabsf(relative_sphere_center.z) > box_half_extents.z + sphere_radius) {
    return 0;
  }

  v3 closest_point;
  float distance = relative_sphere_center.x;
  if (relative_sphere_center.x > box_half_extents.x) distance = box_half_extents.x;
  else if (relative_sphere_center.x < -box_half_extents.x) distance = -box_half_extents.x;
  closest_point.x = distance;

  distance = relative_sphere_center.y;
  if (relative_sphere_center.y > box_half_extents.y) distance = box_half_extents.y;
  else if (relative_sphere_center.y < -box_half_extents.y) distance = -box_half_extents.y;
  closest_point.y = distance;

  distance = relative_sphere_center.z;
  if (relative_sphere_center.z > box_half_extents.z) distance = box_half_extents.z;
  else if (relative_sphere_center.z < -box_half_extents.z) distance = -box_half_extents.z;
  closest_point.z = distance;

  distance = distancesqr(closest_point, relative_sphere_center);
  if (distance > sphere_radius * sphere_radius)
    return 0;

  closest_point = transform(closest_point, box_transform);

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->contacts_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision *collision = &collisions->collisions[collisions->collisions_count++];
  collision->contacts_count = 1;
  collision->contacts_offset = collisions->contacts_count;
  collision->index_a = i;
  collision->index_b = j;

  contact *contact = &collisions->contacts[collisions->contacts_count++];
  contact->point = closest_point;
  contact->normal = normalize(sub(closest_point, sphere_center));
  contact->depth = sphere_radius - sqrtf(distance);

  return 1;
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

collisions* collisions_init(const physics_config *config) {
  collisions* result =  malloc(sizeof(collisions));

  ARRAY_INIT(result, collision, config->collisions_capacity);
  ARRAY_INIT(result, contact, config->collisions_capacity * 4);

  return result;
}

void collisions_detect(collisions *collisions, const common_data *dynamics, const common_data *statics) {
  collisions->collisions_count = 0;
  collisions->contacts_count = 0;

  count_t dyn_count = 0;
  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; ++j) {
      body_shape shape_a = dynamics->shapes[i];
      body_shape shape_b = dynamics->shapes[j];

      switch(shape_a.type) {
        case SHAPE_BOX:
          switch(shape_b.type) {
            case SHAPE_BOX:
              dyn_count += box_box_collision(collisions, i, j, dynamics, dynamics);
              break;

            case SHAPE_SPHERE:
              dyn_count += box_sphere_collision(collisions, i, j, dynamics, dynamics);
              break;

            default:
              break;
          }
          break;

        case SHAPE_SPHERE:
          switch(shape_b.type) {
            case SHAPE_BOX:
              dyn_count += box_sphere_collision(collisions, j, i, dynamics, dynamics);
              break;

            case SHAPE_SPHERE:
              dyn_count += sphere_sphere_collision(collisions, i, j, dynamics, dynamics);
              break;

            default:
              break;
          }
          break;

        default:
          break;
      }
    }
  }

  collisions->dynamic_collisions_count = dyn_count;

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
