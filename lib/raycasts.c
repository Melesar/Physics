#include "physics.h"

extern common_data* as_common(physics_world *world, body_type type);

// TODO: pass the body handle to the raycast functions

static bool raycast_box(v3 origin, v3 direction, float max_distance, v3 position, v3 size, quat rotation, raycast_hit *hit) {
  (void) origin;
  (void) direction;
  (void) max_distance;
  (void) position;
  (void) size;
  (void) rotation;
  (void) hit;
  return false;
}

static bool raycast_plane(v3 origin, v3 direction, float max_distance, v3 point, v3 normal, raycast_hit *hit) {
  (void) origin;
  (void) direction;
  (void) max_distance;
  (void) point;
  (void) normal;
  (void) hit;
  return false;
}

static count_t raycast_bodies(physics_world *world, body_type type, v3 origin, v3 direction, float max_distance, count_t hit_count, count_t max_hits, raycast_hit *hits) {
  if (hit_count >= max_hits) {
    return 0;
  }

  count_t num_hits = 0;
  common_data *data = as_common(world, type);
  for(count_t i = 0; i < data->count; ++i) {
    body_shape shape = data->shapes[i];
    switch(shape.type) {
      case SHAPE_BOX:
        num_hits += raycast_box(origin, direction, max_distance, data->positions[i], shape.box.size, data->rotations[i], hits + hit_count + num_hits);
        break;

      case SHAPE_PLANE:
        num_hits += raycast_plane(origin, direction, max_distance, data->positions[i], shape.plane.normal, hits + hit_count + num_hits);
        break;

      default:
        break;
    }
  }

  return num_hits;
}

count_t physics_raycast(physics_world *world, v3 origin, v3 direction, float max_distance, count_t max_hits, raycast_hit *hits) {
  count_t hit_count = 0;

  hit_count += raycast_bodies(world, BODY_DYNAMIC, origin, direction, max_distance, hit_count, max_hits, hits);
  hit_count += raycast_bodies(world, BODY_STATIC, origin, direction, max_distance, hit_count, max_hits, hits);

  return hit_count;
}
