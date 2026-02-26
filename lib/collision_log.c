#include "physics.h"
#include "pmath.h"

#include <stdio.h>
#include <string.h>

typedef struct {
  bool enabled;
  bool frame_active;
  bool capturing;
  bool wait_for_separation;
  count_t tracked_a;
  count_t tracked_b;
  count_t frames_logged;
  count_t max_frames;
  unsigned long long simulation_frame;
  FILE *file;
} collision_log_state;

static collision_log_state g_log = { 0 };

static bool pair_colliding(const physics_world *world, count_t index_a, count_t index_b) {
  for (count_t i = 0; i < world->collisions->dynamic_collisions_count; ++i) {
    collision c = world->collisions->collisions[i];
    bool same_order = c.index_a == index_a && c.index_b == index_b;
    bool swapped_order = c.index_a == index_b && c.index_b == index_a;
    if (same_order || swapped_order)
      return true;
  }

  return false;
}

static void print_vec3(FILE *file, const char *name, v3 value) {
  fprintf(file, "%s=(%.6f, %.6f, %.6f)", name, value.x, value.y, value.z);
}

static void print_contact(FILE *file, count_t contact_index, const contact *ct) {
  fprintf(file, "      contact=%u\n", contact_index);
  print_vec3(file, "point", ct->point);
  fprintf(file, "\n");
  print_vec3(file, "normal", ct->normal);
  fprintf(file, "\n");
  fprintf(file, "      depth=%.6f\n", ct->depth);
  fprintf(
    file,
    "      basis=[%.6f %.6f %.6f | %.6f %.6f %.6f | %.6f %.6f %.6f]\n",
    ct->basis.m0[0], ct->basis.m0[1], ct->basis.m0[2],
    ct->basis.m1[0], ct->basis.m1[1], ct->basis.m1[2],
    ct->basis.m2[0], ct->basis.m2[1], ct->basis.m2[2]);
  fprintf(file, "      desiredDeltaVelocity=%.6f\n", ct->desired_delta_velocity);
}

void physics_collision_log_enable(const char *path, count_t max_frames) {
  physics_collision_log_disable();

  g_log.file = fopen(path, "w");
  if (!g_log.file)
    return;

  g_log.enabled = true;
  g_log.frame_active = false;
  g_log.capturing = false;
  g_log.wait_for_separation = false;
  g_log.tracked_a = 0;
  g_log.tracked_b = 0;
  g_log.frames_logged = 0;
  g_log.max_frames = max_frames;
  g_log.simulation_frame = 0;
}

void physics_collision_log_disable(void) {
  if (g_log.file) {
    fclose(g_log.file);
    g_log.file = NULL;
  }

  memset(&g_log, 0, sizeof(g_log));
}

void collision_log_begin_frame(const physics_world *world, float dt) {
  g_log.simulation_frame += 1;

  if (!g_log.enabled || !g_log.file) {
    g_log.frame_active = false;
    return;
  }

  g_log.frame_active = false;

  if (g_log.wait_for_separation) {
    if (pair_colliding(world, g_log.tracked_a, g_log.tracked_b))
      return;

    g_log.wait_for_separation = false;
  }

  if (!g_log.capturing) {
    if (world->collisions->dynamic_collisions_count == 0)
      return;

    collision first_dynamic_collision = world->collisions->collisions[0];
    g_log.tracked_a = first_dynamic_collision.index_a;
    g_log.tracked_b = first_dynamic_collision.index_b;
    g_log.capturing = true;
    g_log.frames_logged = 0;
  }

  if (g_log.frames_logged >= g_log.max_frames) {
    g_log.capturing = false;
    g_log.wait_for_separation = true;
    return;
  }

  if (!pair_colliding(world, g_log.tracked_a, g_log.tracked_b)) {
    g_log.capturing = false;
    return;
  }

  g_log.frame_active = true;
  g_log.frames_logged += 1;

  dynamic_bodies dynamics = world->dynamics;

  fprintf(g_log.file, "============================================================\n");
  fprintf(g_log.file, "Frame %llu duration=%.6f boxesColliding=true\n", g_log.simulation_frame, dt);

  count_t ids[2] = { g_log.tracked_a, g_log.tracked_b };
  for (count_t i = 0; i < 2; ++i) {
    count_t body_index = ids[i];
    v3 position = dynamics.positions[body_index];
    quat rotation = dynamics.rotations[body_index];
    v3 velocity = dynamics.velocities[body_index];
    v3 angular_momentum = dynamics.angular_momenta[body_index];
    m3 inv_inertia = matrix_inertia(dynamics.inv_inertia_tensors[body_index], rotation);
    v3 angular_velocity = matrix_rotate(angular_momentum, inv_inertia);
    float motion = dynamics.motion_avgs[body_index];

    fprintf(
      g_log.file,
      "  box[%u]: position=(%.6f, %.6f, %.6f), orientation=(%.6f, %.6f, %.6f, %.6f), velocity=(%.6f, %.6f, %.6f), rotation=(%.6f, %.6f, %.6f), motion=%.6f\n",
      i,
      position.x, position.y, position.z,
      rotation.w, rotation.x, rotation.y, rotation.z,
      velocity.x, velocity.y, velocity.z,
      angular_velocity.x, angular_velocity.y, angular_velocity.z,
      motion);
  }
}

void collision_log_end_frame(void) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fflush(g_log.file);
  g_log.frame_active = false;
}

void collision_log_prepare_contact(count_t contact_index, const contact *ct) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fprintf(g_log.file, "  [prepareContacts]\n");
  print_contact(g_log.file, contact_index, ct);
}

void collision_log_adjust_positions_iteration(count_t iteration) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fprintf(g_log.file, "  [adjustPositions] iteration=%u\n", iteration);
}

void collision_log_penetration(count_t contact_index, const contact *ct, const v3 *deltas, bool is_dynamic) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fprintf(g_log.file, "    [penetration]\n");
  print_contact(g_log.file, contact_index, ct);

  v3 linear1 = deltas[0];
  v3 angular1 = deltas[1];
  v3 linear2 = is_dynamic ? deltas[2] : zero();
  v3 angular2 = is_dynamic ? deltas[3] : zero();

  fprintf(g_log.file, "      linearChange[0]=(%.6f, %.6f, %.6f)\n", linear1.x, linear1.y, linear1.z);
  fprintf(g_log.file, "      linearChange[1]=(%.6f, %.6f, %.6f)\n", linear2.x, linear2.y, linear2.z);
  fprintf(g_log.file, "      angularChange[0]=(%.6f, %.6f, %.6f)\n", angular1.x, angular1.y, angular1.z);
  fprintf(g_log.file, "      angularChange[1]=(%.6f, %.6f, %.6f)\n", angular2.x, angular2.y, angular2.z);
}

void collision_log_depth_update(count_t contact_index, const contact *ct, float before, float after) {
  if (!g_log.frame_active || !g_log.file)
    return;

  float penetration_delta = after - before;

  fprintf(g_log.file, "    [depthUpdate]\n");
  print_contact(g_log.file, contact_index, ct);
  fprintf(g_log.file, "      penetrationDelta=%.6f\n", penetration_delta);
  fprintf(g_log.file, "      newPenetration=%.6f\n", after);
}

void collision_log_adjust_velocities_iteration(count_t iteration) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fprintf(g_log.file, "  [adjustVelocities] iteration=%u\n", iteration);
}

void collision_log_velocity_resolution(count_t contact_index, const contact *ct, const v3 *deltas, bool is_dynamic) {
  if (!g_log.frame_active || !g_log.file)
    return;

  fprintf(g_log.file, "    [velocityResolution]\n");
  print_contact(g_log.file, contact_index, ct);

  v3 velocity1 = deltas[0];
  v3 rotation1 = deltas[1];
  v3 velocity2 = is_dynamic ? deltas[2] : zero();
  v3 rotation2 = is_dynamic ? deltas[3] : zero();

  fprintf(g_log.file, "      velocityChange[0]=(%.6f, %.6f, %.6f)\n", velocity1.x, velocity1.y, velocity1.z);
  fprintf(g_log.file, "      velocityChange[1]=(%.6f, %.6f, %.6f)\n", velocity2.x, velocity2.y, velocity2.z);
  fprintf(g_log.file, "      rotationChange[0]=(%.6f, %.6f, %.6f)\n", rotation1.x, rotation1.y, rotation1.z);
  fprintf(g_log.file, "      rotationChange[1]=(%.6f, %.6f, %.6f)\n", rotation2.x, rotation2.y, rotation2.z);
}

void collision_log_desired_velocity_update(
  count_t contact_index,
  const contact *ct,
  v3 local_velocity_before,
  v3 local_velocity_after,
  float ddv_before,
  float ddv_after
) {
  if (!g_log.frame_active || !g_log.file)
    return;

  v3 local_velocity_delta = sub(local_velocity_after, local_velocity_before);

  fprintf(g_log.file, "    [desiredVelocityUpdate]\n");
  print_contact(g_log.file, contact_index, ct);
  fprintf(
    g_log.file,
    "      contactVelocityDelta=(%.6f, %.6f, %.6f)\n",
    local_velocity_delta.x, local_velocity_delta.y, local_velocity_delta.z);
  fprintf(g_log.file, "      desiredDeltaVelocityOld=%.6f\n", ddv_before);
  fprintf(g_log.file, "      desiredDeltaVelocityNew=%.6f\n", ddv_after);
}
