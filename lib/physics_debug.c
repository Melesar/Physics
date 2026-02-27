#include "physics.h"
#include "string.h"

extern void clear_forces(physics_world *world);
extern void integrate_bodies(physics_world *world, float dt);
extern void prepare_contacts(physics_world *world, float dt);
extern void resolve_interpenetration_contact(physics_world *world, count_t collision_index, const contact *contact, v3 *deltas);
extern void update_penetration_depths_ex(physics_world *world, count_t collision_index, const v3 *deltas, depth_update_record *records, count_t *record_count);
extern void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas, v3 *world_space_impulse);
extern bool find_worst_penetration(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
extern bool find_worst_velocity(physics_world *world, count_t *out_collision_index, count_t *out_contact_index);
extern void update_awake_status_for_collision(physics_world *world, count_t collision_index);
extern void update_velocity_deltas_ex(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt, velocity_update_record *records, count_t *record_count);
extern void resolve_collisions(physics_world *world, float dt);
extern void collisions_detect(collisions* collisions, const common_data *dynamics, const common_data *statics);
extern void update_awake_statuses(physics_world *world, float dt);

void physics_debug_state_init(collision_debug_state *state) {
  state->active = false;
  state->phase = CDBG_IDLE;
  state->iteration = 0;
  state->is_dynamic = false;
  state->current_contact_index = (count_t)-1;
  state->dt = 0;
  state->depth_update_count = 0;
  state->velocity_update_count = 0;

  memset(state->deltas, 0, sizeof(v3) * 4);
}

void physics_step_debug(physics_world *world, float dt, collision_debug_state *state) {
  switch(state->phase) {
    case CDBG_IDLE:
      integrate_bodies(world, dt);
      collisions_detect(world->collisions, (common_data*) &world->dynamics, &world->statics);

      if (world->collisions->dynamic_collisions_count == 0) {
        resolve_collisions(world, dt);
        update_awake_statuses(world, dt);
        clear_forces(world);

        state->active = false;
        state->prev_phase = CDBG_IDLE;
        return;
      }

      state->active = true;
      state->prev_phase = CDBG_IDLE;
      state->phase = CDBG_PENETRATION_RESOLVE;
      state->dt = dt;
      state->iteration = 0;

      prepare_contacts(world, dt);
      break;

    case CDBG_PENETRATION_RESOLVE:
      if (state->iteration < world->config.max_penentration_iterations) {
        count_t worst_collision, worst_contact;
        if (find_worst_penetration(world, &worst_collision, &worst_contact)) {
          state->is_dynamic = worst_collision < world->collisions->dynamic_collisions_count;
          state->current_collision_index = worst_collision;
          state->current_contact_index = worst_contact;
          state->prev_phase = CDBG_PENETRATION_RESOLVE;
          state->phase = CDBG_DEPTH_UPDATE;

          update_awake_status_for_collision(world, worst_collision);
          resolve_interpenetration_contact(world, worst_collision, &world->collisions->contacts[worst_contact], state->deltas);
        } else {
          state->prev_phase = CDBG_DEPTH_UPDATE;
          state->phase = CDBG_VELOCITY_RESOLVE;
          state->iteration = 0;
        }
      } else {
        state->prev_phase = CDBG_DEPTH_UPDATE;
        state->phase = CDBG_VELOCITY_RESOLVE;
        state->iteration = 0;
      }
      break;

    case CDBG_DEPTH_UPDATE:
      update_penetration_depths_ex(world, state->current_collision_index, state->deltas, state->depth_updates, &state->depth_update_count);
      state->iteration += 1;
      state->prev_phase = CDBG_DEPTH_UPDATE;
      state->phase = CDBG_PENETRATION_RESOLVE;
      break;

    case CDBG_VELOCITY_RESOLVE:
      if (state->iteration < world->config.max_velocity_iterations) {
        count_t worst_collisions, worst_contact;
        if (find_worst_velocity(world, &worst_collisions, &worst_contact)) {
          state->is_dynamic = worst_collisions < world->collisions->dynamic_collisions_count;
          state->current_collision_index = worst_collisions;
          state->current_contact_index = worst_contact;
          state->phase = CDBG_VELOCITY_UPDATE;
          state->prev_phase = CDBG_VELOCITY_RESOLVE;

          v3 world_space_contact;
          update_awake_status_for_collision(world, worst_collisions);
          resolve_velocity_contact(world, worst_collisions, &world->collisions->contacts[worst_contact], state->deltas, &world_space_contact);
        } else {
          state->iteration = 0;
          state->prev_phase = CDBG_VELOCITY_UPDATE;
          state->phase = CDBG_DONE;
        }
      } else {
        state->iteration = 0;
        state->prev_phase = CDBG_VELOCITY_UPDATE;
        state->phase = CDBG_DONE;
      }
      break;

    case CDBG_VELOCITY_UPDATE:
      update_velocity_deltas_ex(world, state->current_collision_index, state->deltas, state->dt, state->velocity_updates, &state->velocity_update_count);
      state->iteration += 1;
      state->phase = CDBG_VELOCITY_RESOLVE;
      state->prev_phase = CDBG_VELOCITY_UPDATE;
      break;

    case CDBG_DONE:
      update_awake_statuses(world, dt);
      clear_forces(world);

      state->active = false;
      state->phase = CDBG_IDLE;
      state->prev_phase = CDBG_DONE;
      break;
  }
}
