#include "physics.h"
#include "string.h"

extern void clear_forces(physics_world *world);
extern void integrate_bodies(physics_world *world, float dt);
extern void prepare_contacts(physics_world *world, float dt);
extern void resolve_interpenetration_contact(physics_world *world, count_t collision_index, const contact *contact, v3 *deltas);
extern void update_penetration_depths_ex(physics_world *world, count_t collision_index, const v3 *deltas, depth_update_record *records, count_t *record_count);
extern void resolve_velocity_contact(physics_world *world, count_t worst_collision_index, contact *contact, v3 *deltas);
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
  state->penetration_done = false;
  state->velocity_done = false;
  state->needs_integration = true;
  state->depth_update_count = 0;
  state->velocity_update_count = 0;

  memset(state->deltas, 0, sizeof(v3) * 4);
}

void physics_step_debug(physics_world *world, float dt, collision_debug_state *state) {
  // Phase: integration + collision detection (runs once per frame)
  if (state->needs_integration) {
    integrate_bodies(world, dt);
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
    count_t collision_index, contact_index;
    if (find_worst_penetration(world, &collision_index, &contact_index)) {
      state->iteration = 1;
      state->is_dynamic = collision_index < world->collisions->dynamic_collisions_count;
      state->current_contact_index = contact_index;
      state->current_collision_index = collision_index;

      update_awake_status_for_collision(world, collision_index);

      contact *ct = &world->collisions->contacts[contact_index];
      resolve_interpenetration_contact(world, collision_index, ct, state->deltas);
      state->phase = CDBG_PENETRATION_RESOLVE;
    } else {
      state->penetration_done = true;
      // Skip to velocity
      if (find_worst_velocity(world, &collision_index, &contact_index)) {
        state->iteration = 1;
        state->is_dynamic = collision_index < world->collisions->dynamic_collisions_count;
        state->current_collision_index = collision_index;
        state->current_contact_index = contact_index;

        update_awake_status_for_collision(world, collision_index);

        contact *ct = &world->collisions->contacts[contact_index];
        resolve_velocity_contact(world, collision_index, ct, state->deltas);
        state->phase = CDBG_VELOCITY_RESOLVE;
      } else {
        state->phase = CDBG_DONE;
      }
    }

    return;
  }

  // State machine: advance one sub-step per call
  count_t collision_index, contact_index;

  switch (state->phase) {
    case CDBG_PENETRATION_RESOLVE: {
       update_penetration_depths_ex(world, state->current_collision_index, state->deltas, state->depth_updates, &state->depth_update_count);
       state->phase = CDBG_DEPTH_UPDATE;
      break;
    }

     case CDBG_DEPTH_UPDATE: {
       // Depth update shown. Try next penetration iteration.
       if (state->iteration < world->config.max_penentration_iterations && find_worst_penetration(world, &collision_index, &contact_index)) {
         state->iteration++;
         state->is_dynamic = collision_index < world->collisions->dynamic_collisions_count;
         state->current_contact_index = contact_index;
         state->current_collision_index = collision_index;

         update_awake_status_for_collision(world, collision_index);

         contact *ct = &world->collisions->contacts[contact_index];
         resolve_interpenetration_contact(world, collision_index, ct, state->deltas);
         state->phase = CDBG_PENETRATION_RESOLVE;
       } else {
         // Penetration done, switch to velocity resolution
         state->penetration_done = true;
         state->iteration = 0;

         if (find_worst_velocity(world, &collision_index, &contact_index)) {
           state->iteration = 1;
           state->is_dynamic = collision_index < world->collisions->dynamic_collisions_count;
           state->current_contact_index = contact_index;
           state->current_collision_index = collision_index;

           update_awake_status_for_collision(world, collision_index);

           contact *ct = &world->collisions->contacts[contact_index];
           resolve_velocity_contact(world, collision_index, ct, state->deltas);
           state->phase = CDBG_VELOCITY_RESOLVE;
         } else {
           state->phase = CDBG_DONE;
         }
       }
       break;
     }

     case CDBG_VELOCITY_RESOLVE: {
       update_velocity_deltas_ex(world, state->current_collision_index, state->deltas, state->dt, state->velocity_updates, &state->velocity_update_count);
       state->phase = CDBG_VELOCITY_UPDATE;
       break;
     }

     case CDBG_VELOCITY_UPDATE: {
       // Velocity update shown. Try next velocity iteration.
       if (state->iteration < world->config.max_penentration_iterations && find_worst_velocity(world, &collision_index, &contact_index)) {
         state->iteration++;
         state->is_dynamic = collision_index < world->collisions->dynamic_collisions_count;
         state->current_contact_index = contact_index;
         state->current_collision_index = collision_index;

         update_awake_status_for_collision(world, collision_index);

         contact *ct = &world->collisions->contacts[contact_index];
         resolve_velocity_contact(world, collision_index, ct, state->deltas);
         state->phase = CDBG_VELOCITY_RESOLVE;
       } else {
         state->phase = CDBG_DONE;
       }
       break;
     }

    case CDBG_DONE: {
      // All done — finish the frame
      update_awake_statuses(world, state->dt);
      clear_forces(world);

      state->active = false;
      state->phase = CDBG_IDLE;
      state->needs_integration = true;
      break;
    }

    case CDBG_IDLE:
      break;
  }
}
