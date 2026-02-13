#include "physics.h"
#include "string.h"

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

    toggle_pause(true);
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
    case CDBG_PENETRATION_RESOLVE:
      return "Penetration resolve";

    case CDBG_DEPTH_UPDATE:
      return "Depth update";

    case CDBG_VELOCITY_RESOLVE:
      return "Velocity resolve";

    case CDBG_VELOCITY_UPDATE:
      return "Velocity update";

    case CDBG_DONE:
      return "Done";

    case CDBG_IDLE:
      return "Idle";
  }
  return "Unknown";
}

void physics_draw_config_widget(physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_config_widget";
  const float row_height = 15.0f;
  const float window_width = 260.0f;
  const int row_count = 6;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics config", 20.0f, 360.0f, window_width, row_height, row_count);

  if (draw_content) {
    int max_penetration_iterations = (int) world->config.max_penentration_iterations;
    int max_velocity_iterations = (int) world->config.max_velocity_iterations;

    draw_edit_float(ctx, "Linear damping", &world->config.linear_damping);
    draw_edit_float(ctx, "Angular damping", &world->config.angular_damping);
    draw_edit_float(ctx, "Restitution", &world->config.restitution);
    draw_edit_float(ctx, "Friction", &world->config.friction);
    draw_edit_int(ctx, "Penetration iterations", &max_penetration_iterations);
    draw_edit_int(ctx, "Velocity iterations", &max_velocity_iterations);
    draw_edit_float(ctx, "Penetration epsilon", &world->config.penetration_epsilon);
    draw_edit_float(ctx, "Velocity epsilon", &world->config.velocity_epsilon);
    draw_edit_float(ctx, "Restitution damp limit", &world->config.restitution_damping_limit);

    if (max_penetration_iterations < 0)
      max_penetration_iterations = 0;
    if (max_velocity_iterations < 0)
      max_velocity_iterations = 0;

    world->config.max_penentration_iterations = (count_t) max_penetration_iterations;
    world->config.max_velocity_iterations = (count_t) max_velocity_iterations;
  }

  nk_end(ctx);
}

void physics_draw_stats(const physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_world_stats";
  const float row_height = 18.0f;
  const float window_width = 240.0f;
  const int row_count = 4;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics world stats", 20.0f, 200.0f, window_width, row_height, row_count);

  if (draw_content) {
    count_t dynamic_count = world->dynamics.count;
    count_t static_count = world->statics.count;
    count_t collisions_total = world->collisions->collisions_count;
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
