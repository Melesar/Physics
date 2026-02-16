#include "core.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <float.h>
#include <limits.h>

Mesh arrow_base;
Mesh arrow_head;
Material mat;

static void begin_debug_row(struct nk_context* ctx) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1f);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9f);
}

static void set_arrow_color(Color c) {
  mat.maps[MATERIAL_MAP_DIFFUSE].color = c;
}

void init_debugging() {
  arrow_base = GenMeshCylinder(0.1, 1, 8);
  arrow_head = GenMeshCone(0.2, 0.5, 8);
  mat = LoadMaterialDefault();
}

void draw_arrow(Vector3 start, Vector3 direction, Color color) {
  Vector3 end = Vector3Add(start, direction);
  float distance = Vector3Length(direction);
  Vector3 n = Vector3Scale(direction, 1.0 / distance);

  set_arrow_color(color);

  Matrix base_translation = MatrixTranslate(start.x, start.y, start.z);
  Matrix base_rotation = QuaternionToMatrix(QuaternionFromVector3ToVector3((Vector3) { 0, 1, 0 }, n));
  Matrix base_scale = MatrixScale(1, distance, 1);
  Matrix base_transform = MatrixMultiply(MatrixMultiply(base_scale, base_rotation), base_translation);

  Matrix head_translation = MatrixTranslate(end.x, end.y, end.z);
  Matrix head_rotation = base_rotation;
  Matrix head_transform = MatrixMultiply(head_rotation, head_translation);

  DrawMesh(arrow_base, mat, base_transform);
  DrawMesh(arrow_head, mat, head_transform);
}

void draw_stat_float(struct nk_context* ctx, char* title, float value) {
  begin_debug_row(ctx);
  nk_value_float(ctx, title, value);
  nk_layout_row_end(ctx);
}

void draw_stat_int(struct nk_context* ctx, char* title, int value) {
  begin_debug_row(ctx);
  nk_value_int(ctx, title, value);
  nk_layout_row_end(ctx);
}

void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value) {
  begin_debug_row(ctx);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s: (%.3f, %.3f, %.3f)", title, value.x, value.y, value.z);
  nk_layout_row_end(ctx);
}

void draw_stat_matrix(struct nk_context* ctx, char* title, Matrix value) {
  begin_debug_row(ctx);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s:", title);
  nk_layout_row_end(ctx);

  for (int i = 0; i < 4; ++i) {
    float *m = ((float*)(&value) + i * 4);
    nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 1);
    nk_layout_row_push(ctx, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "\t%.3f\t%.3f\t%.3f\t%.3f", m[0], m[1], m[2], m[3]);
    nk_layout_row_end(ctx);
  }
}

bool begin_widget_window(
  struct nk_context* ctx,
  const char* window_name,
  const char* title,
  float x,
  float y,
  float width,
  float row_height,
  int row_count
) {
  const nk_flags window_flags = NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_MINIMIZABLE |
    NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_TITLE;

  float header_height = ctx->style.font->height + ctx->style.window.header.padding.y * 2.0f;
  float padding_y = ctx->style.window.padding.y;
  float spacing_y = ctx->style.window.spacing.y;
  float content_height = (row_height * row_count) + (spacing_y * (row_count - 1));
  float window_height = header_height + (padding_y * 2.0f) + content_height + 25.0;

  if (nk_begin_titled(ctx, window_name, title, nk_rect(x, y, width, window_height), window_flags)) {
    if (!nk_window_is_collapsed(ctx, window_name)) {
      nk_window_set_size(ctx, window_name, nk_vec2(width, window_height));
      return true;
    }
  }

  return false;
}

void draw_edit_float(struct nk_context* ctx, char* title, float* value) {
  draw_property_float(ctx, title, value, -FLT_MAX, FLT_MAX, 0.1f, 0.01f);
}

void draw_edit_int(struct nk_context* ctx, char* title, int* value) {
  draw_property_int(ctx, title, value, INT_MIN, INT_MAX, 1, 1.0f);
}

bool draw_button(struct nk_context* ctx, char* title) {
  bool pressed = false;

  begin_debug_row(ctx);
  pressed = nk_button_label(ctx, title) != 0;
  nk_layout_row_end(ctx);

  return pressed;
}

bool draw_selectable(struct nk_context* ctx, char* title, bool* selected) {
  nk_bool value = *selected ? nk_true : nk_false;
  nk_bool changed;

  begin_debug_row(ctx);
  changed = nk_selectable_label(ctx, title, NK_TEXT_ALIGN_LEFT, &value);
  nk_layout_row_end(ctx);

  *selected = value != 0;
  return changed != 0;
}

void draw_property_float(struct nk_context* ctx, char* title, float* value, float min, float max, float step_arrow, float step_drag) {
  begin_debug_row(ctx);
  nk_property_float(ctx, title, min, value, max, step_arrow, step_drag);
  nk_layout_row_end(ctx);
}

void draw_property_int(struct nk_context* ctx, char* title, int* value, int min, int max, int step, float step_drag) {
  begin_debug_row(ctx);
  nk_property_int(ctx, title, min, value, max, step, step_drag);
  nk_layout_row_end(ctx);
}

void draw_model_with_wireframe(Model model, Vector3 position, float scale, Color color) {
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
  DrawModel(model, position, scale, WHITE);

  rlEnableWireMode();
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_WIREFRAME;
  DrawModel(model, position, 1.01 * scale, COLOR_WIREFRAME);
  rlDisableWireMode();
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

void physics_draw_collisions(const physics_world *world) {
  count_t count = world->collisions->collisions_count;

  for (count_t i = 0; i < count; ++i) {
    collision c = world->collisions->collisions[i];

    for (count_t j = 0; j < c.contacts_count; ++j) {
      contact contact = world->collisions->contacts[c.contacts_offset + j];

      draw_arrow(contact.point, contact.normal, RED);
    }
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
