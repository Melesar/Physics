#include "raylib.h"
#include "core.h"
#include "raymath.h"
#include "string.h"
#include "rlgl.h"
#include "pmath.h"
#include <stdlib.h>
#include <stdio.h>

#define RLIGHTS_IMPLEMENTATION
#include "shaders/rlights.h"
#include "rcamera.h"
#include "raylib-nuklear.h"
#include "physics.h"

const int ui_font_size = 14;
const int screen_width = 1920;
const int screen_height = 1080;
const int frame_rate = 60;
const int simulation_rate = 120;

const float simulation_step = 1.0 / simulation_rate;

typedef struct {
  float movement_speed;
  float rotation_sensitivity;
} camera_settings;

void init_debugging();
void init_gizmos();
void manipulate_gizmos(Camera *camera);
void draw_gizmos();

static void draw_custom_grid(int slices, float spacing);
static void setup_scene(Shader shader);
static void init_physics();
static Shader setup_lighting();
static Camera setup_camera(program_config config);
static void update_camera(Camera* camera, float deltaTime);
static void draw_scene(Camera camera, struct nk_context* ctx, Shader shader);
static void draw_physics_bodies();
static void draw_body_axes(v3 position, quat rotation);
static void draw_body_angular_momentum(v3 position, v3 angular_momentum);
static void process_inputs(physics_world *world, Camera* camera);
static void reset();
static void draw_ui_widget_controls(struct nk_context* ctx);
static void draw_ui_widget_observe_body(struct nk_context* ctx);
static void print_diagnostics(physics_world *world);

extern void scenario_initialize(program_config* config, physics_config *physics_config);
extern void scenario_setup_scene(physics_world *world);
extern void scenario_handle_input(physics_world *world, Camera *camera);
extern void scenario_simulate(physics_world *world, float dt);
extern void scenario_draw_scene(physics_world *world);
extern void scenario_draw_ui(struct nk_context* ctx);

camera_settings cam_settings = {
  .movement_speed = 10.0f,
  .rotation_sensitivity = 0.1f,
};

Color colors[] = { BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE,  BROWN, YELLOW, GREEN, MAROON, MAGENTA, RAYWHITE, DARKPURPLE, LIME, PINK, ORANGE  };
Material materials[20];
Mesh meshes[20];

bool edit_mode = false;
bool simulation_running = true;
bool step_forward = false;

bool show_physics_world_stats = false;
bool show_physics_config_widget = false;
bool draw_collisions = false;
bool collision_debug_mode = false;
bool draw_gismos = false;
bool draw_angular_momenta = false;
bool observe_body_mode = false;

typedef struct {
  bool has_hit;
  body_handle handle;
  body_shape shape;
  bool is_dynamic;
} observed_body_state;

static observed_body_state observed_body = {0};

static collision_debug_state debug_state;

static Model groundModel;
static physics_world *world;
static physics_config config;

int main(int argc, char** argv) {
  program_config program_config = {0};
  config = physics_default_config();

  scenario_initialize(&program_config, &config);

  InitWindow(screen_width, screen_height, program_config.window_title);
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetExitKey(KEY_F10);
  SetTargetFPS(frame_rate);
  SetTraceLogLevel(LOG_DEBUG);

  struct nk_context *ctx = InitNuklear(ui_font_size);

  Camera camera = setup_camera(program_config);
  Shader shader = setup_lighting();

  init_debugging();
  init_gizmos();
  init_physics();
  setup_scene(shader);
  scenario_setup_scene(world);

  if (argc > 1 && !strncmp(argv[1], "-p", 2)) {
    simulation_running = false;
  }

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    update_camera(&camera, GetFrameTime());
    UpdateNuklear(ctx);

    process_inputs(world, &camera);

    int sim_count = 0;
    if (!edit_mode) {
      accum += deltaTime;
      sim_count = (int)(accum / simulation_step);

      for (int i = 0; i < sim_count; i++) {
        if (!simulation_running && !step_forward) break;

        if (collision_debug_mode) {
          if (debug_state.active) {
            simulation_running = false;
            physics_step_debug(world, simulation_step, &debug_state);
          } else {
            scenario_simulate(world, simulation_step);
            physics_step_debug(world, simulation_step, &debug_state);
          }
        } else {
          scenario_simulate(world, simulation_step);
          physics_step(world, simulation_step);
        }

        step_forward = false;
      }
    } else {
      manipulate_gizmos(&camera);
    }

    draw_ui_widget_controls(ctx);
    scenario_draw_ui(ctx);

    draw_scene(camera, ctx, shader);

    accum -= sim_count * simulation_step;
    deltaTime = GetFrameTime();
  }

  print_diagnostics(world);

  physics_teardown(world);

  UnloadNuklear(ctx);
  UnloadShader(shader);
  CloseWindow();

  return 0;
}

static void process_inputs(physics_world *world, Camera* camera) {
  if (IsKeyPressed(KEY_SPACE)) {
    simulation_running = !simulation_running;
  }

  if (IsKeyPressed(KEY_PERIOD)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) {
    reset();
  }

  if (!edit_mode && IsKeyPressed(KEY_ESCAPE)) {
    edit_mode = true;
  }

  if (edit_mode && IsKeyPressed(KEY_P)) {
    edit_mode = false;
  }

  if (!edit_mode)
    scenario_handle_input(world, camera);

  if (observe_body_mode && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    Ray ray = GetScreenToWorldRay(GetMousePosition(), *camera);
    v3 origin = { ray.position.x, ray.position.y, ray.position.z };
    v3 direction = { ray.direction.x, ray.direction.y, ray.direction.z };

    raycast_hit hit;
    count_t hit_count = physics_raycast(world, origin, direction, 1000.0f, 1, &hit);
    if (hit_count > 0) {
      observed_body.has_hit = true;
      observed_body.handle = hit.body;
      observed_body.is_dynamic = (hit.body.type == BODY_DYNAMIC);
      physics_get_shape(world, hit.body, &observed_body.shape);
    } else {
      observed_body.has_hit = false;
    }
  }
}

static void draw_ui_widget_controls(struct nk_context* ctx) {
  static const char* window_name = "ui_widget_controls";

  const float row_height = 18.0f;
  const float window_width = 220.0f;
  const int checkbox_count = 7;

  float header_height = ctx->style.font->height + ctx->style.window.header.padding.y * 2.0f;
  float padding_y = ctx->style.window.padding.y;
  float spacing_y = ctx->style.window.spacing.y;
  float window_height = header_height + (padding_y * 2.0f) + (row_height * checkbox_count) + (spacing_y * (checkbox_count - 1)) + 25.0;

  if (nk_begin_titled(ctx, window_name, "", nk_rect(20, 20, window_width, window_height), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_MINIMIZABLE | NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_TITLE)) {
    if (!nk_window_is_collapsed(ctx, window_name)) {
      nk_window_set_size(ctx, window_name, nk_vec2(window_width, window_height));
      nk_layout_row_dynamic(ctx, row_height, 1);

      nk_bool physics_world_stats = show_physics_world_stats ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Physics world stats", &physics_world_stats);
      show_physics_world_stats = physics_world_stats != 0;

      nk_bool physics_config = show_physics_config_widget ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Physics config", &physics_config);
      show_physics_config_widget = physics_config != 0;

      nk_bool collisions = draw_collisions ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Draw collisions", &collisions);
      draw_collisions = collisions != 0;

      nk_bool cdbg = collision_debug_mode ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Collision debugging", &cdbg);
      if (cdbg && !collision_debug_mode)
        physics_debug_state_init(&debug_state);
      collision_debug_mode = cdbg != 0;

      nk_bool gismos = draw_gismos ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Draw gismos", &gismos);
      draw_gismos = gismos != 0;

      nk_bool angular = draw_angular_momenta ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Draw angular momenta", &angular);
      draw_angular_momenta = angular != 0;

      nk_bool observe = observe_body_mode ? nk_true : nk_false;
      nk_checkbox_label(ctx, "Observe body", &observe);
      if (!observe) observed_body.has_hit = false;
      observe_body_mode = observe != 0;
    }
  }

  nk_end(ctx);

  if (show_physics_world_stats)
    physics_draw_stats(world, ctx);
  if (show_physics_config_widget)
    physics_draw_config_widget(world, ctx);
  if (collision_debug_mode)
    physics_draw_debug_widget(world, &debug_state, ctx);
  if (observe_body_mode)
    draw_ui_widget_observe_body(ctx);
}

static void draw_ui_widget_observe_body(struct nk_context* ctx) {
  static const char* window_name = "ui_widget_observe_body";

  const float row_height = 18.0f;
  const float window_width = 260.0f;

  int row_count = 1; // "No body selected" or shape line
  if (observed_body.has_hit && observed_body.is_dynamic)
    row_count += 5; // velocity, |velocity|, angular velocity, |angular velocity|, motion avg

  float header_height = ctx->style.font->height + ctx->style.window.header.padding.y * 2.0f;
  float padding_y = ctx->style.window.padding.y;
  float spacing_y = ctx->style.window.spacing.y;
  float window_height = header_height + (padding_y * 2.0f) + (row_height * row_count) + (spacing_y * (row_count - 1)) + 10.0f;

  if (nk_begin_titled(ctx, window_name, "Observe body", nk_rect(20, 300, window_width, window_height), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_MINIMIZABLE | NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_TITLE)) {
    if (!nk_window_is_collapsed(ctx, window_name)) {
      nk_window_set_size(ctx, window_name, nk_vec2(window_width, window_height));
      nk_layout_row_dynamic(ctx, row_height, 1);

      if (!observed_body.has_hit) {
        nk_label(ctx, "No body selected", NK_TEXT_LEFT);
      } else {
        char buf[128];

        const char *shape_name;
        switch (observed_body.shape.type) {
          case SHAPE_BOX:    shape_name = "Box";    break;
          case SHAPE_SPHERE: shape_name = "Sphere"; break;
          case SHAPE_PLANE:  shape_name = "Plane";  break;
          default:           shape_name = "Unknown";
        }
        snprintf(buf, sizeof(buf), "Shape: %s", shape_name);
        nk_label(ctx, buf, NK_TEXT_LEFT);

        if (observed_body.is_dynamic) {
          v3 vel = {0}, av = {0};
          physics_get_velocity(world, observed_body.handle, &vel);
          physics_get_angular_velocity(world, observed_body.handle, &av);
          snprintf(buf, sizeof(buf), "Velocity: (%.2f, %.2f, %.2f)", vel.x, vel.y, vel.z);
          nk_label(ctx, buf, NK_TEXT_LEFT);
          snprintf(buf, sizeof(buf), "  |velocity|: %.2f", len(vel));
          nk_label(ctx, buf, NK_TEXT_LEFT);
          snprintf(buf, sizeof(buf), "Ang. vel: (%.2f, %.2f, %.2f)", av.x, av.y, av.z);
          nk_label(ctx, buf, NK_TEXT_LEFT);
          snprintf(buf, sizeof(buf), "  |ang. vel|: %.2f", len(av));
          nk_label(ctx, buf, NK_TEXT_LEFT);
          float motion_avg = 0;
          physics_get_motion_avg(world, observed_body.handle, &motion_avg);
          snprintf(buf, sizeof(buf), "Motion avg: %.4f", motion_avg);
          nk_label(ctx, buf, NK_TEXT_LEFT);
        }
      }
    }
  }

  nk_end(ctx);
}

static void print_diagnostics(physics_world *world) {
#ifdef DIAGNOSTICS
  diagnostics d = world->diagnostics;

  TraceLog(LOG_INFO, "Physics world diagnostics:");

  float p50, p75, p99;
  percentiles_query(&d.penetration_depth, &p50, &p75, &p99);
  TraceLog(LOG_INFO, "Penetration depth percentiles: p50=%f, p75=%f, p99=%f", p50, p75, p99);

  percentiles_query(&d.velocity_deltas, &p50, &p75, &p99);
  TraceLog(LOG_INFO, "Velocity percentiles: p50=%f, p75=%f, p99=%f", p50, p75, p99);

  float penetrations_percentage = (float) d.unresolved_penetrations / d.frames_simulated * 100.0;
  float velocities_percentage = (float) d.unresolved_velocities / d.frames_simulated * 100.0;
  TraceLog(LOG_INFO, "Unresolved penetrations: %.1f %", penetrations_percentage);
  TraceLog(LOG_INFO, "Unresolved velocities: %.1f %", velocities_percentage);
#else
  (void) world;
#endif
}

static void draw_physics_bodies_type(body_type type) {
  common_data *data = type == BODY_DYNAMIC ? (common_data*) &world->dynamics : &world->statics;
  size_t count = data->count;

  for (size_t i = 0; i < count; ++i) {
    m4 scale;
    count_t index = data->outer_lookup[i];
    v3 position = data->positions[index];
    quat rotation = data->rotations[index];
    body_shape shape = data->shapes[index];

    m4 transform = MatrixMultiply(QuaternionToMatrix(rotation), MatrixTranslate(position.x, position.y, position.z));
    Material material = materials[i % 20];

    switch (shape.type) {
      case SHAPE_BOX:
        scale = MatrixScale(shape.box.size.x, shape.box.size.y, shape.box.size.z);
        DrawMesh(meshes[SHAPE_BOX], material, mul(scale, transform));
        break;

      case SHAPE_SPHERE:
        scale = MatrixScale(shape.sphere.radius, shape.sphere.radius, shape.sphere.radius);
        DrawMesh(meshes[SHAPE_SPHERE], material, mul(scale, transform));
        break;

      default:
        break;
    }
  }
}

static void draw_physics_bodies() {
  draw_physics_bodies_type(BODY_DYNAMIC);
  draw_physics_bodies_type(BODY_STATIC);

  dynamic_bodies *dynamics = &world->dynamics;
  for(count_t i = 0; i < dynamics->count; ++i) {
    count_t index = dynamics->outer_lookup[i];
    v3 position = dynamics->positions[index];
    quat rotation = dynamics->rotations[index];
    v3 angular_momentum = dynamics->angular_momenta[index];

    if (draw_gismos)
      draw_body_axes(position, rotation);

    if (draw_angular_momenta)
      draw_body_angular_momentum(position, angular_momentum);
  }

  if (!debug_state.active || debug_state.prev_phase == CDBG_IDLE || debug_state.prev_phase == CDBG_DONE)
    return;

  contact c = world->collisions->contacts[debug_state.current_contact_index];
  draw_arrow(c.point, c.normal,RED);
}

static void draw_body_axes(v3 position, quat rotation) {
  const float axis_len = 1.5f;

  v3 x = rotate(((v3){axis_len, 0, 0}), rotation);
  v3 y = rotate(((v3){0, axis_len, 0}), rotation);
  v3 z = rotate(((v3){0, 0, axis_len}), rotation);

  draw_arrow(position, x, RED);
  draw_arrow(position, y, GREEN);
  draw_arrow(position, z, BLUE);
}

static void draw_body_angular_momentum(v3 position, v3 angular_momentum) {
  const float scale_factor = 1.0f;
  const float max_len = 2.0f;

  v3 v = scale(angular_momentum, scale_factor);
  float l = len(v);
  if (l < 0.0001f)
    return;

  if (l > max_len)
    v = scale(normalize(v), max_len);

  draw_arrow(position, v, SKYBLUE);
}

static void draw_scene(Camera camera, struct nk_context* ctx, Shader shader) {
  BeginDrawing();

    ClearBackground(COLOR_BACKGROUND);

      BeginMode3D(camera);

        BeginShaderMode(shader);

          if (draw_collisions)
            physics_draw_collisions(world);

          draw_physics_bodies();
          scenario_draw_scene(world);

          // Draw ground plane
          DrawModel(groundModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
          draw_custom_grid(32, 1.0f);

          if (edit_mode)
            draw_gizmos();

        EndShaderMode();

      EndMode3D();

      DrawNuklear(ctx);
      DrawFPS(1800, 1050);

  EndDrawing();
}

static void update_camera(Camera* camera, float deltaTime) {
  Vector3 forward = GetCameraForward(camera);
  Vector3 right = GetCameraRight(camera);

  Vector3 movement = {0};

  if (IsKeyDown(KEY_W)) movement = Vector3Add(movement, Vector3Scale(forward, cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_S)) movement = Vector3Add(movement, Vector3Scale(forward, -cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_A)) movement = Vector3Add(movement, Vector3Scale(right, -cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_D)) movement = Vector3Add(movement, Vector3Scale(right, cam_settings.movement_speed * deltaTime));

  camera->position = Vector3Add(camera->position, movement);
  camera->target = Vector3Add(camera->target, movement);

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    Vector2 mouseDelta = GetMouseDelta();
    Vector3 rotation = {0};
    rotation.x = -mouseDelta.x * cam_settings.rotation_sensitivity;  // Yaw
    rotation.y = -mouseDelta.y * cam_settings.rotation_sensitivity; // Pitch

    UpdateCameraPro(camera, (Vector3){0}, rotation, 0.0f);
  }
}

static Camera setup_camera(program_config config) {
  Camera3D camera = {0};
  camera.position = config.camera_position;
  camera.target = config.camera_target;
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  return camera;
}

static void draw_custom_grid(int slices, float spacing) {
  int halfSlices = slices / 2;
  Color mainColor = COLOR_GRID_MAIN;
  Color subColor = COLOR_GRID_SUB;

  for (int i = -halfSlices; i <= halfSlices; i++) {
    Color lineColor = (i % 10 == 0) ? mainColor : subColor;

    DrawLine3D(
      (Vector3){i * spacing, 0.03f, -halfSlices * spacing},
      (Vector3){i * spacing, 0.03f, halfSlices * spacing},
      lineColor
    );

    DrawLine3D(
      (Vector3){-halfSlices * spacing, 0.03f, i * spacing},
      (Vector3){halfSlices * spacing, 0.03f, i * spacing},
      lineColor
    );
  }
}

static void setup_scene(Shader shader) {
  meshes[SHAPE_BOX] = GenMeshCube(1, 1, 1);
  meshes[SHAPE_SPHERE] = GenMeshSphere(1, 16, 16);
  meshes[SHAPE_PLANE] = GenMeshPlane(200.0f, 200.0f, 1, 1);

  for (size_t i = 0; i < 20; ++i) {
    Material m = LoadMaterialDefault();
    m.shader = shader;
    m.maps[MATERIAL_MAP_ALBEDO].color = colors[i];

    materials[i] = m;
  }

  groundModel = LoadModelFromMesh(meshes[SHAPE_PLANE]);

  groundModel.materials[0].shader = shader;
  groundModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_GROUND;
}

static void reset() {
  physics_reset(world);
  physics_add_plane(world, zero(), up());
  scenario_setup_scene(world);
}

static void init_physics() {
  world = physics_init(&config);
  physics_add_plane(world, zero(), up());
}

static Shader setup_lighting() {
  char *vs_shader_path = "shaders/lighting_fog.vs";
  char *fs_shader_path = "shaders/lighting_fog.fs";

  Shader shader = LoadShader(vs_shader_path, fs_shader_path);

  Light keyLight = CreateLight(
    LIGHT_DIRECTIONAL,
    (Vector3){10.0f, 20.0f, 10.0f},
    Vector3Zero(),
    WHITE,  // #ffffff
    shader
  );
  keyLight.enabled = 1;
  UpdateLightValues(shader, keyLight);

  Light rimLight = CreateLight(
    LIGHT_POINT,
    (Vector3){-10.0f, 10.0f, -10.0f},
    Vector3Zero(),
    (Color){0x44, 0x44, 0xff, 0xff},  // Blue rim light
    shader
  );
  rimLight.enabled = 1;
  UpdateLightValues(shader, rimLight);

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0x40/255.0f, 0x40/255.0f, 0x40/255.0f, 1.0f},
                 SHADER_UNIFORM_VEC4);

  int fogColorLoc = GetShaderLocation(shader, "fogColor");
  int fogStartLoc = GetShaderLocation(shader, "fogStart");
  int fogEndLoc = GetShaderLocation(shader, "fogEnd");

  SetShaderValue(shader, fogColorLoc, (float[3]){0x12/255.0f, 0x12/255.0f, 0x14/255.0f}, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, fogStartLoc, (float[1]){20.0f}, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader, fogEndLoc, (float[1]){100.0f}, SHADER_UNIFORM_FLOAT);

  return shader;
}
