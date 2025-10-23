#include "raylib.h"
#include "core.h"
#include "raymath.h"
#include "string.h"

#define RLIGHTS_IMPLEMENTATION
#include "shaders/rlights.h"

#include "rcamera.h"

#include "raylib-nuklear.h"

const int ui_font_size = 14;
const int screen_width = 1920;
const int screen_height = 1080;
const int frame_rate = 60;
const int simulation_rate = 120;

const float simulation_step = 1.0 / simulation_rate;

bool simulation_running = true;
bool step_forward = false;

// Camera settings
typedef struct {
  float movement_speed;
  float rotation_sensitivity;
} camera_settings;

camera_settings cam_settings = {
  .movement_speed = 10.0f,
  .rotation_sensitivity = 0.1f,
};

void init_debugging();

static void update_custom_camera(Camera* camera, float deltaTime) {
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
    rotation.x = mouseDelta.x * cam_settings.rotation_sensitivity;  // Yaw
    rotation.y = -mouseDelta.y * cam_settings.rotation_sensitivity; // Pitch

    UpdateCameraPro(camera, (Vector3){0}, rotation, 0.0f);
  }
}

static void draw_camera_ui(struct nk_context* ctx) {
  // Position window in top-right corner
  int window_width = 280;
  int window_x = screen_width - window_width - 20;

  if (nk_begin_titled(ctx, "camera_settings", "Camera Settings",
                      nk_rect(window_x, 20, window_width, 150),
                      NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_dynamic(ctx, 30, 1);
    nk_label(ctx, "Movement & Rotation", NK_TEXT_ALIGN_CENTERED);

    nk_layout_row_dynamic(ctx, 25, 1);
    nk_property_float(ctx, "Speed:", 1.0f, &cam_settings.movement_speed, 50.0f, 1.0f, 0.5f);
    // nk_property_float(ctx, "Sensitivity:", 0.001f, &cam_settings.rotation_sensitivity, 0.1f, 0.005f, 0.001f);
  }
  nk_end(ctx);
}

static void draw_scene(Camera camera, float accum, struct nk_context* ctx, Shader shader) {
  BeginDrawing();

    ClearBackground(RAYWHITE);

      BeginMode3D(camera);

        BeginShaderMode(shader);

          float t = Clamp(accum / simulation_step, 0, 1);
          draw(t);

          DrawPlane((Vector3){0.0f, 0.0f, 0.0f}, (Vector2){32.0f, 32.0f}, LIGHTGRAY);
          DrawGrid(32, 1.0f);

        EndShaderMode();

      EndMode3D();

      DrawNuklear(ctx);
      DrawFPS(1800, 1050);

  EndDrawing();
}

static void process_inputs(Camera* camera) {
  if (IsKeyPressed(KEY_SPACE)) {
    simulation_running = !simulation_running;
  }

  if (IsKeyDown(KEY_PERIOD)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) {
    reset();
  }

  on_input(camera);
}

int main(int argc, char** argv) {

  program_config config = {0};

  initialize_program(&config);

  InitWindow(screen_width, screen_height, config.window_title);
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetExitKey(KEY_F10);
  SetTargetFPS(frame_rate); // Set frame rate
  SetTraceLogLevel(LOG_DEBUG);


  struct nk_context *ctx = InitNuklear(ui_font_size);

  Camera3D camera = {0};
  camera.position = config.camera_position;
  camera.target = config.camera_target;
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  char *vs_shader_path = "vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.vs";
  char *fs_shader_path = "vendor/raylib/examples/shaders/resources/shaders/glsl330/lighting.fs";

  Shader shader = LoadShader(vs_shader_path, fs_shader_path);
  Light light = CreateLight(LIGHT_DIRECTIONAL, (Vector3){-2.0f, 5.0f, -2.0f}, Vector3Zero(), WHITE, shader);

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0.4f, 0.4f, 0.4f, 4.0f},
                 SHADER_UNIFORM_VEC4);

  init_debugging();
  setup_scene(shader);

  if (argc > 1 && !strncmp(argv[1], "-p", 2)) {
    simulation_running = false;
    TraceLog(LOG_INFO, "Simulation paused...");
  }

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    update_custom_camera(&camera, GetFrameTime());
    UpdateNuklear(ctx);
    UpdateLightValues(shader, light);

    process_inputs(&camera);

    accum += deltaTime;
    int sim_count = (int)(accum / simulation_step);

    for (int i = 0; i < sim_count; i++) {
      if (!simulation_running && !step_forward) break;
      
      save_state();
      simulate(simulation_step);

      step_forward = false;
    }

    draw_ui(ctx);
    draw_camera_ui(ctx);
    draw_scene(camera, accum, ctx, shader);
    
    accum -= sim_count * simulation_step;
    deltaTime = GetFrameTime();
  }

  UnloadNuklear(ctx);
  UnloadShader(shader);
  CloseWindow();

  return 0;
}
