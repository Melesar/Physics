#include "raylib.h"
#include "core.h"
#include "raymath.h"

#define RLIGHTS_IMPLEMENTATION
#include "shaders/rlights.h"

#include "raylib-nuklear.h"

const int ui_font_size = 12;
const int screenWidth = 1920;
const int screenHeight = 1080;
const int frameRate = 60;
const int simulationRate = 120;

const float simulationStep = 1.0 / simulationRate;

bool simulation_running = true;
bool step_forward = false;

void draw_scene(Camera camera, float accum, struct nk_context* ctx, Shader shader) {
  BeginDrawing();
    ClearBackground(RAYWHITE);
      BeginMode3D(camera);

        BeginShaderMode(shader);

          float t = Clamp(accum / simulationStep, 0, 1);
          draw(t);

          DrawPlane((Vector3){0.0f, 0.0f, 0.0f}, (Vector2){32.0f, 32.0f}, LIGHTGRAY);
          DrawGrid(32, 1.0f);

        EndShaderMode();

      EndMode3D();

      DrawNuklear(ctx);
  EndDrawing();
}

void process_inputs() {
  if (IsKeyPressed(KEY_SPACE)) {
    simulation_running = !simulation_running;
  }

  if (IsKeyDown(KEY_S)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) {
    reset();
  }

  on_input();
}

int main(void) {
  program_config config = {0};

  initialize_program(&config);

  InitWindow(screenWidth, screenHeight, config.window_title);
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetExitKey(KEY_F10);
  SetTargetFPS(frameRate); // Set frame rate
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

  Shader shader = LoadShader(vs_shader_path, fs_shader_path); // Use default lighting shader
  Light light = CreateLight(LIGHT_DIRECTIONAL, (Vector3){-2.0f, 5.0f, -2.0f}, Vector3Zero(), WHITE, shader);

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0.4f, 0.4f, 0.4f, 4.0f},
                 SHADER_UNIFORM_VEC4);

  setup_scene(shader);

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    UpdateCamera(&camera, config.camera_mode);
    UpdateNuklear(ctx);
    UpdateLightValues(shader, light);

    process_inputs();

    accum += deltaTime;
    int sim_count = (int)(accum / simulationStep);

    for (int i = 0; i < sim_count; i++) {
      if (!simulation_running && !step_forward) break;
      
      save_state();
      simulate(simulationStep);

      step_forward = false;
    }

    draw_ui(ctx);
    draw_scene(camera, accum, ctx, shader);
    accum -= sim_count * simulationStep;
    deltaTime = GetFrameTime();
  }

  UnloadNuklear(ctx);
  UnloadShader(shader);
  CloseWindow();

  return 0;
}
