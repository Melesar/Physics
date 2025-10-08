#include "raylib.h"
#include "core.h"
#include "raymath.h"
#include "stdio.h"
#include "raylib-nuklear.h"

const int ui_font_size = 12;
const int screenWidth = 1920;
const int screenHeight = 1080;
const int frameRate = 60;
const int simulationRate = 30;

const float simulationStep = 1.0 / simulationRate;

void draw_scene(Camera camera, float accum, struct nk_context* ctx) {
  BeginDrawing();
    ClearBackground(RAYWHITE);

      BeginMode3D(camera);

        float t = Clamp(accum / simulationStep, 0, 1);
        draw(t);

        DrawPlane((Vector3){0.0f, 0.0f, 0.0f}, (Vector2){32.0f, 32.0f}, LIGHTGRAY);
        DrawGrid(32, 1.0f);

      EndMode3D();

      DrawNuklear(ctx);
  EndDrawing();
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

  setup_scene();

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    UpdateCamera(&camera, config.camera_mode);
    UpdateNuklear(ctx);

    accum += deltaTime;
    int sim_count = (int)(accum / simulationStep);

    for (int i = 0; i < sim_count; i++) {
      save_state();
      simulate(simulationStep);
    }

    draw_ui(ctx);
    draw_scene(camera, accum, ctx);

    accum -= sim_count * simulationStep;
    deltaTime = GetFrameTime();
  }

  UnloadNuklear(ctx);
  CloseWindow();

  return 0;
}
