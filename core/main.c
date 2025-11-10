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

typedef struct {
  float movement_speed;
  float rotation_sensitivity;
} camera_settings;

void init_debugging();
void init_gizmos();
void manipulate_gizmos(Camera *camera);
void draw_gizmos();

static void draw_custom_grid(int slices, float spacing);
static void setup_ground_plane(Shader shader);
static Shader setup_lighting();
static Camera setup_camera(program_config config);
static void update_camera(Camera* camera, float deltaTime);
static void draw_scene(Camera camera, float accum, struct nk_context* ctx, Shader shader);
static void process_inputs(Camera* camera);

camera_settings cam_settings = {
  .movement_speed = 10.0f,
  .rotation_sensitivity = 0.1f,
};

bool edit_mode = false;
bool simulation_running = true;
bool step_forward = false;

static Model groundModel;
static bool groundInitialized = false;

int main(int argc, char** argv) {

  program_config config = {0};

  initialize_program(&config);

  InitWindow(screen_width, screen_height, config.window_title);
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetExitKey(KEY_F10);
  SetTargetFPS(frame_rate);
  SetTraceLogLevel(LOG_DEBUG);

  struct nk_context *ctx = InitNuklear(ui_font_size);

  Camera camera = setup_camera(config);
  Shader shader = setup_lighting();

  init_debugging();
  init_gizmos();
  setup_ground_plane(shader);
  setup_scene(shader);

  if (argc > 1 && !strncmp(argv[1], "-p", 2)) {
    simulation_running = false;
  }

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    update_camera(&camera, GetFrameTime());
    UpdateNuklear(ctx);

    process_inputs(&camera);

    int sim_count = 0;
    if (!edit_mode) {
      accum += deltaTime;
      sim_count = (int)(accum / simulation_step);

      for (int i = 0; i < sim_count; i++) {
        if (!simulation_running && !step_forward) break;
      
        simulate(simulation_step);

        step_forward = false;
      }
    } else {
      manipulate_gizmos(&camera);
    }

    draw_ui(ctx);
    draw_scene(camera, accum, ctx, shader);
    
    accum -= sim_count * simulation_step;
    deltaTime = GetFrameTime();
  }

  UnloadNuklear(ctx);
  UnloadShader(shader);
  CloseWindow();

  return 0;
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

  if (!edit_mode && IsKeyPressed(KEY_ESCAPE)) {
    edit_mode = true;
  }

  if (edit_mode && IsKeyDown(KEY_P)) {
    edit_mode = false;
  }

  if (!edit_mode)
    on_input(camera);
}

static void draw_scene(Camera camera, float accum, struct nk_context* ctx, Shader shader) {
  BeginDrawing();

    ClearBackground((Color){0x12, 0x12, 0x14, 0xFF}); // #121214

      BeginMode3D(camera);

        BeginShaderMode(shader);

          float t = Clamp(accum / simulation_step, 0, 1);
          draw(t);

          // Draw ground plane
          if (groundInitialized) {
            DrawModel(groundModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
          }
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
  Color mainColor = (Color){0x44, 0x44, 0x44, 0xFF};    // Main divisions (#444444)
  Color subColor = (Color){0x22, 0x22, 0x22, 0xFF};     // Sub divisions (#222222)

  for (int i = -halfSlices; i <= halfSlices; i++) {
    Color lineColor = (i % 10 == 0) ? mainColor : subColor;

    // Lines parallel to Z axis
    DrawLine3D(
      (Vector3){i * spacing, 0.01f, -halfSlices * spacing},
      (Vector3){i * spacing, 0.01f, halfSlices * spacing},
      lineColor
    );

    // Lines parallel to X axis
    DrawLine3D(
      (Vector3){-halfSlices * spacing, 0.01f, i * spacing},
      (Vector3){halfSlices * spacing, 0.01f, i * spacing},
      lineColor
    );
  }
}

static void setup_ground_plane(Shader shader) {
  // Create plane mesh (200x200 units)
  Mesh mesh = GenMeshPlane(200.0f, 200.0f, 1, 1);
  groundModel = LoadModelFromMesh(mesh);

  // Set shader
  groundModel.materials[0].shader = shader;

  // Set color to #080808 (nearly black)
  groundModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){0x08, 0x08, 0x08, 0xFF};

  groundInitialized = true;
}

static Shader setup_lighting() {
  char *vs_shader_path = "shaders/lighting_fog.vs";
  char *fs_shader_path = "shaders/lighting_fog.fs";

  Shader shader = LoadShader(vs_shader_path, fs_shader_path);

  // Key light (Directional) - Main shadow caster from upper-right
  Light keyLight = CreateLight(
    LIGHT_DIRECTIONAL,
    (Vector3){10.0f, 20.0f, 10.0f},
    Vector3Zero(),
    WHITE,  // #ffffff
    shader
  );
  keyLight.enabled = 1;
  UpdateLightValues(shader, keyLight);

  // Rim light (Point) - Blue highlight from back-left
  Light rimLight = CreateLight(
    LIGHT_POINT,
    (Vector3){-10.0f, 10.0f, -10.0f},
    Vector3Zero(),
    (Color){0x44, 0x44, 0xff, 0xff},  // #4444ff
    shader
  );
  rimLight.enabled = 1;
  UpdateLightValues(shader, rimLight);

  // Ambient light - #404040
  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc, (float[4]){0x40/255.0f, 0x40/255.0f, 0x40/255.0f, 1.0f},
                 SHADER_UNIFORM_VEC4);

  // Set fog parameters
  int fogColorLoc = GetShaderLocation(shader, "fogColor");
  int fogStartLoc = GetShaderLocation(shader, "fogStart");
  int fogEndLoc = GetShaderLocation(shader, "fogEnd");

  SetShaderValue(shader, fogColorLoc, (float[3]){0x12/255.0f, 0x12/255.0f, 0x14/255.0f}, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, fogStartLoc, (float[1]){20.0f}, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader, fogEndLoc, (float[1]){100.0f}, SHADER_UNIFORM_FLOAT);

  return shader;
}
