#include "raylib.h"
#include "physics.h"
#include "core.h"
#include "math.h"
#include "string.h"
#include "raylib-nuklear.h"

struct object {
  Mesh mesh;
  Material material;
};

const int num_bodies = 3;

const float stiffness = 2;
const float initial_offset = 5;

struct object graphics[num_bodies];
rigidbody prev_state[num_bodies];
rigidbody masses[num_bodies];

float total_time;
bool interpolate = true;

void initialize_program(program_config* config) {
  config->camera_mode = CAMERA_CUSTOM;
  config->window_title = "Mass on a spring";
  config->camera_position = (Vector3) { 2, 7, 10 };
  config->camera_target = (Vector3) { 2, 0.5, 0 };
}

void setup_scene() {
  Mesh cubeMesh = GenMeshCube(1, 1, 1);

  Color colors[num_bodies] = { YELLOW, GREEN, BLUE }; 
  float offsets[num_bodies] = { -2, 0, 2 };

  for (int i = 0; i < num_bodies; ++i) {
    Material m = LoadMaterialDefault();
    m.maps[MATERIAL_MAP_DIFFUSE].color = colors[i];

    graphics[i] = (struct object) { cubeMesh, m };
    masses[i] = rb_new((Vector3){ initial_offset, 0.5, offsets[i] }, 1);
  }

  save_state();
}

void save_state() {
  memcpy(prev_state, masses, sizeof(masses));
}

void simulate(float dt) {
  rigidbody* exact = &masses[0];
  float x = initial_offset * cosf(sqrtf(stiffness / exact->mass) * total_time);
  exact->position.x = x;

  total_time += dt;
}

void draw(float interpolation) {
  for (int i = 0; i < num_bodies; ++i) {
    rigidbody body;
    if (interpolate) {
      const rigidbody* current = &masses[i];
      const rigidbody* prev = &prev_state[i];

      body = rb_interpolate(current, prev, interpolation);
    } else {
      body = masses[i];
    }

    struct object obj = graphics[i];
    
    DrawMesh(obj.mesh, obj.material, rb_transformation(&body));
  }
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin(ctx, "Debug", nk_rect(50, 50, 220, 220), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_static(ctx, 30, 80, 1);
    nk_radio_label(ctx, "Interpolation", &interpolate);
 
    // fixed widget window ratio width
    // nk_layout_row_dynamic(ctx, 30, 2);
    // if (nk_option_label(ctx, "easy", mode == 1)) mode = 1;
    // if (nk_option_label(ctx, "hard", mode == 2)) mode = 2;
 
    // custom widget pixel width
    // nk_layout_row_begin(ctx, NK_STATIC, 30, 2);
    // {
    //     nk_layout_row_push(ctx, 50);
    //     nk_label(ctx, "Volume:", NK_TEXT_LEFT);
    //     nk_layout_row_push(ctx, 110);
    //     nk_slider_float(ctx, 0, &value, 1.0f, 0.1f);
    // }
    // nk_layout_row_end(ctx);
  }

  nk_end(ctx);
}
