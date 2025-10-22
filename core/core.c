#include "core.h"
#include "raylib.h"
#include "raymath.h"

Mesh arrow_base;
Mesh arrow_head;
Material mat;

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
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_value_float(ctx, title, value);
  nk_layout_row_end(ctx);
}

void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s: (%.3f, %.3f, %.3f)", title, value.x, value.y, value.z);
  nk_layout_row_end(ctx);
}

void draw_stat_matrix(struct nk_context* ctx, char* title, Matrix value) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
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


void draw_property_float(struct nk_context* ctx, char* title, float* value, float min, float max, float step_arrow, float step_drag) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9);
  nk_property_float(ctx, title, min, value, max, step_arrow, step_drag);
  nk_layout_row_end(ctx);
}
