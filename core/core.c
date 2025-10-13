#include "core.h"
#include "raylib.h"

void draw_arrow(Vector3 start, Vector3 end, Color color) {
  const float radius = 0.02f;

  DrawSphere(end, radius * 2, color);
  DrawCapsule(start, end, radius, 32, 32, color);
}
