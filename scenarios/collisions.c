#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

// Cylinder parameters
const float cylinder_mass = 3;
const cylinder cylinder_shape = { .height = 2.5, .radius = 1 };

rigidbody cylinder_body;
struct object cylinder_graphics;
Mesh cylinder_mesh;

const Vector3 initial_position = { 0, 5, 0 };
const Vector3 mesh_origin_offset = { 0, -0.5 * cylinder_shape.height, 0 };

// Ground plane
const Vector3 plane_point = { 0, 0, 0 };
const Vector3 plane_normal = { 0, 1, 0 };

// Gizmo parameters
const float gizmo_arrow_length = 2.5f;
const float gizmo_arrow_radius = 0.15f;
const float gizmo_rotation_ring_radius = 2.0f;
const float gizmo_selection_threshold = 0.3f;

// Mouse interaction state
typedef enum {
  GIZMO_NONE,
  GIZMO_TRANSLATE_X,
  GIZMO_TRANSLATE_Y,
  GIZMO_TRANSLATE_Z,
  GIZMO_ROTATE_X,
  GIZMO_ROTATE_Y,
  GIZMO_ROTATE_Z
} GizmoType;

GizmoType selected_gizmo = GIZMO_NONE;
GizmoType hovered_gizmo = GIZMO_NONE;
bool is_dragging = false;
Vector3 drag_start_pos;
Vector3 cylinder_start_pos;
Quaternion cylinder_start_rot;
float drag_start_angle;
float drag_start_offset;

// Collision state
bool is_colliding = false;

static struct object generate_cylinder_graphics(Shader shader) {
  cylinder_mesh = GenMeshCylinder(cylinder_shape.radius, cylinder_shape.height, 32);
  Vector3 *vertices = (Vector3*)cylinder_mesh.vertices;
  for (int i = 0; i < cylinder_mesh.vertexCount; ++i) {
    vertices[i] = Vector3Add(vertices[i], mesh_origin_offset);
  }
  UpdateMeshBuffer(cylinder_mesh, RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION, vertices, cylinder_mesh.vertexCount * sizeof(Vector3), 0);

  Material m = LoadMaterialDefault();
  m.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
  m.shader = shader;

  return (struct object) {
    .mesh = cylinder_mesh,
    .material = m,
    .label = "Cylinder"
  };
}

void draw_rotation_gizmo(Vector3 center, Vector3 axis, Color color, float thickness) {
  const int segments = 64;
  Vector3 perpendicular;

  // Find a perpendicular vector to the axis
  if (fabsf(axis.y) < 0.9f) {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){0, 1, 0}));
  } else {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){1, 0, 0}));
  }

  Vector3 prev_point = Vector3Add(center, Vector3Scale(perpendicular, gizmo_rotation_ring_radius));

  for (int i = 1; i <= segments; i++) {
    float angle = (float)i / segments * 2 * PI;

    // Rotate perpendicular around axis
    Quaternion rot = QuaternionFromAxisAngle(axis, angle);
    Matrix rot_mat = QuaternionToMatrix(rot);
    Vector3 rotated = Vector3Transform(perpendicular, rot_mat);
    Vector3 point = Vector3Add(center, Vector3Scale(rotated, gizmo_rotation_ring_radius));

    DrawLine3D(prev_point, point, color);
    prev_point = point;
  }
}

float ray_to_line_distance(Ray ray, Vector3 line_start, Vector3 line_end, Vector3 *closest_point_on_ray) {
  Vector3 line_dir = Vector3Normalize(Vector3Subtract(line_end, line_start));
  Vector3 ray_dir = Vector3Normalize(ray.direction);

  Vector3 w0 = Vector3Subtract(ray.position, line_start);

  float a = Vector3DotProduct(ray_dir, ray_dir);
  float b = Vector3DotProduct(ray_dir, line_dir);
  float c = Vector3DotProduct(line_dir, line_dir);
  float d = Vector3DotProduct(ray_dir, w0);
  float e = Vector3DotProduct(line_dir, w0);

  float denom = a * c - b * b;
  float t_ray = (b * e - c * d) / denom;

  if (closest_point_on_ray != NULL) {
    *closest_point_on_ray = Vector3Add(ray.position, Vector3Scale(ray_dir, t_ray));
  }

  float t_line = (a * e - b * d) / denom;
  t_line = fmaxf(0.0f, fminf(1.0f, t_line));

  Vector3 point_on_ray = Vector3Add(ray.position, Vector3Scale(ray_dir, t_ray));
  Vector3 point_on_line = Vector3Add(line_start, Vector3Scale(line_dir, t_line * Vector3Length(Vector3Subtract(line_end, line_start))));

  return Vector3Distance(point_on_ray, point_on_line);
}

float ray_to_circle_distance(Ray ray, Vector3 center, Vector3 axis, float radius, Vector3 *intersection_point) {
  // Project ray onto the plane of the circle
  float denom = Vector3DotProduct(ray.direction, axis);

  if (fabsf(denom) < 0.001f) {
    return 1000.0f; // Ray parallel to plane
  }

  float t = Vector3DotProduct(Vector3Subtract(center, ray.position), axis) / denom;

  if (t < 0) {
    return 1000.0f; // Plane behind ray
  }

  Vector3 plane_intersection = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
  Vector3 to_center = Vector3Subtract(plane_intersection, center);
  float dist_to_center = Vector3Length(to_center);

  if (intersection_point != NULL) {
    *intersection_point = plane_intersection;
  }

  return fabsf(dist_to_center - radius);
}

GizmoType check_gizmo_hover(Ray mouse_ray, Vector3 cylinder_pos) {
  float min_dist = gizmo_selection_threshold;
  GizmoType result = GIZMO_NONE;

  // Check translation gizmos
  Vector3 x_end = Vector3Add(cylinder_pos, (Vector3){gizmo_arrow_length, 0, 0});
  float dist_x = ray_to_line_distance(mouse_ray, cylinder_pos, x_end, NULL);
  if (dist_x < min_dist) {
    min_dist = dist_x;
    result = GIZMO_TRANSLATE_X;
  }

  Vector3 y_end = Vector3Add(cylinder_pos, (Vector3){0, gizmo_arrow_length, 0});
  float dist_y = ray_to_line_distance(mouse_ray, cylinder_pos, y_end, NULL);
  if (dist_y < min_dist) {
    min_dist = dist_y;
    result = GIZMO_TRANSLATE_Y;
  }

  Vector3 z_end = Vector3Add(cylinder_pos, (Vector3){0, 0, gizmo_arrow_length});
  float dist_z = ray_to_line_distance(mouse_ray, cylinder_pos, z_end, NULL);
  if (dist_z < min_dist) {
    min_dist = dist_z;
    result = GIZMO_TRANSLATE_Z;
  }

  // Check rotation gizmos
  float dist_rx = ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){1, 0, 0}, gizmo_rotation_ring_radius, NULL);
  if (dist_rx < min_dist) {
    min_dist = dist_rx;
    result = GIZMO_ROTATE_X;
  }

  float dist_ry = ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){0, 1, 0}, gizmo_rotation_ring_radius, NULL);
  if (dist_ry < min_dist) {
    min_dist = dist_ry;
    result = GIZMO_ROTATE_Y;
  }

  float dist_rz = ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){0, 0, 1}, gizmo_rotation_ring_radius, NULL);
  if (dist_rz < min_dist) {
    min_dist = dist_rz;
    result = GIZMO_ROTATE_Z;
  }

  return result;
}

void initialize_program(program_config* config) {
  config->window_title = "Collision Testing";
  config->camera_mode = CAMERA_CUSTOM;
  config->camera_position = (Vector3) { 15, 10, 15 };
  config->camera_target = (Vector3) { 0, 2, 0 };
}

void setup_scene(Shader shader) {
  cylinder_body = rb_new(initial_position, cylinder_mass);
  cylinder_body.r = QuaternionIdentity();

  cylinder_graphics = generate_cylinder_graphics(shader);
}

void reset() {
  cylinder_body.p = initial_position;
  cylinder_body.r = QuaternionIdentity();
  cylinder_body.v = Vector3Zero();
  cylinder_body.l = Vector3Zero();
  is_dragging = false;
  selected_gizmo = GIZMO_NONE;
}

void on_input(Camera *camera) {
  Ray mouse_ray = GetScreenToWorldRay(GetMousePosition(), *camera);

  if (!is_dragging) {
    hovered_gizmo = check_gizmo_hover(mouse_ray, cylinder_body.p);

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && hovered_gizmo != GIZMO_NONE) {
      is_dragging = true;
      selected_gizmo = hovered_gizmo;
      drag_start_pos = mouse_ray.position;
      cylinder_start_pos = cylinder_body.p;
      cylinder_start_rot = cylinder_body.r;

      // For translation, calculate initial offset along axis
      if (selected_gizmo >= GIZMO_TRANSLATE_X && selected_gizmo <= GIZMO_TRANSLATE_Z) {
        Vector3 axis = {0, 0, 0};
        if (selected_gizmo == GIZMO_TRANSLATE_X) axis = (Vector3){1, 0, 0};
        else if (selected_gizmo == GIZMO_TRANSLATE_Y) axis = (Vector3){0, 1, 0};
        else if (selected_gizmo == GIZMO_TRANSLATE_Z) axis = (Vector3){0, 0, 1};

        Vector3 line_start = cylinder_body.p;
        Vector3 line_end = Vector3Add(cylinder_body.p, Vector3Scale(axis, 100));

        Vector3 closest_point;
        ray_to_line_distance(mouse_ray, line_start, line_end, &closest_point);

        Vector3 offset = Vector3Subtract(closest_point, cylinder_body.p);
        drag_start_offset = Vector3DotProduct(offset, axis);
      }

      // For rotation, calculate initial angle
      if (selected_gizmo >= GIZMO_ROTATE_X) {
        Vector3 axis = {0, 0, 0};
        if (selected_gizmo == GIZMO_ROTATE_X) axis = (Vector3){1, 0, 0};
        else if (selected_gizmo == GIZMO_ROTATE_Y) axis = (Vector3){0, 1, 0};
        else if (selected_gizmo == GIZMO_ROTATE_Z) axis = (Vector3){0, 0, 1};

        Vector3 intersection;
        ray_to_circle_distance(mouse_ray, cylinder_body.p, axis, gizmo_rotation_ring_radius, &intersection);
        Vector3 to_intersection = Vector3Subtract(intersection, cylinder_body.p);

        // Project to find perpendicular for angle calculation
        Vector3 perpendicular;
        if (fabsf(axis.y) < 0.9f) {
          perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){0, 1, 0}));
        } else {
          perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){1, 0, 0}));
        }
        Vector3 binormal = Vector3CrossProduct(axis, perpendicular);

        float x = Vector3DotProduct(to_intersection, perpendicular);
        float y = Vector3DotProduct(to_intersection, binormal);
        drag_start_angle = atan2f(y, x);
      }
    }
  }

  if (is_dragging) {
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
      if (selected_gizmo == GIZMO_TRANSLATE_X || selected_gizmo == GIZMO_TRANSLATE_Y || selected_gizmo == GIZMO_TRANSLATE_Z) {
        // Translation
        Vector3 axis = {0, 0, 0};
        if (selected_gizmo == GIZMO_TRANSLATE_X) axis = (Vector3){1, 0, 0};
        else if (selected_gizmo == GIZMO_TRANSLATE_Y) axis = (Vector3){0, 1, 0};
        else if (selected_gizmo == GIZMO_TRANSLATE_Z) axis = (Vector3){0, 0, 1};

        // Project mouse movement onto axis
        Vector3 line_start = cylinder_start_pos;
        Vector3 line_end = Vector3Add(cylinder_start_pos, Vector3Scale(axis, 100));

        Vector3 closest_point;
        ray_to_line_distance(mouse_ray, line_start, line_end, &closest_point);

        Vector3 offset = Vector3Subtract(closest_point, cylinder_start_pos);
        float current_offset = Vector3DotProduct(offset, axis);

        // Calculate delta from initial click position
        float delta = current_offset - drag_start_offset;

        cylinder_body.p = Vector3Add(cylinder_start_pos, Vector3Scale(axis, delta));
      } else {
        // Rotation
        Vector3 axis = {0, 0, 0};
        if (selected_gizmo == GIZMO_ROTATE_X) axis = (Vector3){1, 0, 0};
        else if (selected_gizmo == GIZMO_ROTATE_Y) axis = (Vector3){0, 1, 0};
        else if (selected_gizmo == GIZMO_ROTATE_Z) axis = (Vector3){0, 0, 1};

        Vector3 intersection;
        float dist = ray_to_circle_distance(mouse_ray, cylinder_body.p, axis, gizmo_rotation_ring_radius, &intersection);

        if (dist < 1.0f) {
          Vector3 to_intersection = Vector3Subtract(intersection, cylinder_body.p);

          Vector3 perpendicular;
          if (fabsf(axis.y) < 0.9f) {
            perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){0, 1, 0}));
          } else {
            perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){1, 0, 0}));
          }
          Vector3 binormal = Vector3CrossProduct(axis, perpendicular);

          float x = Vector3DotProduct(to_intersection, perpendicular);
          float y = Vector3DotProduct(to_intersection, binormal);
          float current_angle = atan2f(y, x);

          float angle_delta = current_angle - drag_start_angle;

          Quaternion rotation = QuaternionFromAxisAngle(axis, angle_delta);
          cylinder_body.r = QuaternionMultiply(rotation, cylinder_start_rot);
        }
      }
    } else {
      is_dragging = false;
      selected_gizmo = GIZMO_NONE;
    }
  }
}

void simulate(float dt) {
  // Check collision with ground plane
  collision col = check_collision_cylinder_plane(cylinder_shape, &cylinder_body, plane_point, plane_normal);
  is_colliding = col.valid;

  // Update cylinder color based on collision
  if (is_colliding) {
    cylinder_graphics.material.maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
  } else {
    cylinder_graphics.material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
  }
}

void draw(float interpolation) {
  // Draw ground plane
  DrawPlane((Vector3){0, 0, 0}, (Vector2){20, 20}, LIGHTGRAY);
  DrawGrid(20, 1.0f);

  // Draw cylinder
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, rb_transformation(&cylinder_body));

  // Draw translation gizmos
  Color x_color = (hovered_gizmo == GIZMO_TRANSLATE_X || selected_gizmo == GIZMO_TRANSLATE_X) ? ORANGE : RED;
  Color y_color = (hovered_gizmo == GIZMO_TRANSLATE_Y || selected_gizmo == GIZMO_TRANSLATE_Y) ? ORANGE : GREEN;
  Color z_color = (hovered_gizmo == GIZMO_TRANSLATE_Z || selected_gizmo == GIZMO_TRANSLATE_Z) ? ORANGE : BLUE;

  draw_arrow(cylinder_body.p, (Vector3){gizmo_arrow_length, 0, 0}, x_color);
  draw_arrow(cylinder_body.p, (Vector3){0, gizmo_arrow_length, 0}, y_color);
  draw_arrow(cylinder_body.p, (Vector3){0, 0, gizmo_arrow_length}, z_color);

  // Draw rotation gizmos
  Color rx_color = (hovered_gizmo == GIZMO_ROTATE_X || selected_gizmo == GIZMO_ROTATE_X) ? ORANGE : RED;
  Color ry_color = (hovered_gizmo == GIZMO_ROTATE_Y || selected_gizmo == GIZMO_ROTATE_Y) ? ORANGE : GREEN;
  Color rz_color = (hovered_gizmo == GIZMO_ROTATE_Z || selected_gizmo == GIZMO_ROTATE_Z) ? ORANGE : BLUE;

  draw_rotation_gizmo(cylinder_body.p, (Vector3){1, 0, 0}, rx_color, 2.0f);
  draw_rotation_gizmo(cylinder_body.p, (Vector3){0, 1, 0}, ry_color, 2.0f);
  draw_rotation_gizmo(cylinder_body.p, (Vector3){0, 0, 1}, rz_color, 2.0f);
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin_titled(ctx, "debug", "Collision Testing", nk_rect(50, 50, 350, 300), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_label(ctx, cylinder_graphics.label, NK_TEXT_ALIGN_LEFT);

    draw_stat_float3(ctx, "Position", cylinder_body.p);

    nk_layout_row_static(ctx, 30, 300, 1);
    if (is_colliding) {
      nk_label(ctx, "Collision: DETECTED", NK_TEXT_ALIGN_LEFT);
    } else {
      nk_label(ctx, "Collision: NONE", NK_TEXT_ALIGN_LEFT);
    }

    const char* gizmo_names[] = {
      "None", "Translate X", "Translate Y", "Translate Z",
      "Rotate X", "Rotate Y", "Rotate Z"
    };

    if (hovered_gizmo != GIZMO_NONE) {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "Hovered: %s", gizmo_names[hovered_gizmo]);
      nk_label(ctx, buffer, NK_TEXT_ALIGN_LEFT);
    }

    if (is_dragging) {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "Dragging: %s", gizmo_names[selected_gizmo]);
      nk_label(ctx, buffer, NK_TEXT_ALIGN_LEFT);
    }
  }

  nk_end(ctx);
}

void save_state() { }
