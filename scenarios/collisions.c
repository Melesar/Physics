#include "config.h"
#include "core.h"
#include "physics.h"
#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

const cylinder cylinder_shape = { .height = 2.5, .radius = 1 };
const float cylinder_mass = 3;

Vector3 cylinder_position = { 0, 3, 0 };
Quaternion cylinder_rotation;
struct object cylinder_graphics;
Mesh cylinder_mesh;

const Vector3 mesh_origin_offset = { 0, -0.5 * 2.5, 0 };
const Vector3 ground_plane_point = { 0, 0, 0 };
const Vector3 ground_plane_normal = { 0, 1, 0 };

bool is_colliding = false;

typedef enum {
  GIZMO_NONE = 0,
  GIZMO_TRANSLATE_X,
  GIZMO_TRANSLATE_Y,
  GIZMO_TRANSLATE_Z,
  GIZMO_ROTATE_X,
  GIZMO_ROTATE_Y,
  GIZMO_ROTATE_Z
} gizmo_type;

gizmo_type active_gizmo = GIZMO_NONE;
gizmo_type hovered_gizmo = GIZMO_NONE;
Vector3 drag_start_pos;
Vector3 drag_start_cylinder_pos;
Quaternion drag_start_rotation;
float drag_start_angle;

float debug_distances[6] = {0};
Vector3 debug_ray_pos;
Vector3 debug_ray_dir;

const float gizmo_arrow_length = 2.0f;
const float gizmo_arrow_thickness = 0.1f;
const float gizmo_rotate_radius = 2.5f;
const float gizmo_hover_distance = 0.5f;

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

static Matrix cylinder_transform() {
  Matrix translation = MatrixTranslate(cylinder_position.x, cylinder_position.y, cylinder_position.z);
  Matrix rotation = QuaternionToMatrix(cylinder_rotation);
  return MatrixMultiply(rotation, translation);
}

static Vector3 get_axis_vector(gizmo_type type) {
  switch (type) {
    case GIZMO_TRANSLATE_X:
    case GIZMO_ROTATE_X:
      return (Vector3){ 1, 0, 0 };
    case GIZMO_TRANSLATE_Y:
    case GIZMO_ROTATE_Y:
      return (Vector3){ 0, 1, 0 };
    case GIZMO_TRANSLATE_Z:
    case GIZMO_ROTATE_Z:
      return (Vector3){ 0, 0, 1 };
    default:
      return Vector3Zero();
  }
}

static Color get_gizmo_color(gizmo_type type) {
  Color base_color;
  switch (type) {
    case GIZMO_TRANSLATE_X:
    case GIZMO_ROTATE_X:
      base_color = RED;
      break;
    case GIZMO_TRANSLATE_Y:
    case GIZMO_ROTATE_Y:
      base_color = GREEN;
      break;
    case GIZMO_TRANSLATE_Z:
    case GIZMO_ROTATE_Z:
      base_color = BLUE;
      break;
    default:
      base_color = WHITE;
  }
  
  if (hovered_gizmo == type || active_gizmo == type) {
    return YELLOW;
  }
  return base_color;
}

static void draw_rotation_gizmo(Vector3 center, Vector3 axis, Color color, float radius) {
  const int segments = 64;
  const int line_thickness = 3;
  Vector3 perpendicular;
  
  if (fabs(axis.y) < 0.9f) {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){0, 1, 0}));
  } else {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){1, 0, 0}));
  }
  
  Vector3 second_perp = Vector3CrossProduct(axis, perpendicular);
  
  for (int thickness = 0; thickness < line_thickness; thickness++) {
    float offset = (thickness - line_thickness / 2.0f) * 0.02f;
    float current_radius = radius + offset;
    
    for (int i = 0; i < segments; i++) {
      float angle1 = (float)i / segments * 2.0f * PI;
      float angle2 = (float)(i + 1) / segments * 2.0f * PI;
      
      Vector3 p1 = Vector3Add(center, 
        Vector3Add(
          Vector3Scale(perpendicular, cosf(angle1) * current_radius),
          Vector3Scale(second_perp, sinf(angle1) * current_radius)
        )
      );
      
      Vector3 p2 = Vector3Add(center,
        Vector3Add(
          Vector3Scale(perpendicular, cosf(angle2) * current_radius),
          Vector3Scale(second_perp, sinf(angle2) * current_radius)
        )
      );
      
      DrawLine3D(p1, p2, color);
    }
  }
}

static float ray_to_line_distance(Ray ray, Vector3 line_start, Vector3 line_end, Vector3* closest_on_line) {
  Vector3 line_vec = Vector3Subtract(line_end, line_start);
  float line_length = Vector3Length(line_vec);
  if (line_length < 0.0001f) {
    return 1000.0f;
  }
  Vector3 line_dir = Vector3Scale(line_vec, 1.0f / line_length);
  
  Vector3 w0 = Vector3Subtract(ray.position, line_start);
  
  float a = Vector3DotProduct(ray.direction, ray.direction);
  float b = Vector3DotProduct(ray.direction, line_dir);
  float c = Vector3DotProduct(line_dir, line_dir);
  float d = Vector3DotProduct(ray.direction, w0);
  float e = Vector3DotProduct(line_dir, w0);
  
  float denom = a * c - b * b;
  
  float sc, tc;
  if (fabsf(denom) < 0.0001f) {
    sc = 0.0f;
    tc = e / c;
  } else {
    sc = (b * e - c * d) / denom;
    tc = (a * e - b * d) / denom;
  }
  
  sc = fmaxf(0.0f, sc);
  tc = fmaxf(0.0f, fminf(line_length, tc));
  
  Vector3 point_on_ray = Vector3Add(ray.position, Vector3Scale(ray.direction, sc));
  Vector3 point_on_line = Vector3Add(line_start, Vector3Scale(line_dir, tc));
  
  if (closest_on_line) {
    *closest_on_line = point_on_line;
  }
  
  return Vector3Distance(point_on_ray, point_on_line);
}

static float ray_to_circle_distance(Ray ray, Vector3 center, Vector3 axis, float radius) {
  float min_dist = 1000.0f;
  const int samples = 32;
  
  Vector3 perpendicular;
  if (fabs(axis.y) < 0.9f) {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){0, 1, 0}));
  } else {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){1, 0, 0}));
  }
  Vector3 second_perp = Vector3CrossProduct(axis, perpendicular);
  
  for (int i = 0; i < samples; i++) {
    float angle = (float)i / samples * 2.0f * PI;
    Vector3 point = Vector3Add(center,
      Vector3Add(
        Vector3Scale(perpendicular, cosf(angle) * radius),
        Vector3Scale(second_perp, sinf(angle) * radius)
      )
    );
    
    Vector3 w = Vector3Subtract(ray.position, point);
    float b = Vector3DotProduct(w, ray.direction);
    float c = Vector3DotProduct(w, w);
    float discriminant = b * b - c;
    
    float t = -b;
    if (t < 0) t = 0;
    
    Vector3 closest_on_ray = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
    float dist = Vector3Distance(closest_on_ray, point);
    
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  
  return min_dist;
}

static gizmo_type check_gizmo_hover(Ray ray) {
  float min_distance = gizmo_hover_distance;
  gizmo_type result = GIZMO_NONE;
  
  debug_ray_pos = ray.position;
  debug_ray_dir = ray.direction;
  
  gizmo_type translation_gizmos[] = { GIZMO_TRANSLATE_X, GIZMO_TRANSLATE_Y, GIZMO_TRANSLATE_Z };
  for (int i = 0; i < 3; i++) {
    Vector3 axis = get_axis_vector(translation_gizmos[i]);
    Vector3 start = cylinder_position;
    Vector3 end = Vector3Add(cylinder_position, Vector3Scale(axis, gizmo_arrow_length));
    
    float dist = ray_to_line_distance(ray, start, end, NULL);
    debug_distances[i] = dist;
    if (dist < min_distance) {
      min_distance = dist;
      result = translation_gizmos[i];
    }
  }
  
  gizmo_type rotation_gizmos[] = { GIZMO_ROTATE_X, GIZMO_ROTATE_Y, GIZMO_ROTATE_Z };
  for (int i = 0; i < 3; i++) {
    Vector3 axis = get_axis_vector(rotation_gizmos[i]);
    float dist = ray_to_circle_distance(ray, cylinder_position, axis, gizmo_rotate_radius);
    debug_distances[3 + i] = dist;
    if (dist < min_distance) {
      min_distance = dist;
      result = rotation_gizmos[i];
    }
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
  cylinder_rotation = QuaternionIdentity();
  cylinder_graphics = generate_cylinder_graphics(shader);
}

void reset() {
  cylinder_position = (Vector3){ 0, 3, 0 };
  cylinder_rotation = QuaternionIdentity();
  active_gizmo = GIZMO_NONE;
}

void on_input(Camera *camera) {
  Ray ray = GetScreenToWorldRay(GetMousePosition(), *camera);
  
  if (active_gizmo == GIZMO_NONE) {
    if (!IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
      hovered_gizmo = check_gizmo_hover(ray);
    }
    
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hovered_gizmo != GIZMO_NONE) {
      active_gizmo = hovered_gizmo;
      drag_start_cylinder_pos = cylinder_position;
      drag_start_rotation = cylinder_rotation;
      
      if (active_gizmo >= GIZMO_TRANSLATE_X && active_gizmo <= GIZMO_TRANSLATE_Z) {
        Vector3 axis = get_axis_vector(active_gizmo);
        ray_to_line_distance(ray, cylinder_position, 
          Vector3Add(cylinder_position, Vector3Scale(axis, 100)), &drag_start_pos);
      } else if (active_gizmo >= GIZMO_ROTATE_X && active_gizmo <= GIZMO_ROTATE_Z) {
        Vector3 axis = get_axis_vector(active_gizmo);
        float denom = Vector3DotProduct(ray.direction, axis);
        
        if (fabsf(denom) > 0.0001f) {
          float t = Vector3DotProduct(Vector3Subtract(cylinder_position, ray.position), axis) / denom;
          
          if (t > 0) {
            Vector3 hit_point = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
            Vector3 radial = Vector3Subtract(hit_point, cylinder_position);
            radial = Vector3Subtract(radial, Vector3Scale(axis, Vector3DotProduct(radial, axis)));
            
            if (Vector3Length(radial) > 0.001f) {
              drag_start_pos = Vector3Normalize(radial);
            }
          }
        }
      }
    }
  } else {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      if (active_gizmo >= GIZMO_TRANSLATE_X && active_gizmo <= GIZMO_TRANSLATE_Z) {
        Vector3 axis = get_axis_vector(active_gizmo);
        Vector3 current_pos;
        ray_to_line_distance(ray, drag_start_cylinder_pos, 
          Vector3Add(drag_start_cylinder_pos, Vector3Scale(axis, 100)), &current_pos);
        
        Vector3 movement = Vector3Subtract(current_pos, drag_start_pos);
        cylinder_position = Vector3Add(drag_start_cylinder_pos, movement);
      } else if (active_gizmo >= GIZMO_ROTATE_X && active_gizmo <= GIZMO_ROTATE_Z) {
        Vector3 axis = get_axis_vector(active_gizmo);
        float denom = Vector3DotProduct(ray.direction, axis);
        
        if (fabsf(denom) > 0.0001f) {
          float t = Vector3DotProduct(Vector3Subtract(cylinder_position, ray.position), axis) / denom;
          
          if (t > 0) {
            Vector3 hit_point = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
            Vector3 radial = Vector3Subtract(hit_point, cylinder_position);
            radial = Vector3Subtract(radial, Vector3Scale(axis, Vector3DotProduct(radial, axis)));
            
            if (Vector3Length(radial) > 0.001f) {
              Vector3 current_dir = Vector3Normalize(radial);
              
              Vector3 cross = Vector3CrossProduct(drag_start_pos, current_dir);
              float dot = Vector3DotProduct(drag_start_pos, current_dir);
              float angle = atan2f(Vector3DotProduct(cross, axis), dot);
              
              Quaternion rotation = QuaternionFromAxisAngle(axis, angle);
              cylinder_rotation = QuaternionMultiply(rotation, drag_start_rotation);
            }
          }
        }
      }
    } else {
      active_gizmo = GIZMO_NONE;
    }
  }
}

void simulate(float dt) {
  rigidbody temp_rb;
  temp_rb.p = cylinder_position;
  temp_rb.r = cylinder_rotation;
  
  collision col = check_collision_cylinder_plane(
    cylinder_shape, 
    &temp_rb, 
    ground_plane_point, 
    ground_plane_normal
  );
  
  is_colliding = col.valid && col.depth > 0;
  
  if (is_colliding) {
    cylinder_graphics.material.maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
  } else {
    cylinder_graphics.material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
  }
}

void draw(float interpolation) {
  DrawMesh(cylinder_graphics.mesh, cylinder_graphics.material, cylinder_transform());
  
  draw_arrow(cylinder_position, Vector3Scale((Vector3){1, 0, 0}, gizmo_arrow_length), 
    get_gizmo_color(GIZMO_TRANSLATE_X));
  draw_arrow(cylinder_position, Vector3Scale((Vector3){0, 1, 0}, gizmo_arrow_length), 
    get_gizmo_color(GIZMO_TRANSLATE_Y));
  draw_arrow(cylinder_position, Vector3Scale((Vector3){0, 0, 1}, gizmo_arrow_length), 
    get_gizmo_color(GIZMO_TRANSLATE_Z));
  
  draw_rotation_gizmo(cylinder_position, (Vector3){1, 0, 0}, get_gizmo_color(GIZMO_ROTATE_X), gizmo_rotate_radius);
  draw_rotation_gizmo(cylinder_position, (Vector3){0, 1, 0}, get_gizmo_color(GIZMO_ROTATE_Y), gizmo_rotate_radius);
  draw_rotation_gizmo(cylinder_position, (Vector3){0, 0, 1}, get_gizmo_color(GIZMO_ROTATE_Z), gizmo_rotate_radius);
}

void draw_ui(struct nk_context* ctx) {
  if (nk_begin_titled(ctx, "debug", "Debug", nk_rect(50, 50, 400, 650), NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|NK_WINDOW_CLOSABLE)) {
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_label(ctx, cylinder_graphics.label, NK_TEXT_ALIGN_LEFT);
    
    draw_stat_float3(ctx, "Position", cylinder_position);
    
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_label(ctx, is_colliding ? "Colliding: YES" : "Colliding: NO", NK_TEXT_ALIGN_LEFT);
    
    nk_layout_row_static(ctx, 30, 200, 1);
    char hover_text[64];
    snprintf(hover_text, 64, "Hovered: %d", hovered_gizmo);
    nk_label(ctx, hover_text, NK_TEXT_ALIGN_LEFT);
    
    char active_text[64];
    snprintf(active_text, 64, "Active: %d", active_gizmo);
    nk_label(ctx, active_text, NK_TEXT_ALIGN_LEFT);
    
    nk_layout_row_static(ctx, 30, 200, 1);
    nk_label(ctx, "=== Ray Debug ===", NK_TEXT_ALIGN_LEFT);
    
    draw_stat_float3(ctx, "Ray Pos", debug_ray_pos);
    draw_stat_float3(ctx, "Ray Dir", debug_ray_dir);
    
    nk_layout_row_static(ctx, 30, 300, 1);
    nk_label(ctx, "=== Gizmo Distances ===", NK_TEXT_ALIGN_LEFT);
    
    char dist_text[128];
    snprintf(dist_text, 128, "X Arrow: %.3f", debug_distances[0]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "Y Arrow: %.3f", debug_distances[1]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "Z Arrow: %.3f", debug_distances[2]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "X Circle: %.3f", debug_distances[3]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "Y Circle: %.3f", debug_distances[4]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "Z Circle: %.3f", debug_distances[5]);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    nk_layout_row_static(ctx, 30, 300, 1);
    snprintf(dist_text, 128, "Hover threshold: %.3f", gizmo_hover_distance);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    Vector2 mouse_pos = GetMousePosition();
    snprintf(dist_text, 128, "Mouse: (%.0f, %.0f)", mouse_pos.x, mouse_pos.y);
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
    
    snprintf(dist_text, 128, "LMB: %d  RMB: %d", 
      IsMouseButtonDown(MOUSE_BUTTON_LEFT), 
      IsMouseButtonDown(MOUSE_RIGHT_BUTTON));
    nk_label(ctx, dist_text, NK_TEXT_ALIGN_LEFT);
  }
  
  nk_end(ctx);
}

void save_state() { }
