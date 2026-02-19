#include "physics.h"
#include "pmath.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define ARRAY_INIT(base, type, capacity) \
  base->type##s_capacity = capacity; \
  base->type##s_count = 0; \
  base->type##s = malloc(base->type##s_capacity * sizeof(type));

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type) \
  while (count >= capacity) { \
    capacity <<= 1; \
    if (count <= capacity) { \
      array = realloc(array, capacity * sizeof(type)); \
      break; \
    } \
  }

// ======== Arena ========

typedef struct {
  uint8_t *buffer;
  count_t capacity;
  count_t used;
} arena;

static arena epa_arena;

static void arena_reset(arena *a) {
  a->used = 0;
}

static void* arena_alloc(arena *a, count_t size) {
  count_t aligned = (a->used + 15) & ~15;
  if (aligned + size > a->capacity) return NULL;
  void *ptr = a->buffer + aligned;
  a->used = aligned + size;
  return ptr;
}

// ======== Support Functions ========

static v3 support_box(v3 dir, v3 position, quat rotation, const body_shape *shape) {
  v3 local_dir = rotate(dir, qinvert(rotation));
  v3 half = scale(shape->box.size, 0.5f);
  v3 result = {
    local_dir.x >= 0 ? half.x : -half.x,
    local_dir.y >= 0 ? half.y : -half.y,
    local_dir.z >= 0 ? half.z : -half.z
  };
  return add(position, rotate(result, rotation));
}

static v3 support_sphere(v3 dir, v3 position, quat rotation, const body_shape *shape) {
  (void)rotation;
  float l = len(dir);
  if (l < 1e-8f) return position;
  return add(position, scale(dir, shape->sphere.radius / l));
}

static v3 support_cylinder(v3 dir, v3 position, quat rotation, const body_shape *shape) {
  v3 local_dir = rotate(dir, qinvert(rotation));
  float r = shape->cylinder.radius;
  float hh = shape->cylinder.half_height;

  // Y component: pick top or bottom cap
  float y = local_dir.y >= 0 ? hh : -hh;

  // XZ component: project onto disc
  float xz_len = sqrtf(local_dir.x * local_dir.x + local_dir.z * local_dir.z);
  float x, z;
  if (xz_len > 1e-8f) {
    x = r * local_dir.x / xz_len;
    z = r * local_dir.z / xz_len;
  } else {
    x = 0;
    z = 0;
  }

  v3 result = { x, y, z };
  return add(position, rotate(result, rotation));
}

static v3 support_capsule(v3 dir, v3 position, quat rotation, const body_shape *shape) {
  v3 local_dir = rotate(dir, qinvert(rotation));
  float hh = shape->capsule.half_height;

  // Pick the hemisphere center along the local Y axis
  v3 center_local = { 0, local_dir.y >= 0 ? hh : -hh, 0 };
  v3 center_world = add(position, rotate(center_local, rotation));

  // Then support of sphere at that center
  float l = len(dir);
  if (l < 1e-8f) return center_world;
  return add(center_world, scale(dir, shape->capsule.radius / l));
}

typedef v3 (*support_fn)(v3 dir, v3 position, quat rotation, const body_shape *shape);

static support_fn support_functions[] = {
  [SHAPE_BOX]      = support_box,
  [SHAPE_SPHERE]   = support_sphere,
  [SHAPE_CYLINDER] = support_cylinder,
  [SHAPE_CAPSULE]  = support_capsule,
};

static v3 support_dispatch(v3 dir, v3 position, quat rotation, const body_shape *shape) {
  return support_functions[shape->type](dir, position, rotation, shape);
}

// ======== Minkowski Difference Support ========

typedef struct {
  v3 point; // a - b
  v3 a;     // support point on shape A
  v3 b;     // support point on shape B
} support_point;

static support_point minkowski_support(
  v3 dir,
  v3 pos_a, quat rot_a, const body_shape *shape_a,
  v3 pos_b, quat rot_b, const body_shape *shape_b
) {
  v3 a = support_dispatch(dir, pos_a, rot_a, shape_a);
  v3 b = support_dispatch(negate(dir), pos_b, rot_b, shape_b);
  return (support_point){ .point = sub(a, b), .a = a, .b = b };
}

// ======== GJK ========

#define GJK_MAX_ITERATIONS 64

typedef struct {
  support_point points[4];
  count_t count;
} simplex;

static void simplex_push(simplex *s, support_point p) {
  // Shift existing points and insert new one at front
  for (count_t i = s->count; i > 0; --i)
    s->points[i] = s->points[i - 1];
  s->points[0] = p;
  if (s->count < 4) s->count++;
}

// Returns true if simplex contains origin and updates direction.
// Points[0] is always the most recently added point (A).
static bool do_simplex_line(simplex *s, v3 *dir) {
  v3 a = s->points[0].point;
  v3 b = s->points[1].point;
  v3 ab = sub(b, a);
  v3 ao = negate(a);

  if (dot(ab, ao) > 0) {
    *dir = cross(cross(ab, ao), ab);
  } else {
    s->points[0] = s->points[0];
    s->count = 1;
    *dir = ao;
  }
  return false;
}

static bool do_simplex_triangle(simplex *s, v3 *dir) {
  v3 a = s->points[0].point;
  v3 b = s->points[1].point;
  v3 c = s->points[2].point;
  v3 ab = sub(b, a);
  v3 ac = sub(c, a);
  v3 ao = negate(a);
  v3 abc = cross(ab, ac);

  if (dot(cross(abc, ac), ao) > 0) {
    if (dot(ac, ao) > 0) {
      // Region AC
      s->points[0] = s->points[0];
      s->points[1] = s->points[2];
      s->count = 2;
      *dir = cross(cross(ac, ao), ac);
    } else {
      // Reduce to line AB or point A
      s->points[1] = s->points[1];
      s->count = 2;
      return do_simplex_line(s, dir);
    }
  } else {
    if (dot(cross(ab, abc), ao) > 0) {
      // Reduce to line AB
      s->count = 2;
      return do_simplex_line(s, dir);
    } else {
      // Inside triangle; check above or below
      if (dot(abc, ao) > 0) {
        *dir = abc;
      } else {
        // Flip winding
        support_point tmp = s->points[1];
        s->points[1] = s->points[2];
        s->points[2] = tmp;
        *dir = negate(abc);
      }
    }
  }
  return false;
}

static bool do_simplex_tetrahedron(simplex *s, v3 *dir) {
  v3 a = s->points[0].point;
  v3 b = s->points[1].point;
  v3 c = s->points[2].point;
  v3 d = s->points[3].point;

  v3 ab = sub(b, a);
  v3 ac = sub(c, a);
  v3 ad = sub(d, a);
  v3 ao = negate(a);

  v3 abc = cross(ab, ac);
  v3 acd = cross(ac, ad);
  v3 adb = cross(ad, ab);

  if (dot(abc, ao) > 0) {
    // In front of face ABC
    s->points[0] = s->points[0];
    s->points[1] = s->points[1];
    s->points[2] = s->points[2];
    s->count = 3;
    return do_simplex_triangle(s, dir);
  }

  if (dot(acd, ao) > 0) {
    // In front of face ACD
    s->points[0] = s->points[0];
    s->points[1] = s->points[2];
    s->points[2] = s->points[3];
    s->count = 3;
    return do_simplex_triangle(s, dir);
  }

  if (dot(adb, ao) > 0) {
    // In front of face ADB
    s->points[0] = s->points[0];
    s->points[1] = s->points[3];
    s->points[2] = s->points[1];
    s->count = 3;
    return do_simplex_triangle(s, dir);
  }

  // Origin is inside the tetrahedron
  return true;
}

static bool do_simplex(simplex *s, v3 *dir) {
  switch (s->count) {
    case 2: return do_simplex_line(s, dir);
    case 3: return do_simplex_triangle(s, dir);
    case 4: return do_simplex_tetrahedron(s, dir);
    default: return false;
  }
}

static bool gjk(
  simplex *result,
  v3 pos_a, quat rot_a, const body_shape *shape_a,
  v3 pos_b, quat rot_b, const body_shape *shape_b
) {
  v3 dir = sub(pos_a, pos_b);
  if (lensq(dir) < 1e-8f) dir = right();

  simplex s = { .count = 0 };
  support_point sp = minkowski_support(dir, pos_a, rot_a, shape_a, pos_b, rot_b, shape_b);
  simplex_push(&s, sp);
  dir = negate(sp.point);

  for (count_t i = 0; i < GJK_MAX_ITERATIONS; ++i) {
    sp = minkowski_support(dir, pos_a, rot_a, shape_a, pos_b, rot_b, shape_b);

    if (dot(sp.point, dir) < 0) {
      return false;
    }

    simplex_push(&s, sp);

    if (do_simplex(&s, &dir)) {
      *result = s;
      return true;
    }

    // Safety: zero direction
    if (lensq(dir) < 1e-12f) {
      return false;
    }
  }

  return false;
}

// ======== EPA ========

#define EPA_MAX_ITERATIONS 64
#define EPA_TOLERANCE 1e-4f

typedef struct {
  count_t a, b, c;
  v3 normal;
  float distance;
} epa_face;

typedef struct {
  count_t a, b;
} epa_edge;

typedef struct {
  v3 normal;
  float depth;
  v3 point;
} epa_result;

static epa_face epa_make_face(const support_point *vertices, count_t a, count_t b, count_t c) {
  v3 va = vertices[a].point;
  v3 vb = vertices[b].point;
  v3 vc = vertices[c].point;

  v3 ab = sub(vb, va);
  v3 ac = sub(vc, va);
  v3 n = normalize(cross(ab, ac));
  float d = dot(n, va);

  // Ensure normal points outward (away from origin).
  // Swap winding to keep consistent edge order for horizon detection.
  if (d < 0) {
    n = negate(n);
    d = -d;
    count_t tmp = b;
    b = c;
    c = tmp;
  }

  return (epa_face){ .a = a, .b = b, .c = c, .normal = n, .distance = d };
}

static bool epa(
  const simplex *initial,
  epa_result *result,
  v3 pos_a, quat rot_a, const body_shape *shape_a,
  v3 pos_b, quat rot_b, const body_shape *shape_b
) {
  arena_reset(&epa_arena);

  count_t max_vertices = 4 + EPA_MAX_ITERATIONS;
  count_t max_faces = 4 + EPA_MAX_ITERATIONS * 4;
  count_t max_edges = EPA_MAX_ITERATIONS * 3;

  support_point *vertices = arena_alloc(&epa_arena, max_vertices * sizeof(support_point));
  epa_face *faces = arena_alloc(&epa_arena, max_faces * sizeof(epa_face));
  epa_edge *edges = arena_alloc(&epa_arena, max_edges * sizeof(epa_edge));

  if (!vertices || !faces || !edges) return false;

  count_t vertex_count = 4;
  count_t face_count = 0;

  for (count_t i = 0; i < 4; ++i)
    vertices[i] = initial->points[i];

  // Build initial tetrahedron (4 faces)
  // Ensure consistent winding: each face's normal should point away from the
  // centroid of the tetrahedron. We build faces and fix winding.
  v3 centroid = scale(add(add(vertices[0].point, vertices[1].point),
                          add(vertices[2].point, vertices[3].point)), 0.25f);

  count_t face_indices[4][3] = {
    {0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}
  };

  for (count_t i = 0; i < 4; ++i) {
    count_t a = face_indices[i][0];
    count_t b = face_indices[i][1];
    count_t c = face_indices[i][2];

    v3 va = vertices[a].point;
    v3 vb = vertices[b].point;
    v3 vc = vertices[c].point;
    v3 ab = sub(vb, va);
    v3 ac = sub(vc, va);
    v3 n = cross(ab, ac);
    v3 face_center = scale(add(add(va, vb), vc), 1.0f / 3.0f);

    // Normal should point away from centroid
    if (dot(n, sub(face_center, centroid)) < 0) {
      // Swap b and c to flip winding
      count_t tmp = b;
      b = c;
      c = tmp;
    }

    faces[face_count++] = epa_make_face(vertices, a, b, c);
  }

  for (count_t iteration = 0; iteration < EPA_MAX_ITERATIONS; ++iteration) {
    // Find closest face to origin
    count_t closest_face = 0;
    float closest_distance = faces[0].distance;
    for (count_t i = 1; i < face_count; ++i) {
      if (faces[i].distance < closest_distance) {
        closest_distance = faces[i].distance;
        closest_face = i;
      }
    }

    v3 search_dir = faces[closest_face].normal;
    support_point new_point = minkowski_support(search_dir, pos_a, rot_a, shape_a, pos_b, rot_b, shape_b);

    float new_distance = dot(new_point.point, search_dir);
    if (new_distance - closest_distance < EPA_TOLERANCE) {
      // Converged. Compute contact point via barycentric coordinates.
      epa_face *f = &faces[closest_face];
      v3 va = vertices[f->a].point;
      v3 vb = vertices[f->b].point;
      v3 vc = vertices[f->c].point;

      // Project origin onto the face plane
      v3 projected = scale(f->normal, f->distance);

      // Barycentric coordinates of projected point in triangle
      v3 v0 = sub(vb, va);
      v3 v1 = sub(vc, va);
      v3 v2 = sub(projected, va);

      float d00 = dot(v0, v0);
      float d01 = dot(v0, v1);
      float d11 = dot(v1, v1);
      float d20 = dot(v2, v0);
      float d21 = dot(v2, v1);

      float denom = d00 * d11 - d01 * d01;
      if (fabsf(denom) < 1e-10f) {
        // Degenerate triangle, use face center
        result->point = scale(add(add(vertices[f->a].a, vertices[f->b].a), vertices[f->c].a), 1.0f / 3.0f);
        v3 point_b = scale(add(add(vertices[f->a].b, vertices[f->b].b), vertices[f->c].b), 1.0f / 3.0f);
        result->point = scale(add(result->point, point_b), 0.5f);
      } else {
        float bv = (d11 * d20 - d01 * d21) / denom;
        float bw = (d00 * d21 - d01 * d20) / denom;
        float bu = 1.0f - bv - bw;

        // Clamp barycentric coordinates
        if (bu < 0) bu = 0;
        if (bv < 0) bv = 0;
        if (bw < 0) bw = 0;
        float sum = bu + bv + bw;
        if (sum > 0) { bu /= sum; bv /= sum; bw /= sum; }

        v3 point_a = add(add(scale(vertices[f->a].a, bu), scale(vertices[f->b].a, bv)), scale(vertices[f->c].a, bw));
        v3 point_b = add(add(scale(vertices[f->a].b, bu), scale(vertices[f->b].b, bv)), scale(vertices[f->c].b, bw));
        result->point = scale(add(point_a, point_b), 0.5f);
      }

      result->normal = f->normal;
      result->depth = closest_distance;
      return true;
    }

    // Add new vertex
    if (vertex_count >= max_vertices) {
      // Out of vertex space - return best so far
      epa_face *f = &faces[closest_face];
      result->normal = f->normal;
      result->depth = closest_distance;
      result->point = scale(add(add(vertices[f->a].a, vertices[f->b].a), vertices[f->c].a), 1.0f / 3.0f);
      return true;
    }
    count_t new_vertex = vertex_count++;
    vertices[new_vertex] = new_point;

    // Find and remove faces visible from new point, collect horizon edges
    count_t edge_count = 0;

    for (count_t i = 0; i < face_count; ) {
      if (dot(faces[i].normal, sub(new_point.point, vertices[faces[i].a].point)) > 0) {
        // Face is visible from new point. Add its edges to horizon.
        count_t fa = faces[i].a;
        count_t fb = faces[i].b;
        count_t fc = faces[i].c;

        epa_edge face_edges[3] = {
          { fa, fb }, { fb, fc }, { fc, fa }
        };

        for (count_t e = 0; e < 3; ++e) {
          // Check if reverse edge already exists (shared edge = not on horizon)
          bool found = false;
          for (count_t k = 0; k < edge_count; ++k) {
            if (edges[k].a == face_edges[e].b && edges[k].b == face_edges[e].a) {
              // Remove this edge (shared, not on horizon)
              edges[k] = edges[edge_count - 1];
              edge_count--;
              found = true;
              break;
            }
          }
          if (!found && edge_count < max_edges) {
            edges[edge_count++] = face_edges[e];
          }
        }

        // Remove this face
        faces[i] = faces[face_count - 1];
        face_count--;
      } else {
        ++i;
      }
    }

    // Create new faces from horizon edges to new vertex
    for (count_t i = 0; i < edge_count; ++i) {
      if (face_count >= max_faces) break;
      faces[face_count++] = epa_make_face(vertices, edges[i].a, edges[i].b, new_vertex);
    }

    if (face_count == 0) {
      return false;
    }
  }

  // Didn't converge; return best result
  if (face_count > 0) {
    count_t closest_face = 0;
    float closest_distance = faces[0].distance;
    for (count_t i = 1; i < face_count; ++i) {
      if (faces[i].distance < closest_distance) {
        closest_distance = faces[i].distance;
        closest_face = i;
      }
    }
    epa_face *f = &faces[closest_face];
    result->normal = f->normal;
    result->depth = closest_distance;
    result->point = scale(add(add(vertices[f->a].a, vertices[f->b].a), vertices[f->c].a), 1.0f / 3.0f);
    return true;
  }

  return false;
}

// ======== GJK/EPA Collision Wrapper ========

static count_t gjk_epa_collision(
  collisions *collisions,
  count_t index_a, count_t index_b,
  const common_data *data_a, const common_data *data_b
) {
  v3 pos_a = data_a->positions[index_a];
  quat rot_a = data_a->rotations[index_a];
  const body_shape *shape_a = &data_a->shapes[index_a];

  v3 pos_b = data_b->positions[index_b];
  quat rot_b = data_b->rotations[index_b];
  const body_shape *shape_b = &data_b->shapes[index_b];

  simplex s;
  if (!gjk(&s, pos_a, rot_a, shape_a, pos_b, rot_b, shape_b))
    return 0;

  epa_result epa_res;
  if (!epa(&s, &epa_res, pos_a, rot_a, shape_a, pos_b, rot_b, shape_b))
    return 0;

  if (epa_res.depth < 1e-6f)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision *col = &collisions->collisions[collisions->collisions_count++];
  col->index_a = index_a;
  col->index_b = index_b;
  col->contacts_count = 1;
  col->contacts_offset = collisions->contacts_count;

  contact *ct = &collisions->contacts[collisions->contacts_count++];
  ct->point = epa_res.point;
  // EPA normal points from B toward A (Minkowski difference convention A - B)
  ct->normal = epa_res.normal;
  ct->depth = epa_res.depth;

  return 1;
}

// ======== Plane Collision Functions ========

static count_t box_plane_collision(
  collisions *collisions,
  count_t index_a, count_t index_b,
  const common_data *data_a, const common_data *data_b
) {
  v3 extents = scale(data_a->shapes[index_a].box.size, 0.5);
  v3 plane_normal = data_b->shapes[index_b].plane.normal;

  v3 corners[] = {
    {  extents.x,  extents.y,  extents.z },
    {  extents.x, -extents.y,  extents.z },
    {  extents.x, -extents.y, -extents.z },
    {  extents.x,  extents.y, -extents.z },
    { -extents.x,  extents.y,  extents.z },
    { -extents.x, -extents.y,  extents.z },
    { -extents.x, -extents.y, -extents.z },
    { -extents.x,  extents.y, -extents.z },
  };

  const count_t max_contacts = 4;

  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + max_contacts, collisions->contacts_capacity, contact)

  count_t contact_count = 0;
  contact *contacts = collisions->contacts + collisions->contacts_count;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    v3 corner = add(data_a->positions[index_a], rotate(corners[i], data_a->rotations[index_a]));
    float distance = dot(sub(corner, data_b->positions[index_b]), plane_normal);
    if (distance > 0)
      continue;

    contact *new_contact = &contacts[contact_count++];

    new_contact->normal = plane_normal;
    new_contact->point = add(corner, scale(plane_normal, -0.5 * distance));
    new_contact->depth = -distance;
  }

  if (contact_count == 0)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);

  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return 1;
}

static count_t sphere_plane_collision(
  collisions *collisions,
  count_t index_a, count_t index_b,
  const common_data *data_a, const common_data *data_b
) {
  v3 plane_point = data_b->positions[index_b];
  v3 plane_normal = data_b->shapes[index_b].plane.normal;
  v3 sphere_center = data_a->positions[index_a];
  float sphere_radius = data_a->shapes[index_a].sphere.radius;

  float plane_sphere_distance = dot(sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + 1, collisions->contacts_capacity, contact);

  collision *col = &collisions->collisions[collisions->collisions_count++];
  contact *ct = &collisions->contacts[collisions->contacts_count];

  col->index_a = index_a;
  col->index_b = index_b;
  col->contacts_offset = collisions->contacts_count;
  col->contacts_count = 1;

  ct->normal = plane_normal;
  ct->point = add(sphere_center, scale(plane_normal, -plane_sphere_distance));
  ct->depth = sphere_radius - plane_sphere_distance;

  collisions->contacts_count++;

  return 1;
}

static count_t cylinder_plane_collision(
  collisions *collisions,
  count_t index_a, count_t index_b,
  const common_data *data_a, const common_data *data_b
) {
  v3 position = data_a->positions[index_a];
  quat rotation = data_a->rotations[index_a];
  float radius = data_a->shapes[index_a].cylinder.radius;
  float hh = data_a->shapes[index_a].cylinder.half_height;

  v3 plane_point = data_b->positions[index_b];
  v3 plane_normal = data_b->shapes[index_b].plane.normal;

  // Cylinder local Y axis in world space
  v3 cyl_axis = rotate(up(), rotation);

  // Two cap centers in world space
  v3 top_center = add(position, scale(cyl_axis, hh));
  v3 bot_center = sub(position, scale(cyl_axis, hh));

  // For each cap, find the point on the cap circle closest to the plane
  // The direction on the cap disc that goes deepest into the plane is
  // the component of -plane_normal projected onto the disc plane, normalized.
  // disc_dir = normalize(-plane_normal - dot(-plane_normal, cyl_axis) * cyl_axis)
  v3 neg_normal = negate(plane_normal);
  v3 proj = sub(neg_normal, scale(cyl_axis, dot(neg_normal, cyl_axis)));
  float proj_len = len(proj);

  const count_t max_contacts = 2;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + max_contacts, collisions->contacts_capacity, contact);

  count_t contact_count = 0;
  contact *contacts = collisions->contacts + collisions->contacts_count;

  v3 cap_centers[2] = { top_center, bot_center };
  for (count_t i = 0; i < 2; ++i) {
    // Deepest point on this cap's rim
    v3 deepest;
    if (proj_len > 1e-8f) {
      deepest = add(cap_centers[i], scale(proj, radius / proj_len));
    } else {
      // Plane normal is parallel to cylinder axis; any rim point works
      v3 perp = rotate(right(), rotation);
      deepest = add(cap_centers[i], scale(perp, radius));
    }

    float dist = dot(sub(deepest, plane_point), plane_normal);
    if (dist > 0) continue;

    contact *ct = &contacts[contact_count++];
    ct->normal = plane_normal;
    ct->point = add(deepest, scale(plane_normal, -0.5f * dist));
    ct->depth = -dist;
  }

  if (contact_count == 0)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);

  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return 1;
}

static count_t capsule_plane_collision(
  collisions *collisions,
  count_t index_a, count_t index_b,
  const common_data *data_a, const common_data *data_b
) {
  v3 position = data_a->positions[index_a];
  quat rotation = data_a->rotations[index_a];
  float radius = data_a->shapes[index_a].capsule.radius;
  float hh = data_a->shapes[index_a].capsule.half_height;

  v3 plane_point = data_b->positions[index_b];
  v3 plane_normal = data_b->shapes[index_b].plane.normal;

  // Capsule axis in world space
  v3 cap_axis = rotate(up(), rotation);

  // Two hemisphere centers
  v3 top_center = add(position, scale(cap_axis, hh));
  v3 bot_center = sub(position, scale(cap_axis, hh));

  const count_t max_contacts = 2;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->contacts_count + max_contacts, collisions->contacts_capacity, contact);

  count_t contact_count = 0;
  contact *contacts = collisions->contacts + collisions->contacts_count;

  v3 centers[2] = { top_center, bot_center };
  for (count_t i = 0; i < 2; ++i) {
    // Closest point on hemisphere surface toward plane
    float center_dist = dot(sub(centers[i], plane_point), plane_normal);
    if (center_dist > radius) continue;

    v3 sphere_point = sub(centers[i], scale(plane_normal, radius));
    float point_dist = dot(sub(sphere_point, plane_point), plane_normal);
    if (point_dist > 0) continue;

    contact *ct = &contacts[contact_count++];
    ct->normal = plane_normal;
    ct->point = add(sphere_point, scale(plane_normal, -0.5f * point_dist));
    ct->depth = -point_dist;
  }

  if (contact_count == 0)
    return 0;

  ARRAY_RESIZE_IF_NEEDED(collisions->collisions, collisions->collisions_count + 1, collisions->collisions_capacity, collision);

  collision c = { .index_a = index_a, .index_b = index_b, .contacts_count = contact_count, .contacts_offset = collisions->contacts_count };
  collisions->collisions[collisions->collisions_count++] = c;
  collisions->contacts_count += contact_count;

  return 1;
}

// ======== Plane dispatch ========

typedef count_t (*plane_collision_fn)(collisions*, count_t, count_t, const common_data*, const common_data*);

static plane_collision_fn plane_detectors[] = {
  [SHAPE_BOX]      = box_plane_collision,
  [SHAPE_SPHERE]   = sphere_plane_collision,
  [SHAPE_CYLINDER] = cylinder_plane_collision,
  [SHAPE_CAPSULE]  = capsule_plane_collision,
};

// ======== Public API ========

#define EPA_ARENA_SIZE (32 * 1024)

collisions* collisions_init(const physics_config *config) {
  collisions *result = malloc(sizeof(collisions));

  ARRAY_INIT(result, collision, config->collisions_capacity);
  ARRAY_INIT(result, contact, config->collisions_capacity * 4);

  epa_arena.capacity = EPA_ARENA_SIZE;
  epa_arena.buffer = malloc(EPA_ARENA_SIZE);
  epa_arena.used = 0;

  return result;
}

void collisions_detect(collisions *collisions, const common_data *dynamics, const common_data *statics) {
  collisions->collisions_count = 0;
  collisions->contacts_count = 0;

  count_t dyn_count = 0;

  // Dynamic-dynamic pairs
  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; ++j) {
      shape_type type_a = dynamics->shapes[i].type;
      shape_type type_b = dynamics->shapes[j].type;

      // Skip planes (shouldn't be dynamic, but guard anyway)
      if (type_a == SHAPE_PLANE || type_b == SHAPE_PLANE)
        continue;

      dyn_count += gjk_epa_collision(collisions, i, j, dynamics, dynamics);
    }
  }

  collisions->dynamic_collisions_count = dyn_count;

  // Dynamic-static pairs
  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < statics->count; ++j) {
      shape_type dyn_type = dynamics->shapes[i].type;
      shape_type stat_type = statics->shapes[j].type;

      if (stat_type == SHAPE_PLANE) {
        if (dyn_type < SHAPES_COUNT && plane_detectors[dyn_type])
          plane_detectors[dyn_type](collisions, i, j, dynamics, statics);
      } else {
        // Both are convex shapes, use GJK/EPA
        // Convention: index_a = dynamic index, index_b = static index
        // GJK/EPA with A=dynamic, B=static gives normal from static->dynamic
        if (dyn_type != SHAPE_PLANE)
          gjk_epa_collision(collisions, i, j, dynamics, statics);
      }
    }
  }
}

void collisions_teardown(collisions *collisions) {
  free(collisions->collisions);
  free(collisions->contacts);
  free(epa_arena.buffer);
  epa_arena.buffer = NULL;
  epa_arena.capacity = 0;
  epa_arena.used = 0;
  free(collisions);
}
