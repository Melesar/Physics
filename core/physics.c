#include "raylib.h"
#include "physics.h"
#include "raymath.h"
#include "stdlib.h"
#include "string.h"

#define COLLISION_MAX_POINTS 32
#define COLLISION_MAX_INDICES (3 * COLLISION_MAX_FACES)
#define COLLISION_MAX_FACES 64

#define SOLVER_TOLERANCE 0.01
#define MAX_GJK_ATTEMPTS 20
#define GJK_TOLERANCE 0.0001
#define EPA_TOLERANCE 0.001

typedef enum {
  SHAPE_CYLINDER,
  SHAPE_SPHERE,
  SHAPE_BOX,
} shape_type;

typedef struct {
  Vector3 minkowski;
  Vector3 support_a;
  Vector3 support_b;
} support_point;

Vector3 cylinder_inertia_tensor(cylinder c, float mass) {
  float principal =  mass * (3 * c.radius * c.radius + c.height * c.height) / 12.0;
  return (Vector3){ principal, mass * c.radius * c.radius / 2.0, principal };
}

Vector3 sphere_inertia_tensor(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;  
  return scale(one(), scale);
}

Vector3 box_inertia_tensor(Vector3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  Vector3 i = { yy + zz, xx + zz, xx + yy };
  return scale(i, m);
}

rigidbody rb_new(Vector3 position, float mass) {
  return (rigidbody) {
    .f = zero(),
    .fi = zero(),
    .v = zero(),
    .p = position,

    .t = zero(),
    .ti = zero(),
    .r = qidentity(),
    .i0_inv = one(),
    .l = zero(),

    .mass = mass,
  };
}

Matrix rb_transformation(const rigidbody* rb) {
  return mul(
    as_matrix(rb->r),
    translate(rb->p.x, rb->p.y, rb->p.z)
  );
}

Matrix rb_transformation_with_offset(const rigidbody *rb, Vector3 offset) {
  return mul(
    mul(
      translate(rb->p.x, rb->p.y, rb->p.z),
      as_matrix(rb->r)),
        translate(offset.x, offset.y, offset.z));
}

static Matrix rb_inv_i0_m(const rigidbody* rb) {
  Matrix inv_i0 = { 0 };
  inv_i0.m0 = rb->i0_inv.x;
  inv_i0.m5 = rb->i0_inv.y;
  inv_i0.m10 = rb->i0_inv.z;
  return inv_i0;
}

static Vector3 rb_angular_velocity_ex(const rigidbody* rb, Matrix inv_i0) {
  Matrix orientation = as_matrix(rb->r);
  Matrix transform = mul(mul(orientation, inv_i0), transpose(orientation));
  return transform(rb->l, transform);
}

static Matrix rb_inertia_world_ex(const rigidbody* rb, Matrix orientation, Matrix inv_i0) {
  return mul(mul(orientation, inv_i0), transpose(orientation));
}

Matrix rb_inertia_world(const rigidbody* rb) {
  Matrix orientation = as_matrix(rb->r);
  Matrix inv_i0 = rb_inv_i0_m(rb);

  return rb_inertia_world_ex(rb, orientation, inv_i0);
}

Vector3 rb_angular_velocity(const rigidbody* rb) {
  Matrix inv_i0 = rb_inv_i0_m(rb);
  return rb_angular_velocity_ex(rb, inv_i0);
}

void rb_angular_params(const rigidbody *rb, Matrix *inertia, Vector3 *omega) {
  Matrix orientation = as_matrix(rb->r);
  Matrix inv_i0 = rb_inv_i0_m(rb);
  *inertia = mul(mul(orientation, inv_i0), transpose(orientation));
  *omega = transform(rb->l, *inertia);
}

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t) {
  rigidbody result;
  result.p = vlerp(from->p, to->p, t);
  result.r = slerp(from->r, to->r, t);
  result.v = vlerp(from->v, to->v, t);
  result.mass = lerp(from->mass, to->mass, t);

  return result;
}

void rb_apply_impulse_at(rigidbody* rb, Vector3 at, Vector3 impulse) {
  Vector3 r = sub(at, rb->p); 
  rb->ti = add(rb->ti, cross(r, impulse));
  rb->fi = add(rb->fi, impulse);
}

void rb_apply_force_at(rigidbody* rb, Vector3 at, Vector3 force) {
  Vector3 r = sub(at, rb->p); 
  rb->t = add(rb->t, cross(r, force));
  rb->f = add(rb->f, force);
}

void rb_apply_impulse(rigidbody* rb, Vector3 impulse) {
  rb->fi = add(rb->fi, impulse);
}

void rb_apply_force(rigidbody* rb, Vector3 force) {
  rb->f = add(rb->f, force);
}

void rb_simulate(rigidbody* rb, float dt) {
  rb->l = add(rb->l, rb->ti);
  rb->l = add(rb->l, scale(rb->t, dt));

  Matrix inv_i0 = rb_inv_i0_m(rb);
  Vector3 omega = rb_angular_velocity_ex(rb, inv_i0);
  Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
  Quaternion dq = qscale(qmul(q_omega, rb->r), 0.5 * dt);

  Quaternion q_orientation = qadd(rb->r, dq);
  rb->r = qnormalize(q_orientation);
  rb->t = rb->ti = zero();

  float inv_mass = 1.0 / rb->mass;
  Vector3 acc = scale(add(rb->fi, scale(rb->f, dt)), inv_mass);
  rb->v = add(rb->v, acc);
  rb->p = add(rb->p, scale(rb->v, dt));
  rb->f = rb->fi = zero();
}

Vector3 sphere_support(Vector3 center, float radius, Vector3 direction) {
  return add(center, scale(direction, radius));
}

Vector3 cylinder_support(Vector3 center, float radius, float height, Quaternion rotation, Vector3 direction) {
  Quaternion inv_rotation = qinvert(rotation);
  Vector3 local_dir = rotate(direction, inv_rotation);

  float half_height = 0.5f * height;

  float axial_component = local_dir.y;
  Vector3 radial_dir = (Vector3){ local_dir.x, 0.0f, local_dir.z };
  float radial_magnitude = sqrtf(radial_dir.x * radial_dir.x + radial_dir.z * radial_dir.z);

  Vector3 local_support;

  if (radial_magnitude > 1e-6f) {
    local_support.x = (radial_dir.x / radial_magnitude) * radius;
    local_support.z = (radial_dir.z / radial_magnitude) * radius;
  } else {
    local_support.x = 0.0f;
    local_support.z = 0.0f;
  }
  if (axial_component > 0.0f) {
    local_support.y = half_height;
  } else {
    local_support.y = -half_height;
  }

  Vector3 world_support = rotate(local_support, rotation);
  return add(center, world_support);
}

Vector3 box_support(Vector3 center, Vector3 size, Quaternion rotation, Vector3 direction) {
  Quaternion inv_rotation = qinvert(rotation);
  Vector3 local_dir = rotate(direction, inv_rotation);

  Vector3 half = scale(size, 0.5f);
  Vector3 local_support = {
    local_dir.x >= 0 ? half.x : -half.x,
    local_dir.y >= 0 ? half.y : -half.y,
    local_dir.z >= 0 ? half.z : -half.z
  };

  Vector3 world_support = rotate(local_support, rotation);
  return add(center, world_support);
}

Vector3 single_shape_support(shape_type type, const rigidbody *rb, Vector3 params, Vector3 direction) {
  switch(type) {
    case SHAPE_CYLINDER:
      return cylinder_support(rb->p, params.y, params.x, rb->r, direction);

    case SHAPE_SPHERE:
      return sphere_support(rb->p, params.x, direction);

    case SHAPE_BOX:
      return box_support(rb->p, params, rb->r, direction);
  }
}

support_point shapes_support(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector3 params_a, Vector3 params_b, Vector3 direction) {
  Vector3 support_a = single_shape_support(shape_a, rb_a, params_a, direction);
  Vector3 support_b = single_shape_support(shape_b, rb_b, params_b, negate(direction));

  support_point point = {0};
  point.support_a = support_a;
  point.support_b = support_b;
  point.minkowski = sub(support_a, support_b);
  return point;
}

static void face_normals(Vector4 *normals, int *num_normals, int *min_face_index, support_point *points, int num_points, int *indices, int num_indices) {
  float min_distance = INFINITY;

  for (int i = 0; i < num_indices; i += 3) {
    Vector3 a = points[indices[i]].minkowski;
    Vector3 b = points[indices[i + 1]].minkowski;
    Vector3 c = points[indices[i + 2]].minkowski;
    
    Vector3 normal = normalize(cross(sub(b, a), sub(c, a)));
    float distance = dot(normal, a);

    if (distance < 0) {
      normal = scale(normal, -1);
      distance *= -1;
    }

    normals[(*num_normals)++] = (Vector4) { normal.x, normal.y, normal.z, distance  };

    if (distance < min_distance) {
      *min_face_index = i / 3;
      min_distance = distance;
    }
  }
}

static void add_if_unique_edge(int *unique_edges, int *num_unique_edges, int *faces, int num_faces, int a, int b) {
  for (int i = *num_unique_edges - 1; i >= 0; i -= 2) {
    if (unique_edges[i] == faces[a] && unique_edges[i - 1] == faces[b]) {
      unique_edges[i] = unique_edges[*num_unique_edges - 1];
      unique_edges[i - 1] = unique_edges[*num_unique_edges - 2];
      *num_unique_edges -= 2;
      return;
    }
  }

  unique_edges[*num_unique_edges] = faces[a];
  unique_edges[*num_unique_edges + 1] = faces[b];
  *num_unique_edges += 2;
}

static int epa(support_point *points, int num_points, shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector3 params_a, Vector3 params_b, collision *contacts, int max_contacts) {
  if (contacts == NULL || max_contacts <= 0) {
    return 0;
  }

  int num_indices = 12;
  int indices[COLLISION_MAX_INDICES];
  indices[0] = 0;
  indices[1] = 1;
  indices[2] = 2;

  indices[3] = 0;
  indices[4] = 3;
  indices[5] = 1;

  indices[6] = 0;
  indices[7] = 2;
  indices[8] = 3;

  indices[9] = 1;
  indices[10] = 3;
  indices[11] = 2;

  int num_normals = 0;
  int min_face_index = -1;
  Vector4 normals[COLLISION_MAX_FACES];
  face_normals(normals, &num_normals, &min_face_index, points, num_points, indices, num_indices);

  Vector3 min_normal;
  float min_distance = INFINITY;
  while (min_distance == INFINITY) {
    min_normal = *(Vector3*)&normals[min_face_index];
    min_distance = normals[min_face_index].w;

    support_point support = shapes_support(shape_a, shape_b, rb_a, rb_b, params_a, params_b, min_normal);
    float distance = dot(min_normal, support.minkowski);
    if (fabsf(distance - min_distance) > EPA_TOLERANCE) {
      min_distance = INFINITY;
    } else {
      break;
    }

    int num_unique_edges = 0;
    int unique_edges[COLLISION_MAX_INDICES] = {0};
    for (int i = 0; i < num_normals; ++i) {
      Vector3 normal = *(Vector3*)&normals[i];
      if (dot(normal, support.minkowski) - normals[i].w <= EPA_TOLERANCE) {
        continue;
      }

      int f = i * 3;
      add_if_unique_edge(unique_edges, &num_unique_edges, indices, num_indices, f, f + 1);
      add_if_unique_edge(unique_edges, &num_unique_edges, indices, num_indices, f + 1, f + 2);
      add_if_unique_edge(unique_edges, &num_unique_edges, indices, num_indices, f + 2, f);

      indices[f + 2] = indices[num_indices - 1];
      indices[f + 1] = indices[num_indices - 2];
      indices[f] = indices[num_indices - 3];

      num_indices -= 3;

      normals[i] = normals[num_normals - 1];
      num_normals -= 1;

      i -= 1;
    }

    int num_new_faces = 0;
    int new_faces[COLLISION_MAX_INDICES];
    for (int i = 0; i < num_unique_edges; i += 2) {
      int edge_1 = unique_edges[i];
      int edge_2 = unique_edges[i + 1];
      new_faces[num_new_faces++] = edge_1;
      new_faces[num_new_faces++] = edge_2;
      new_faces[num_new_faces++] = num_points;
    }

    if (num_points >= COLLISION_MAX_POINTS) {
      TraceLog(LOG_ERROR, "EPA: Maximum number of points reached: %d", num_points);
      return 0;
    }

    points[num_points++] = support;

    int num_new_normals = 0;
    int new_min_face = -1;
    Vector4 new_normals[COLLISION_MAX_FACES];

    face_normals(new_normals, &num_new_normals, &new_min_face, points, num_points, new_faces, num_new_faces);

    float old_min_distance = INFINITY;
    for (int i = 0; i < num_normals; ++i) {
      if (normals[i].w < old_min_distance) {
        old_min_distance = normals[i].w;
        min_face_index = i;
      }
    }

    if (new_normals[new_min_face].w < old_min_distance) {
      min_face_index = new_min_face + num_normals;
    }

    if (num_indices + num_new_faces > COLLISION_MAX_INDICES) {
      TraceLog(LOG_ERROR, "EPA: Maximum number of indices exceeded: %d", num_indices + num_new_faces);
      return 0;
    }

    if (num_normals + num_new_normals > COLLISION_MAX_FACES) {
      TraceLog(LOG_ERROR, "EPA: Maximum number of faces exceeded: %d", num_normals + num_new_normals);
      return 0;
    }

    memcpy(indices + num_indices, new_faces, num_new_faces * sizeof(int));
    memcpy(normals + num_normals, new_normals, num_new_normals * sizeof(Vector4));

    num_indices += num_new_faces;
    num_normals += num_new_normals;
  }

  Vector3 t1;
  if (min_normal.x >= 0.57735f) {
    t1.x = min_normal.y;
    t1.y = -min_normal.x;
    t1.z = 0;
  } else {
    t1.x = 0;
    t1.y = min_normal.z;
    t1.z = -min_normal.y;
  }

  Vector3 tangent = normalize(t1);
  Vector3 bitangent = cross(min_normal, tangent);

  int face_base = min_face_index * 3;
  int contact_count = 0;
  float penetration = fmaxf(0.0f, min_distance + 0.001f);

  for (int i = 0; i < 3 && contact_count < max_contacts; ++i) {
    int idx = indices[face_base + i];
    support_point vertex = points[idx];
    collision *contact = &contacts[contact_count++];
    contact->valid = true;
    contact->normal = min_normal;
    contact->tangent = tangent;
    contact->bitangent = bitangent;
    contact->depth = penetration;
    contact->world_contact_a = vertex.support_a;
    contact->world_contact_b = vertex.support_b;
    contact->local_contact_a = sub(vertex.support_a, rb_a->p);
    contact->local_contact_b = sub(vertex.support_b, rb_b->p);
  }

  if (contact_count == 0) {
    Vector3 support_a = single_shape_support(shape_a, rb_a, params_a, min_normal);
    Vector3 support_b = single_shape_support(shape_b, rb_b, params_b, negate(min_normal));
    collision *contact = &contacts[contact_count++];
    contact->valid = true;
    contact->normal = min_normal;
    contact->tangent = tangent;
    contact->bitangent = bitangent;
    contact->depth = penetration;
    contact->world_contact_a = add(support_a, scale(min_normal, -min_distance * 0.5f));
    contact->world_contact_b = add(support_b, scale(min_normal, min_distance * 0.5f));
    contact->local_contact_a = sub(contact->world_contact_a, rb_a->p);
    contact->local_contact_b = sub(contact->world_contact_b, rb_b->p);
  }

  return contact_count;
}

static bool gjk_update_simplex(support_point *points, int *count, Vector3 *direction) {
  support_point a, b, c, d;
  Vector3 ab, ac, ad, ao;
  Vector3 abc, acd, adb;

  switch(*count) {
    case 4:
      a = points[0];
      b = points[1];
      c = points[2];
      d = points[3];

      ab = sub(b.minkowski, a.minkowski);
      ac = sub(c.minkowski, a.minkowski);
      ad = sub(d.minkowski, a.minkowski);
      ao = negate(a.minkowski);

      abc = cross(ab, ac);
      acd = cross(ac, ad);
      adb = cross(ad, ab);

      if (dot(abc, ao) > 0) {
        *count = 3;
        // Fallthrough to case 3
      } else if (dot(acd, ao) > 0) {
        points[1] = c;
        points[2] = d;
        *count = 3;
        //Fallthrough to case 3
      } else if (dot(adb, ao) > 0) {
        points[1] = d;
        points[2] = b;
        *count = 3;
        //Fallthrough to case 3
      } else {
        return true;
      }
      
    case 3:
      a = points[0];
      b = points[1];
      c = points[2];

      ab = sub(b.minkowski, a.minkowski);
      ac = sub(c.minkowski, a.minkowski);
      ao = negate(a.minkowski);

      Vector3 abc = cross(ab, ac);
      if (dot(cross(abc, ac), ao) > 0) {
    		if (dot(ac, ao) > 0) {
    			points[1] = c;
    			*count = 2;
    			*direction = normalize(cross(cross(ac, ao), ac));

    			return false;
    		} else {
    		  *count = 2;
    		  // Fallthrough to case 2
    		}
    	} else {
    		if (dot(cross(ab, abc), ao) > 0) {
    		  *count = 2;
    		  // Fallthrough to case 2
    		} else {
    			if (dot(abc, ao) > 0) {
    				*direction = normalize(abc);
    				return false;
    			} else {
    			  points[2] = b;
    			  points[1] = c;

    				*direction = normalize(negate(abc));
    				return false;
    			}
    		}
    	}

    case 2:
      ab = sub(points[1].minkowski, points[0].minkowski);
      ao = negate(points[0].minkowski);

      if (dot(ab, ao) > 0) {
        *direction = normalize(cross(cross(ab, ao), ab));
      } else {
        *count = 1;
        *direction = normalize(ao); 
      }
      return false;

    case 1:
      *direction = normalize(negate(points[0].minkowski));
      return false;
  }

  return false;
}

int check_collision(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector3 params_a, Vector3 params_b, collision *contacts, int max_contacts) {
  if (contacts == NULL || max_contacts <= 0) {
    return 0;
  }

  Vector3 direction = right();

  int num_points = 0;
  support_point points[COLLISION_MAX_POINTS];

  int num_attempts = 0;
  while(++num_attempts < MAX_GJK_ATTEMPTS) {
    support_point support = shapes_support(shape_a, shape_b, rb_a, rb_b, params_a, params_b, direction);

    if (dot(support.minkowski, direction) < GJK_TOLERANCE) {
      return 0;
    }

    points[3] = points[2];
    points[2] = points[1];
    points[1] = points[0];
    points[0] = support;

    num_points += 1;

    if (gjk_update_simplex(points, &num_points, &direction)) {
      return epa(points, num_points, shape_a, shape_b, rb_a, rb_b, params_a, params_b, contacts, max_contacts);
    }
  }

  TraceLog(LOG_WARNING, "Max GJK attempts reached withot the result. Collision detection failed");
  return 0;
}

int cylinder_sphere_contact_manifold(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius, collision *contacts, int max_contacts) {
  return check_collision(SHAPE_SPHERE, SHAPE_CYLINDER, sphere_rb, cylinder_rb, (Vector3) { sphere_radius, 0, 0 }, (Vector3) { cylinder_height, cylinder_radius, 0 }, contacts, max_contacts);
}

int box_plane_contact_manifold(const rigidbody *box_rb, Vector3 box_size, Vector3 plane_point, Vector3 plane_normal, collision *contacts, int max_collisions) {
  if (contacts == NULL || max_collisions <= 0) {
    return 0;
  }

  Vector3 normal = normalize(plane_normal);
  Vector3 tangent_seed = (fabsf(normal.x) < 0.9f) ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
  Vector3 tangent = normalize(cross(tangent_seed, normal));
  Vector3 bitangent = cross(normal, tangent);

  Vector3 half = scale(box_size, 0.5f);
  Vector3 local_corners[8] = {
    { half.x,  half.y,  half.z},
    { half.x,  half.y, -half.z},
    { half.x, -half.y,  half.z},
    { half.x, -half.y, -half.z},
    {-half.x,  half.y,  half.z},
    {-half.x,  half.y, -half.z},
    {-half.x, -half.y,  half.z},
    {-half.x, -half.y, -half.z},
  };

  int contact_count = 0;
  const float contact_epsilon = 0.001f;

  for (int i = 0; i < 8 && contact_count < max_collisions; ++i) {
    Vector3 world_corner = add(box_rb->p, rotate(local_corners[i], box_rb->r));
    float distance = dot(sub(world_corner, plane_point), normal);

    if (distance < contact_epsilon) {
      collision *contact = &contacts[contact_count++];
      contact->valid = true;
      contact->normal = negate(normal);
      contact->tangent = tangent;
      contact->bitangent = bitangent;
      contact->depth = fmaxf(0.0f, -distance);

      contact->world_contact_a = world_corner;
      contact->world_contact_b = sub(world_corner, scale(normal, distance));
      contact->local_contact_a = sub(world_corner, box_rb->p);
      contact->local_contact_b = sub(contact->world_contact_b, plane_point);
    }
  }

  return contact_count;
}

int box_box_contact_manifold(const rigidbody *box_a, const rigidbody *box_b, Vector3 size_a, Vector3 size_b, collision *contacts, int max_collisions) {
  return check_collision(SHAPE_BOX, SHAPE_BOX, box_a, box_b, size_a, size_b, contacts, max_collisions);
}


static int cylinder_plane_store_contact(Vector3 point_on_cylinder, Vector3 normal, Vector3 plane_point, Vector3 tangent, Vector3 bitangent, const rigidbody *cylinder_rb, collision *contacts, int max_contacts, int count) {
  if (count >= max_contacts) {
    return count;
  }

  const float contact_epsilon = 0.001f;
  float distance = dot(sub(point_on_cylinder, plane_point), normal);
  if (distance < contact_epsilon) {
    collision *contact = &contacts[count++];
    contact->valid = true;
    contact->normal = normal;
    contact->tangent = tangent;
    contact->bitangent = bitangent;
    contact->depth = fmaxf(0.0f, -distance);
    contact->world_contact_a = point_on_cylinder;
    contact->world_contact_b = sub(point_on_cylinder, scale(normal, distance));
    contact->local_contact_a = sub(point_on_cylinder, cylinder_rb->p);
    contact->local_contact_b = sub(contact->world_contact_b, plane_point);
  }

  return count;
}

int cylinder_plane_contact_manifold(const rigidbody *cylinder_rb, float cylinder_height, float cylinder_radius, Vector3 plane_point, Vector3 plane_normal, collision *contacts, int max_contacts) {
  if (max_contacts <= 0 || contacts == NULL) {
    return 0;
  }

  Vector3 normal = normalize(plane_normal);
  Vector3 tangent_seed = (fabsf(normal.x) < 0.9f) ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
  Vector3 tangent = normalize(cross(tangent_seed, normal));
  Vector3 bitangent = cross(normal, tangent);

  Vector3 cylinder_axis = rotate(up(), cylinder_rb->r);
  float half_height = cylinder_height * 0.5f;

  Vector3 cap_top = add(cylinder_rb->p, scale(cylinder_axis, half_height));
  Vector3 cap_bottom = add(cylinder_rb->p, scale(cylinder_axis, -half_height));

  float dist_top = dot(sub(cap_top, plane_point), normal);
  float dist_bottom = dot(sub(cap_bottom, plane_point), normal);

  float closest_cap_dist = fminf(dist_top, dist_bottom);
  Vector3 closest_cap_center = (dist_top < dist_bottom) ? cap_top : cap_bottom;

  float axis_projection = fabsf(dot(cylinder_axis, normal));
  const float perpendicular_threshold = 0.001f;

  Vector3 closest_point;
  float min_distance;

  if (axis_projection < perpendicular_threshold) {
    Vector3 normal_radial = sub(normal, scale(cylinder_axis, dot(normal, cylinder_axis)));
    float radial_length = len(normal_radial);

    if (radial_length > perpendicular_threshold) {
      Vector3 radial_dir = scale(normal_radial, -1.0f / radial_length);
      closest_point = add(cylinder_rb->p, scale(radial_dir, cylinder_radius));
      min_distance = dot(sub(closest_point, plane_point), normal);
    } else {
      closest_point = closest_cap_center;
      min_distance = closest_cap_dist;
    }
  } else {
    Vector3 normal_on_cap = sub(normal, scale(cylinder_axis, dot(normal, cylinder_axis)));
    float normal_on_cap_length = len(normal_on_cap);

    if (normal_on_cap_length > perpendicular_threshold) {
      Vector3 radial_dir = scale(normal_on_cap, -1.0f / normal_on_cap_length);
      closest_point = add(closest_cap_center, scale(radial_dir, cylinder_radius));
    } else {
      closest_point = closest_cap_center;
    }

    min_distance = dot(sub(closest_point, plane_point), normal);
  }

  int contact_count = 0;
  const float contact_epsilon = 0.001f;
  if (min_distance >= contact_epsilon) {
    return 0;
  }

  if (axis_projection < perpendicular_threshold) {
    Vector3 normal_radial = sub(normal, scale(cylinder_axis, dot(normal, cylinder_axis)));
    float radial_length = len(normal_radial);
    if (radial_length > perpendicular_threshold) {
      Vector3 radial_dir = scale(normal_radial, -1.0f / radial_length);
      Vector3 mid_point = add(cylinder_rb->p, scale(radial_dir, cylinder_radius));

      Vector3 offsets[] = {
        zero(),
        scale(cylinder_axis, half_height),
        scale(cylinder_axis, -half_height),
        scale(cylinder_axis, half_height * 0.5f),
        scale(cylinder_axis, -half_height * 0.5f)
      };

      int num_offsets = sizeof(offsets) / sizeof(offsets[0]);
      for (int i = 0; i < num_offsets && contact_count < max_contacts; ++i) {
        Vector3 sample_point = add(mid_point, offsets[i]);
        contact_count = cylinder_plane_store_contact(sample_point, normal, plane_point, tangent, bitangent, cylinder_rb, contacts, max_contacts, contact_count);
      }
    }
  } else {
    Vector3 axis_hint = (fabsf(cylinder_axis.y) < 0.999f) ? up() : right();
    Vector3 cap_u = cross(axis_hint, cylinder_axis);
    float cap_u_len = len(cap_u);
    if (cap_u_len < 0.001f) {
      axis_hint = forward();
      cap_u = cross(axis_hint, cylinder_axis);
      cap_u_len = len(cap_u);
    }
    cap_u = scale(cap_u, 1.0f / cap_u_len);
    Vector3 cap_v = cross(cylinder_axis, cap_u);
    cap_v = normalize(cap_v);

    Vector3 rim_dirs[9];
    int rim_count = 0;
    rim_dirs[rim_count++] = zero();
    rim_dirs[rim_count++] = cap_u;
    rim_dirs[rim_count++] = negate(cap_u);
    rim_dirs[rim_count++] = cap_v;
    rim_dirs[rim_count++] = negate(cap_v);

    Vector3 diag1 = normalize(add(cap_u, cap_v));
    Vector3 diag2 = normalize(sub(cap_u, cap_v));
    rim_dirs[rim_count++] = diag1;
    rim_dirs[rim_count++] = negate(diag1);
    rim_dirs[rim_count++] = diag2;
    rim_dirs[rim_count++] = negate(diag2);

    for (int i = 0; i < rim_count && contact_count < max_contacts; ++i) {
      Vector3 offset = scale(rim_dirs[i], cylinder_radius);
      Vector3 sample_point = add(closest_cap_center, offset);
      contact_count = cylinder_plane_store_contact(sample_point, normal, plane_point, tangent, bitangent, cylinder_rb, contacts, max_contacts, contact_count);
    }
  }

  if (contact_count == 0) {
    contact_count = cylinder_plane_store_contact(closest_point, normal, plane_point, tangent, bitangent, cylinder_rb, contacts, max_contacts, contact_count);
  }

  return contact_count;
}

collision sphere_plane_check_collision(const rigidbody *sphere_rb, float radius, Vector3 plane_point, Vector3 plane_normal) {
  collision result = {0};

  Vector3 normal = normalize(plane_normal);

  Vector3 center_to_plane = sub(sphere_rb->p, plane_point);
  float distance = dot(center_to_plane, normal);

  if (distance >= 0.0f && distance < radius) {
    result.valid = true;
    result.depth = radius - distance;
    result.normal = normal;

    Vector3 tangent_seed = (fabsf(normal.x) < 0.9f) ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
    result.tangent = normalize(cross(tangent_seed, normal));
    result.bitangent = cross(normal, result.tangent);

    result.local_contact_a = scale(normal, -radius);
    result.local_contact_b = scale(normal, -result.depth);

    result.world_contact_a = add(sphere_rb->p, result.local_contact_a);
    result.world_contact_b = add(plane_point, result.local_contact_b);
  }

  return result;
}

constraints constraints_new(int num_bodies, int num_constraints, int num_dof, float stabilization, int gauss_seidel_iterations) {
  constraints c;
  c.beta = stabilization;
  c.num_bodies = num_bodies;
  c.num_constraints = num_constraints;
  c.num_dof = num_dof;
  c.gauss_seidel_iterations = gauss_seidel_iterations;

  c.errors = (float*) malloc(num_constraints * sizeof(float));
  c.j = (float*) malloc(num_constraints * num_bodies * num_dof * sizeof(float));
  c.inv_m = (float*) malloc(num_dof * num_bodies * sizeof(float));
  c.v = (float*) malloc(num_bodies * num_dof * sizeof(float));

  c.a = (float*) malloc(num_constraints * num_constraints * sizeof(float));
  c.b = (float*) malloc(num_constraints * sizeof(float));
  c.lambda = (float*) malloc(num_constraints * sizeof(float));

  c.dv = (float*) malloc(num_constraints * num_dof * sizeof(float));

  return c;
}

static void gauss_seidel_solve(float* a, float* b, float* solution, int num_dimensions, int max_iterations) {
  memset(solution, 0, num_dimensions * sizeof(float));
   
  float max_delta = 0.0;
  for (int iter = 0; iter < max_iterations; iter++) {
    max_delta = 0.0;
    for (int i = 0; i < num_dimensions; i++) {
      float sigma = 0.0;
      
      for (int j = 0; j < i; j++) {
        sigma += a[i * num_dimensions + j] * solution[j];
      }
      
      for (int j = i + 1; j < num_dimensions; j++) {
        sigma += a[i * num_dimensions + j] * solution[j];
      }
      
      float x_new = (b[i] - sigma) / a[i * num_dimensions + i];
      float delta = fabs(x_new - solution[i]);
      if (delta > max_delta) {
        max_delta = delta;
      }
      
      solution[i] = x_new;
    }
    
    if (max_delta < SOLVER_TOLERANCE) {
        return;
    }
  }
}

void constraints_solve(constraints *c, float dt) {
  int row_size = c->num_bodies * c->num_dof;
  int nc = c->num_constraints;

  for (int i = 0; i < nc; i++) {
    for (int j = 0; j < nc; ++j) {
      float aij = 0;
      for (int k = 0; k < row_size; ++k) {
        aij += c->j[i * row_size + k] * c->inv_m[k] * c->j[j * row_size + k];
      }
      c->a[i * nc + j] = aij;
    }
  }

  float inv_t = 1.0 / dt;
  for (int i = 0; i < nc; ++i) {
    float bi = 0;
    for (int j = 0; j < row_size; ++j) {
      bi += c->j[i * row_size + j] * c->v[j];
    }

    c->b[i] = -(bi + c->beta * c->errors[i] * inv_t);
  }

  gauss_seidel_solve(c->a, c->b, c->lambda, nc, c->gauss_seidel_iterations);

  for (int i = 0; i < row_size; ++i) {
    float jt_lambda = 0;
    for (int j = 0; j < nc; ++j) {
      jt_lambda += c->j[j * row_size + i] * c->lambda[j];
    }
    c->dv[i] = jt_lambda * c->inv_m[i];
  }
}

void constraints_free(constraints c) {
  free(c.errors);
  free(c.j);
  free(c.inv_m);
  free(c.a);
  free(c.b);
  free(c.lambda);
  free(c.v);
  free(c.dv);
}

oscillation_period oscillation_period_new() {
  return (oscillation_period) { .timestamp = GetTime() };
}

void oscillation_period_track(oscillation_period* period, const rigidbody* current, const rigidbody* prev) {
  float prev_velocity = prev->v.x;
  float current_velocity = current->v.x;

  if (prev_velocity * current_velocity < 0) {
    period->num_turns += 1;
  }

  float num_oscillations = 0.5f * period->num_turns;
  float time_passed = GetTime() - period->timestamp;

  if (num_oscillations > 0) {
    period->period = time_passed / num_oscillations;
  }
}
