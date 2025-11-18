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
} shape_type;

Vector3 cylinder_inertia_tensor(cylinder c, float mass) {
  float principal =  mass * (3 * c.radius * c.radius + c.height * c.height) / 12.0;
  return (Vector3){ principal, mass * c.radius * c.radius / 2.0, principal };
}

Vector3 sphere_inertia_tensor(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;  
  return scale(one(), scale);
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

Vector3 single_shape_support(shape_type type, const rigidbody *rb, Vector2 params, Vector3 direction) {
  switch(type) {
    case SHAPE_CYLINDER:
      return cylinder_support(rb->p, params.y, params.x, rb->r, direction);

    case SHAPE_SPHERE:
      return sphere_support(rb->p, params.x, direction);
  }
}

Vector3 shapes_support(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b, Vector3 direction) {
  Vector3 support_a = single_shape_support(shape_a, rb_a, params_a, direction);
  Vector3 support_b = single_shape_support(shape_b, rb_b, params_b, negate(direction));

  return sub(support_a, support_b);
}

static void face_normals(Vector4 *normals, int *num_normals, int *min_face_index, Vector3 *points, int num_points, int *indices, int num_indices) {
  float min_distance = INFINITY;

  for (int i = 0; i < num_indices; i += 3) {
    Vector3 a = points[indices[i]];
    Vector3 b = points[indices[i + 1]];
    Vector3 c = points[indices[i + 2]];
    
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

static collision epa(Vector3 *points, int num_points, shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b) {
  collision result = {0};
  result.valid = true;

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

    Vector3 support = shapes_support(shape_a, shape_b, rb_a, rb_b, params_a, params_b, min_normal);
    float distance = dot(min_normal, support);
    if (fabsf(distance - min_distance) > EPA_TOLERANCE) {
      min_distance = INFINITY;
    } else {
      break;
    }

    int num_unique_edges = 0;
    int unique_edges[COLLISION_MAX_INDICES] = {0};
    for (int i = 0; i < num_normals; ++i) {
      Vector3 normal = *(Vector3*)&normals[i];
      if (dot(normal, support) - normals[i].w <= EPA_TOLERANCE) {
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
      return (collision) {0};
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
      return (collision) { 0 };
    }

    if (num_normals + num_new_normals > COLLISION_MAX_FACES) {
      TraceLog(LOG_ERROR, "EPA: Maximum number of faces exceeded: %d", num_normals + num_new_normals);
      return (collision) { 0 };
    }

    memcpy(indices + num_indices, new_faces, num_new_faces * sizeof(int));
    memcpy(normals + num_normals, new_normals, num_new_normals * sizeof(Vector4));

    num_indices += num_new_faces;
    num_normals += num_new_normals;
  }

  Vector3 t1, t2;
  if (min_normal.x >= 0.57735f) {
    t1.x = min_normal.y;
    t1.y = -min_normal.x;
    t1.z = 0;
  } else {
    t1.x = 0;
    t1.y = min_normal.z;
    t1.z = -min_normal.y;
  }

  Vector3 support_a = single_shape_support(shape_a, rb_a, params_a, min_normal);
  Vector3 support_b = single_shape_support(shape_b, rb_b, params_b, negate(min_normal));

  result.world_contact_a = add(support_a, scale(min_normal, -min_distance * 0.5f));
  result.world_contact_b = add(support_b, scale(min_normal, min_distance * 0.5f));

  result.local_contact_a = sub(result.world_contact_a, rb_a->p);
  result.local_contact_b = sub(result.world_contact_b, rb_b->p);
  
  result.depth = min_distance + 0.001f;
  result.normal = min_normal;
  result.tangent = normalize(t1);
  result.bitangent = cross(min_normal, result.tangent);

  return result;
}

static bool gjk_update_simplex(Vector3 *points, int *count, Vector3 *direction) {
  Vector3 a, b, c, d;
  Vector3 ab, ac, ad, ao;
  Vector3 abc, acd, adb;

  switch(*count) {
    case 4:
      a = points[0];
      b = points[1];
      c = points[2];
      d = points[3];

      ab = sub(b, a);
      ac = sub(c, a);
      ad = sub(d, a);
      ao = negate(a);

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

      ab = sub(b, a);
      ac = sub(c, a);
      ao = negate(a);

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
      ab = sub(points[1], points[0]);
      ao = negate(points[0]);

      if (dot(ab, ao) > 0) {
        *direction = normalize(cross(cross(ab, ao), ab));
      } else {
        *count = 1;
        *direction = normalize(ao); 
      }
      return false;

    case 1:
      *direction = normalize(negate(points[0]));
      return false;
  }

  return false;
}

collision check_collision(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b) {
  Vector3 direction = right();

  int num_points = 0;
  Vector3 points[COLLISION_MAX_POINTS];

  int num_attempts = 0;
  while(++num_attempts < MAX_GJK_ATTEMPTS) {
    Vector3 support = shapes_support(shape_a, shape_b, rb_a, rb_b, params_a, params_b, direction);

    if (dot(support, direction) < GJK_TOLERANCE) {
      return (collision) { 0 };
    }

    points[3] = points[2];
    points[2] = points[1];
    points[1] = points[0];
    points[0] = support;

    num_points += 1;

    if (gjk_update_simplex(points, &num_points, &direction)) {
      return epa(points, num_points, shape_a, shape_b, rb_a, rb_b, params_a, params_b);
    }
  }

  TraceLog(LOG_WARNING, "Max GJK attempts reached withot the result. Collision detection failed");
  return (collision) { 0 };
}

collision cylinder_sphere_check_collision(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius) {
  return check_collision(SHAPE_SPHERE, SHAPE_CYLINDER, sphere_rb, cylinder_rb, (Vector2) { sphere_radius, 0 }, (Vector2) { cylinder_height, cylinder_radius });
}

collision cylinder_plane_check_collision(const rigidbody *cylinder_rb, float cylinder_height, float cylinder_radius, Vector3 plane_point, Vector3 plane_normal) {
  collision result = {0};

  Vector3 normal = normalize(plane_normal);
  Vector3 cylinder_axis = rotate(up(), cylinder_rb->r);

  float half_height = cylinder_height * 0.5f;

  Vector3 cap_top = add(cylinder_rb->p, scale(cylinder_axis, half_height));
  Vector3 cap_bottom = add(cylinder_rb->p, scale(cylinder_axis, -half_height));

  float dist_top = dot(sub(cap_top, plane_point), normal);
  float dist_bottom = dot(sub(cap_bottom, plane_point), normal);

  float closest_cap_dist = fminf(dist_top, dist_bottom);
  Vector3 closest_cap_center = (dist_top < dist_bottom) ? cap_top : cap_bottom;

  float axis_projection = fabsf(dot(cylinder_axis, normal));

  Vector3 closest_point;
  float min_distance;

  const float perpendicular_threshold = 0.001f;

  if (axis_projection < perpendicular_threshold) {
    Vector3 normal_radial = sub(normal, scale(cylinder_axis,
                                            dot(normal, cylinder_axis)));
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
    Vector3 normal_on_cap = sub(normal, scale(cylinder_axis,
                                            dot(normal, cylinder_axis)));
    float normal_on_cap_length = len(normal_on_cap);

    if (normal_on_cap_length > perpendicular_threshold) {
      Vector3 radial_dir = scale(normal_on_cap, -1.0f / normal_on_cap_length);
      closest_point = add(closest_cap_center, scale(radial_dir, cylinder_radius));
    } else {
      closest_point = closest_cap_center;
    }

    min_distance = dot(sub(closest_point, plane_point), normal);
  }

  const float contact_epsilon = 0.001f;
  if (min_distance < contact_epsilon) {
    result.valid = true;
    result.normal = normal;
    result.depth = fmaxf(0.0f, -min_distance);

    Vector3 tangent_seed = (fabsf(normal.x) < 0.9f) ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
    result.tangent = normalize(cross(tangent_seed, normal));
    result.bitangent = cross(normal, result.tangent);

    result.world_contact_a = closest_point;
    result.world_contact_b = sub(closest_point, scale(normal, min_distance));

    result.local_contact_a = sub(closest_point, cylinder_rb->p);
    result.local_contact_b = sub(result.world_contact_b, plane_point);
  }

  return result;
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
