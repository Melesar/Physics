#include "raylib.h"
#include "physics.h"
#include "raymath.h"
#include "stdlib.h"
#include "string.h"

#define SOLVER_TOLERANCE 0.01

Vector3 cylinder_inertia_tensor(cylinder c, float mass) {
  float principal =  mass * (3 * c.radius * c.radius + c.height * c.height) / 12.0;
  return (Vector3){ principal, mass * c.radius * c.radius / 2.0, principal };
}

Vector3 sphere_inertia_tensor(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;  
  return Vector3Scale(Vector3One(), scale);
}

rigidbody rb_new(Vector3 position, float mass) {
  return (rigidbody) {
    .f = Vector3Zero(),
    .fi = Vector3Zero(),
    .v = Vector3Zero(),
    .p = position,

    .t = Vector3Zero(),
    .ti = Vector3Zero(),
    .r = QuaternionIdentity(),
    .i0_inv = Vector3One(),
    .l = Vector3Zero(),

    .mass = mass,
  };
}

Matrix rb_transformation(const rigidbody* rb) {
  return MatrixMultiply(
    QuaternionToMatrix(rb->r),
    MatrixTranslate(rb->p.x, rb->p.y, rb->p.z)
  );
}

Matrix rb_transformation_with_offset(const rigidbody *rb, Vector3 offset) {
  return MatrixMultiply(
    MatrixMultiply(
      MatrixTranslate(rb->p.x, rb->p.y, rb->p.z),
      QuaternionToMatrix(rb->r)),
        MatrixTranslate(offset.x, offset.y, offset.z));
}

static Matrix rb_inv_i0_m(const rigidbody* rb) {
  Matrix inv_i0 = { 0 };
  inv_i0.m0 = rb->i0_inv.x;
  inv_i0.m5 = rb->i0_inv.y;
  inv_i0.m10 = rb->i0_inv.z;
  return inv_i0;
}

static Vector3 rb_angular_velocity_ex(const rigidbody* rb, Matrix inv_i0) {
  Matrix orientation = QuaternionToMatrix(rb->r);
  Matrix transform = MatrixMultiply(MatrixMultiply(orientation, inv_i0), MatrixTranspose(orientation));
  return Vector3Transform(rb->l, transform);
}

static Matrix rb_inertia_world_ex(const rigidbody* rb, Matrix orientation, Matrix inv_i0) {
  return MatrixMultiply(MatrixMultiply(orientation, inv_i0), MatrixTranspose(orientation));
}

Matrix rb_inertia_world(const rigidbody* rb) {
  Matrix orientation = QuaternionToMatrix(rb->r);
  Matrix inv_i0 = rb_inv_i0_m(rb);

  return rb_inertia_world_ex(rb, orientation, inv_i0);
}

Vector3 rb_angular_velocity(const rigidbody* rb) {
  Matrix inv_i0 = rb_inv_i0_m(rb);
  return rb_angular_velocity_ex(rb, inv_i0);
}

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t) {
  rigidbody result;
  result.p = Vector3Lerp(from->p, to->p, t);
  result.r = QuaternionSlerp(from->r, to->r, t);
  result.v = Vector3Lerp(from->v, to->v, t);
  result.mass = Lerp(from->mass, to->mass, t);

  return result;
}

void rb_apply_impulse_at(rigidbody* rb, Vector3 at, Vector3 impulse) {
  Vector3 r = Vector3Subtract(at, rb->p); 
  rb->ti = Vector3Add(rb->ti, Vector3CrossProduct(r, impulse));
  rb->fi = Vector3Add(rb->fi, impulse);
}

void rb_apply_force_at(rigidbody* rb, Vector3 at, Vector3 force) {
  Vector3 r = Vector3Subtract(at, rb->p); 
  rb->t = Vector3Add(rb->t, Vector3CrossProduct(r, force));
  rb->f = Vector3Add(rb->f, force);
}

void rb_apply_impulse(rigidbody* rb, Vector3 impulse) {
  rb->fi = Vector3Add(rb->fi, impulse);
}

void rb_apply_force(rigidbody* rb, Vector3 force) {
  rb->f = Vector3Add(rb->f, force);
}

void rb_simulate(rigidbody* rb, float dt) {
  rb->l = Vector3Add(rb->l, rb->ti);
  rb->l = Vector3Add(rb->l, Vector3Scale(rb->t, dt));

  Matrix inv_i0 = rb_inv_i0_m(rb);
  Vector3 omega = rb_angular_velocity_ex(rb, inv_i0);
  Quaternion q_omega = { omega.x, omega.y, omega.z, 0 };
  Quaternion dq = QuaternionScale(QuaternionMultiply(q_omega, rb->r), 0.5 * dt);

  Quaternion q_orientation = QuaternionAdd(rb->r, dq);
  rb->r = QuaternionNormalize(q_orientation);
  rb->t = rb->ti = Vector3Zero();

  float inv_mass = 1.0 / rb->mass;
  Vector3 acc = Vector3Scale(Vector3Add(rb->fi, Vector3Scale(rb->f, dt)), inv_mass);
  rb->v = Vector3Add(rb->v, acc);
  rb->p = Vector3Add(rb->p, Vector3Scale(rb->v, dt));
  rb->f = rb->fi = Vector3Zero();
}

Vector3 sphere_support(Vector3 center, float radius, Vector3 direction) {
  return Vector3Add(center, Vector3Scale(direction, radius));
}

Vector3 cylinder_support(Vector3 center, float radius, float height, Quaternion rotation, Vector3 direction) {
  Vector3 axis = { 0, 1, 0 };
  float half_height = 0.5f * height;
  float hh = half_height * half_height;
  float a = Vector3DotProduct(axis, direction);
  float b = radius * radius + half_height * half_height;
  float h = b * a;
  Vector3 proj = Vector3Normalize((Vector3){ direction.x, 0, direction.z });

  if (h <= hh && h >= -hh) {
    return Vector3Add(center, Vector3Scale(proj, radius));
  }

  float d = half_height * tanf(a);
  return Vector3Add(center, Vector3Scale(direction, d));
}

collision cylinder_sphere_check_collision(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius) {
  collision result = { 0 };


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
