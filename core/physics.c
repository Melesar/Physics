#include "raylib.h"
#include "physics.h"
#include "stdlib.h"
#include "string.h"

#define MAX_SOLVER_ITERATIONS 5
#define SOLVER_TOLERANCE 0.01

rigidbody rb_new(Vector3 position, float mass) {
  return (rigidbody){ position, (Vector3){0}, QuaternionIdentity(), mass };
}

Matrix rb_transformation(const rigidbody* rb) {
  return MatrixMultiply(
    QuaternionToMatrix(rb->orientation),
    MatrixTranslate(rb->position.x, rb->position.y, rb->position.z)
  );
}

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t) {
  rigidbody result;
  result.position = Vector3Lerp(from->position, to->position, t);
  result.orientation = QuaternionSlerp(from->orientation, to->orientation, t);
  result.linear_velocity = Vector3Lerp(from->linear_velocity, to->linear_velocity, t);
  result.mass = Lerp(from->mass, to->mass, t);

  return result;
}

oscillation_period oscillation_period_new() {
  return (oscillation_period) { .timestamp = GetTime() };
}

void oscillation_period_track(oscillation_period* period, const rigidbody* current, const rigidbody* prev) {
  float prev_velocity = prev->linear_velocity.x;
  float current_velocity = current->linear_velocity.x;

  if (prev_velocity * current_velocity < 0) {
    period->num_turns += 1;
  }

  float num_oscillations = 0.5f * period->num_turns;
  float time_passed = GetTime() - period->timestamp;

  if (num_oscillations > 0) {
    period->period = time_passed / num_oscillations;
  }
}

constraints constraints_new(int num_bodies, int num_constraints, int num_dof, float stabilization) {
  constraints c;
  c.beta = stabilization;
  c.num_bodies = num_bodies;
  c.num_constraints = num_constraints;
  c.num_dof = num_dof;

  c.errors = (float*) malloc(num_constraints * sizeof(float));
  c.j = (float*) malloc(num_constraints * num_bodies * num_dof * sizeof(float));
  c.inv_m = (float*) malloc(num_dof * num_bodies * sizeof(float));

  c.a = (float*) malloc(num_constraints * num_constraints * sizeof(float));
  c.b = (float*) malloc(num_constraints * sizeof(float));
  c.lambda = (float*) malloc(num_constraints * sizeof(float));
  c.v = (float*) malloc(num_bodies * num_dof * sizeof(float));

  c.dv = (float*) malloc(num_constraints * num_dof * sizeof(float));

  return c;
}

static void gauss_seidel_solve(float* a, float* b, float* solution, int num_dimensions) {
  memset(solution, 0, num_dimensions * sizeof(float));
   
  for (int iter = 0; iter < MAX_SOLVER_ITERATIONS; iter++) {
    float max_delta = 0.0;
    
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
        // TODO reintroduce jm matrix and use it here
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

  gauss_seidel_solve(c->a, c->b, c->lambda, nc);

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



