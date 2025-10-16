#include "raylib.h"
#include "physics.h"

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

void gauss_seidel_solve(float* a, float* b, float* solution, int num_dimensions) {
  for (int i = 0; i < num_dimensions; i++) {
    solution[i] = 0.0;
  }
   
  // Iterative solver
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
