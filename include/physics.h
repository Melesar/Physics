#ifndef PHYSICS_H
#define PHYSICS_H

#include "raymath.h"

#define GRAVITY 9.81f
#define GRAVITY_V (Vector3) { 0, -9.81f, 0 }

// ==== SHAPES =====

typedef struct {
  float height, radius;
} cylinder;

Vector3 cylinder_inertia_tensor(cylinder c, float mass);

// ==== RIGIDBODY =====

typedef struct {
    Vector3 p;
    Vector3 v;
    Quaternion r;
    Vector3 i0_inv;
    Vector3 l;

    float mass;
} rigidbody;

rigidbody rb_new(Vector3 position, float mass);
Matrix rb_transformation(const rigidbody* rb);
Matrix rb_transformation_with_offset(const rigidbody *rb, Vector3 offset);
rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t);

// ==== CONSTRAINTS ====

typedef struct {
  int num_constraints, num_dof, num_bodies;
  int gauss_seidel_iterations;

  float beta;

  float *j;
  float *errors;
  float *inv_m;
  float *v;

  float *a;
  float *b;
  float *lambda;

  float *dv;
 
} constraints;

constraints constraints_new(int num_bodies, int num_constraints, int num_dof, float stabilization, int gauss_seidel_iterations);
void constraints_solve(constraints *c, float dt);
void constraints_free(constraints c);

// ==== MISC ====

typedef struct {
  int num_turns;
  double timestamp; 
  double period;
} oscillation_period;

oscillation_period oscillation_period_new();
void oscillation_period_track(oscillation_period* period, const rigidbody* current, const rigidbody* prev);

#endif
