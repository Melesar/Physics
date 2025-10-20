#ifndef PHYSICS_H
#define PHYSICS_H

#include "raymath.h"

#define GRAVITY 9.81f
#define GRAVITY_V (Vector3) { 0, -9.81f, 0 }

typedef struct {
    Vector3 position;
    Vector3 linear_velocity;
    Quaternion orientation;

    float mass;
} rigidbody;

typedef struct {
  int num_turns;
  double timestamp; 
  double period;
} oscillation_period;

typedef struct {
  int num_constraints, num_dof, num_bodies;
  float beta;

  float *j;
  float *errors;
  float *inv_m;
  float *v;

  float *jm;
  float *a;
  float *b;
  float *lambda;
  float *jt_lambda;

  float *dv;
 
} constraints;

rigidbody rb_new(Vector3 position, float mass);

Matrix rb_transformation(const rigidbody* rb);

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t);

oscillation_period oscillation_period_new();
void oscillation_period_track(oscillation_period* period, const rigidbody* current, const rigidbody* prev);

constraints* constraints_new(int num_bodies, int num_constraints, int num_dof, float stabilization);
void constraints_solve(constraints *c, float dt);
void constraints_free(constraints *c);

#endif
