#ifndef PHYSICS_H
#define PHYSICS_H

#include "raylib.h"
#include "stdbool.h"
#include "raymath.h"

#define GRAVITY 9.81f
#define GRAVITY_V (Vector3) { 0, -9.81f, 0 }

// ==== SHAPES =====

typedef struct {
  float height, radius;
} cylinder;

Vector3 cylinder_inertia_tensor(cylinder c, float mass);
Vector3 sphere_inertia_tensor(float radius, float mass);

// ==== RIGIDBODY =====

typedef struct {
  Vector3 f, fi; // Linear force applied to the center of mass
  Vector3 v; // Linear velocity
  Vector3 p; // Position of the center of mass

  Vector3 t, ti; // Torque
  Quaternion r; // Orientation
  Vector3 i0_inv; // Inversed inertia tensor in body space
  Vector3 l; // Moment of inertia

  float mass;
} rigidbody;

rigidbody rb_new(Vector3 position, float mass);

Matrix rb_transformation(const rigidbody* rb);
Matrix rb_transformation_with_offset(const rigidbody *rb, Vector3 offset);
Matrix rb_inertia_world(const rigidbody* rb);
Vector3 rb_angular_velocity(const rigidbody* rb);

void rb_apply_impulse_at(rigidbody* rb, Vector3 at, Vector3 impulse);
void rb_apply_force_at(rigidbody* rb, Vector3 at, Vector3 force);
void rb_apply_impulse(rigidbody* rb, Vector3 impulse);
void rb_apply_force(rigidbody* rb, Vector3 force);

void rb_simulate(rigidbody* r, float dt);

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t);

// ==== COLLISIONS ====

typedef struct {
  Vector3 contact_a, contact_b;
  Vector3 normal;
  float depth;
  bool valid;
} collision;

collision cylinder_sphere_check_collision(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius);

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
