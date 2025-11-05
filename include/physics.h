#ifndef PHYSICS_H
#define PHYSICS_H

#include "raylib.h"
#include "stdbool.h"

#define GRAVITY 9.81f
#define GRAVITY_V (Vector3) { 0, -9.81f, 0 }

// ==== MATH  ======
 
#define cross(x, y) Vector3CrossProduct(x, y)
#define dot(x, y) Vector3DotProduct(x, y)
#define add(x, y) Vector3Add(x, y)
#define scale(x, y) Vector3Scale(x, y)
#define normalize(x) Vector3Normalize(x)
#define sub(x, y) Vector3Subtract(x, y)
#define len(x) Vector3Length(x)
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3) { 0, 1, 0 })
#define right() ((Vector3) { 1, 0, 0 })
#define forward() ((Vector3) { 0, 0, 1 })
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)

#define qadd(x, y) QuaternionAdd(x, y)
#define qscale(x, y) QuaternionScale(x, y)
#define qmul(x, y) QuaternionMultiply(x, y)
#define qnormalize(x) QuaternionNormalize(x)
#define qinvert(x) QuaternionInvert(x)
#define as_matrix(x) QuaternionToMatrix(x)
#define qidentity() QuaternionIdentity()

#define mul(x, y) MatrixMultiply(x, y)
#define transpose(x) MatrixTranspose(x)
#define translate(x, y, z) MatrixTranslate(x, y, z)

#define vlerp(x, y, t) Vector3Lerp(x, y, t)
#define lerp(x, y, t) Lerp(x, y, t)
#define slerp(x, y, t) QuaternionSlerp(x, y, t)

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
  Vector3 world_contact_a, world_contact_b;
  Vector3 local_contact_a, local_contact_b;
  Vector3 normal;
  Vector3 tangent, bitangent;
  float depth;
  bool valid;
} collision;

Vector3 sphere_support(Vector3 center, float radius, Vector3 direction);
Vector3 cylinder_support(Vector3 center, float radius, float height, Quaternion rotation, Vector3 direction);

collision cylinder_sphere_check_collision(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius);
collision cylinder_plane_check_collision(const rigidbody *cylinder_rb, float cylinder_height, float cylinder_radius, Vector3 plane_point, Vector3 plane_normal);
collision sphere_plane_check_collision(const rigidbody *shpere_rb, float radius, Vector3 plane_point, Vector3 plane_normal);

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
