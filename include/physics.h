#ifndef PHYSICS_H
#define PHYSICS_H

#include "raymath.h"

typedef struct {
    Vector3 position;
    Quaternion orientation;

    float mass;
} rigidbody;

rigidbody rb_new(Vector3 position, float mass);

Matrix rb_transformation(const rigidbody* rb);

rigidbody rb_interpolate(const rigidbody* from, const rigidbody* to, float t);

#endif
