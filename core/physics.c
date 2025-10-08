#include "physics.h"
#include "raymath.h"

rigidbody rb_new(Vector3 position, float mass) {
  return (rigidbody){ position, QuaternionIdentity(), mass };
}

Matrix rb_transformation(const rigidbody* rb) {
    return MatrixMultiply(
      QuaternionToMatrix(rb->orientation),
      MatrixTranslate(rb->position.x, rb->position.y, rb->position.z)
    );
}
