#include "pmath.h"
#include "raymath.h"

v3 matrix_rotate(v3 v, m3 m) {
  return (v3) {
    dot(*(v3*)m.m0, v),
    dot(*(v3*)m.m1, v),
    dot(*(v3*)m.m2, v)
  };
}

m3 matrix_transpose(m3 m) {
  return (m3) {
    { m.m0[0], m.m1[0], m.m2[0] },
    { m.m0[1], m.m1[1], m.m2[1] },
    { m.m0[2], m.m1[2], m.m2[2] }
  };
}

v3 matrix_rotate_inverse(v3 v, m3 m) {
  return matrix_rotate(v, matrix_transpose(m));
}

m3 matrix_from_basis(v3 x, v3 y, v3 z) {
  return (m3) {
    { x.x, y.x, z.x },
    { x.y, y.y, z.y },
    { x.z, y.z, z.z }
  };
}

m3 matrix_skew_symmetric(v3 v) {
  return (m3) {
    { 0, -v.z, v.y },
    { v.z, 0, -v.x },
    { -v.y, v.x, 0 }
  };
}

m4 inertia_tensor_matrix(v3 inertia) {
  m4 tensor = { 0 };
  tensor.m0 = inertia.x;
  tensor.m5 = inertia.y;
  tensor.m10 = inertia.z;
  return tensor;
}
