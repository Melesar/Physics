#include "raylib.h"

#define cross(x, y) Vector3CrossProduct(x, y)
#define dot(x, y) Vector3DotProduct(x, y)
#define add(x, y) Vector3Add(x, y)
#define scale(x, y) Vector3Scale(x, y)
#define normalize(x) Vector3Normalize(x)
#define sub(x, y) Vector3Subtract(x, y)
#define len(x) Vector3Length(x)
#define lensq(x) Vector3LengthSqr(x)
#define distance(x, y) Vector3Distance(x, y)
#define distancesqr(x, y) Vector3DistanceSqr(x, y)
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3) { 0, 1, 0 })
#define right() ((Vector3) { 1, 0, 0 })
#define forward() ((Vector3) { 0, 0, 1 })
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)
#define invert(x) Vector3Invert(x)

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
#define inverse(x) MatrixInvert(x)
#define m4identity(x) MatrixIdentity(x)

#define vlerp(x, y, t) Vector3Lerp(x, y, t)
#define lerp(x, y, t) Lerp(x, y, t)
#define slerp(x, y, t) QuaternionSlerp(x, y, t)

typedef Vector3 v3;
typedef Vector4 v4;
typedef Quaternion quat;
typedef Matrix m4;

typedef struct {
  float m0[3];  // Row 0
  float m1[3];  // Row 1
  float m2[3];  // Row 2
} m3;

m3 matrix_transpose(m3 m);
v3 matrix_rotate(v3 v, m3 m);
v3 matrix_rotate_inverse(v3 v, m3 m);
m3 matrix_from_basis(v3 x, v3 y, v3 z);
m3 matrix_skew_symmetric(v3 v);
m4 inertia_tensor_matrix(v3 inertia);
