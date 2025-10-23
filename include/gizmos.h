#ifndef GIZMOS_H
#define GIZMOS_H

#include "raymath.h"

int register_gizmo(Vector3 *pos, Quaternion *rot); 
void unregister_gizmo(int id);

#endif
