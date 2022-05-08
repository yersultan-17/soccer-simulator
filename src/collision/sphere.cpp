#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  //if ((pm.position - origin).norm() > radius) return;
  //Vector3D tangentPoint = radius * (pm.position - origin).unit() + origin;
  //Vector3D correction = tangentPoint - pm.last_position;
  //pm.position = pm.last_position + correction * (1 - friction);
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
