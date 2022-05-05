#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
  double perpDist = dot(pm.position - point, normal);
  double oldPerpDist = dot(pm.last_position - point, normal);
  if (perpDist * oldPerpDist > 0) return;
  Vector3D tangentPoint = (pm.position - perpDist * normal);
  if (oldPerpDist < 0) tangentPoint -= normal * SURFACE_OFFSET;
  if (oldPerpDist > 0) tangentPoint += normal * SURFACE_OFFSET;
  Vector3D correction = tangentPoint - pm.last_position;
  Vector3D cur_position = pm.position;
  pm.position = pm.last_position + correction * (1 - friction);
  pm.last_position = cur_position;
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 50 * (sCross + sParallel);
  positions.col(1) << sPoint + 50 * (sCross - sParallel);
  positions.col(2) << sPoint + 50 * (-sCross + sParallel);
  positions.col(3) << sPoint + 50 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
