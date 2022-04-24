//
// Created by Yersultan Sapar on 4/7/22.
//

#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "box.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

bool inRange(const Vector3D& lower, const Vector3D& upper, const Vector3D& target) {
  bool xRange = (lower.x < target.x && target.x < upper.x);
  bool yRange = (lower.y < target.y && target.y < upper.y);
  bool zRange = (lower.z < target.z && target.z < upper.z);
  return int(zRange) + int(yRange) + int(xRange) == 3;
}

void Box::collide(PointMass &pm) {
  if (!inRange(minCoord, maxCoord, pm.position)) return;
  double surfaceDist = min(min(min(fabs(pm.position.x - minCoord.x), fabs(pm.position.x - maxCoord.x)),
                               min(fabs(pm.position.y - minCoord.y), fabs(pm.position.y - maxCoord.y))),
                           min(fabs(pm.position.z - minCoord.z), fabs(pm.position.z - maxCoord.z)));
  Vector3D surfaceDir = pm.last_position - center;
  Vector3D tangentPoint = center + surfaceDir.unit() * (surfaceDist + surfaceDir.norm());
  Vector3D correction = tangentPoint - pm.last_position;
  pm.position = pm.last_position + correction * (1 - friction);
}

void Box::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);
  MatrixXf positions(3, 36);
  MatrixXf normals(3, 36);

  // Top and bottom
  Vector3f normalTop = Vector3f(0, 1, 0);
  positions.col(0) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(1) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z);
  positions.col(2) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z + side);
  positions.col(3) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(4) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z + side);
  positions.col(5) << Vector3f(minCoord.x, minCoord.y, minCoord.z + side);
  positions.col(6) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z);
  positions.col(7) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z);
  positions.col(8) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(9) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z);
  positions.col(10) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(11) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z + side);
  for (int i = 0; i < 6; ++i) normals.col(i) << -normalTop;
  for (int i = 6; i < 12; ++i) normals.col(i) << normalTop;

  // front and back
  Vector3f normal2 = Vector3f(0, 0, 1);
  positions.col(12) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(13) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z);
  positions.col(14) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z);
  positions.col(15) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(16) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z);
  positions.col(17) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z);
  positions.col(18) << Vector3f(minCoord.x, minCoord.y, minCoord.z + side);
  positions.col(19) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z + side);
  positions.col(20) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(21) << Vector3f(minCoord.x, minCoord.y, minCoord.z + side);
  positions.col(22) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(23) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z + side);
  for (int i = 12; i < 18; ++i) normals.col(i) << normal2;
  for (int i = 18; i < 24; ++i) normals.col(i) << -normal2;

  // Left and right
  Vector3f normal3 = Vector3f(1, 0, 0);
  positions.col(24) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(25) << Vector3f(minCoord.x, minCoord.y, minCoord.z + side);
  positions.col(26) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z + side);
  positions.col(27) << Vector3f(minCoord.x, minCoord.y, minCoord.z);
  positions.col(28) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z + side);
  positions.col(29) << Vector3f(minCoord.x, minCoord.y + side, minCoord.z);
  positions.col(30) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z);
  positions.col(31) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z + side);
  positions.col(32) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(33) << Vector3f(minCoord.x + side, minCoord.y, minCoord.z);
  positions.col(34) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z + side);
  positions.col(35) << Vector3f(minCoord.x + side, minCoord.y + side, minCoord.z);
  for (int i = 24; i < 30; ++i) normals.col(i) << normal3;
  for (int i = 30; i < 36; ++i) normals.col(i) << -normal3;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }
  shader.drawArray(GL_TRIANGLES, 0, 36);
}
