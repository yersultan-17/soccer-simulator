//
// Created by Yersultan Sapar on 4/7/22.
//

#ifndef COLLISIONOBJECT_BOX_H
#define COLLISIONOBJECT_BOX_H

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Box : public CollisionObject {
public:
    Box(const Vector3D &center, double side, double friction)
      : center(center), side(side), friction(friction) {
      surfaceNormals = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, 1}};
      minCoord = center - side / 2;
      maxCoord = center + side / 2;
    }

    void render(GLShader &shader);
    void collide(PointMass &pm);

    Vector3D center;
    double side;

    double friction;
    vector<Vector3D> surfaceNormals;
    Vector3D minCoord;
    Vector3D maxCoord;
};

#endif //COLLISIONOBJECT_BOX_H
