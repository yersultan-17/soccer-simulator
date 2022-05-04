//
// Created by Yersultan Sapar on 5/3/22.
//

#ifndef SOCCERSIM_BALLMESH_H
#define SOCCERSIM_BALLMESH_H

#include <vector>

#include "CGL/CGL.h"
#include "clothMesh.h"
#include "pointMass.h"

using namespace CGL;
using namespace std;

class BallMesh {
public:
    ~BallMesh() {}

    vector<Triangle *> triangles;
}; // struct BallMesh

#endif //SOCCERSIM_BALLMESH_H

