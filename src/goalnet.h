//
// Created by Yersultan Sapar on 5/3/22.
//

#ifndef SOCCERSIM_GOALNET_H
#define SOCCERSIM_GOALNET_H

#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "clothMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"
#include "ball.h"

using namespace CGL;
using namespace std;

struct GoalnetParameters {
    GoalnetParameters() {}
    GoalnetParameters(double damping,
                    double density, double ks) :
              damping(damping), density(density), ks(ks) {}
    ~GoalnetParameters() {}
    double damping;

    // Mass-spring parameters
    double density;
    double ks;
};

struct Goalnet {
    Goalnet() {}
    Goalnet(double width, double height, int num_width_points,
            int num_height_points, float thickness);
    ~Goalnet();

    void buildGrid();

    void simulate(double frames_per_sec, double simulation_steps, GoalnetParameters *gnp,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects,
                  Ball *ball,
                  float windSpeed, Vector3D& windDirection);

    void reset();
    void buildGoalnetMesh();

    // Goalnet properties
    double width;
    double height;
    int num_width_points;
    int num_height_points;
    double thickness;

    // Goalnet components
    vector<PointMass> point_masses;
    vector<vector<int>> pinned;
    vector<Spring> springs;
    ClothMesh *goalnetMesh;
};

#endif //SOCCERSIM_GOALNET_H
