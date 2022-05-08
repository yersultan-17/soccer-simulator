#ifndef BALL_H
#define BALL_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "ballMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"

using namespace CGL;
using namespace std;

struct BallParameters {
  BallParameters() {}
  BallParameters(bool enable_structural_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        damping(damping), density(density), ks(ks) {}
  ~BallParameters() {}

  // Global simulation parameters
  bool enable_structural_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Ball {
    Ball();
    //~Ball();
  void buildShape(); // includes building the shape and the mesh.

  void simulate(double frames_per_sec, double simulation_steps, BallParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects,
                float windSpeed, const Vector3D& windDirection);

  void reset();
  void buildBallMesh();
  void setupNormals();
  void collide(PointMass &pm);

  // Cloth components
  vector<PointMass> point_masses;
  vector<Spring> springs;
  BallMesh *ballMesh;

  // Soccer ball components
  vector<vector<double>> vertices;
  vector<vector<int>> edges;
  vector<vector<int>> faces;

  Vector3D centroid;
  double radius;

  // Frame number
  int frame_num;

  // Wind
  double clock;

  bool windOn;
};

#endif /* BALL_H */
