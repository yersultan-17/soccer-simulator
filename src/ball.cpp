#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <numeric>

#include "ball.h"
#include "ballMesh.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

const double PHI = (1 + sqrt(5)) / 2;
const double INTERNAL_PRESSURE = 20000.0;

Ball::Ball() {
    frame_num = 0;
    windOn = true;
    buildShape();
    buildBallMesh();
}

//Ball::~Ball() {
//  point_masses.clear();
//  springs.clear();
//
//  if (ballMesh) {
//    delete ballMesh;
//  }
//}

void Ball::buildShape() {
  // Build a truncated icosahedron of masses and springs.
  for (auto vertex: vertices) {
      double scale = 0.7;
      point_masses.emplace_back(scale * Vector3D(vertex[0], vertex[1], vertex[2]), false);
  }
  // Now, let's set up the springs
  for (auto edge: edges) {
      PointMass *pm1 = &point_masses[edge[0]];
      PointMass *pm2 = &point_masses[edge[1]];
      springs.emplace_back(pm1, pm2, STRUCTURAL);
  }
}

void Ball::collide(PointMass &pm) {
    if ((pm.position - centroid).norm() > radius) return;
    windOn = false;
    Vector3D tangentPoint = radius * (pm.position - centroid).unit() + centroid;
    Vector3D correction = tangentPoint - pm.last_position;
    pm.position = pm.last_position + correction * 1.8;
    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *pm = it.operator->();
        pm->position -= correction * 0.06;
    }
}

void Ball::simulate(double frames_per_sec, double simulation_steps, BallParameters *bp,
                     vector<Vector3D> external_accelerations,
                    vector<CollisionObject *> *collision_objects,
                    float windSpeed, const Vector3D& windDirection) {
    double mass = bp->density / 60.f;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;
    windSpeed = 500.0;
    Vector3D windDir = Vector3D(1, 0, 0);

    // Wind EC: keeping track of global c`lock of wind that resets periodically
    clock += delta_t;
    if (clock >= PI / 10) clock = 0;

    // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D totalExtAcceleration = accumulate(external_accelerations.begin(), external_accelerations.end(),
                                               Vector3D());
    Vector3D totalExtForce = totalExtAcceleration * mass;

    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *pm = it.operator->();
        pm->forces = Vector3D();
        pm->forces += Vector3D(totalExtForce);
        // Integrate pressure inside the soccer bal
        Vector3D dir = pm->position - centroid;
        pm->forces += (INTERNAL_PRESSURE * dir.unit()) / dir.norm();
        if (windOn) {
            double cos_theta_wind = std::max(0.0, dot((pm->position - centroid), -windDir));
            Vector3D windForce = cos_theta_wind * windSpeed * windDir;
            pm->forces += windForce;
        }
    }

    for (auto it = springs.begin(); it != springs.end(); ++it) {
        Spring *s = it.operator->();
        PointMass *pm_a = s->pm_a, *pm_b = s->pm_b;
        double f = bp->ks * ((pm_a->position - pm_b->position).norm() - s->rest_length);
        if (s->spring_type == BENDING) {
            // The force should be weaker
            double bendingCoefficient = 0.2;
            f *= bendingCoefficient;
        }
        Vector3D force = (pm_a->position - pm_b->position).unit();
        force *= f;
        pm_a->forces -= force;
        pm_b->forces += force;
    }


    // TODO (Part 2): Use Verlet integration to compute new point mass positions

    int i = 0;
    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {

        PointMass *pm = it.operator->();

        // hack: initial "hit" force in first iteration
        if (frame_num < 10) {
            pm->forces += Vector3D(-20000, 10000, 50000);
            //pm->forces += Vector3D(0, 5000, 50000);
        }

        Vector3D radiusVec = (pm->position - centroid).unit();
        Vector3D spin_around = Vector3D{0, 1, 0};
        Vector3D spin_dir = cross(radiusVec, spin_around);
        pm->forces += 5000 * (pm->position - pm->last_position).norm() * spin_dir;

        if (pm->pinned) continue;
        double d = bp->damping / 5000.0;
        Vector3D updated =
                pm->position + (1.0 - d) * (pm->position - pm->last_position) + (pm->forces / mass) * delta_t * delta_t;
        pm->last_position = pm->position;
        pm->position = updated;
        i += 1;
    }
    frame_num += 1;

  for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
    PointMass *pm = it.operator->();
    for (auto obj: *collision_objects) {
      obj->collide(*pm);
    }
  }

    // in length more than 10% per timestep [Provot 1995].
    for (auto it = springs.begin(); it != springs.end(); ++it) {
        Spring *s = it.operator->();
        PointMass *pm_a = s->pm_a;
        PointMass *pm_b = s->pm_b;
        double len = (pm_a->position - pm_b->position).norm();
        if (len > 1.1 * s->rest_length) {
            if (pm_a->pinned && pm_b->pinned) continue;
            int numPinned = 2 - (int) pm_a->pinned - (int) pm_b->pinned;
            double adjust = (len - 1.1 * s->rest_length) / numPinned;
            Vector3D adjustVector = adjust * (pm_a->position - pm_b->position).unit();
            if (!pm_a->pinned) pm_a->position -= adjustVector;
            if (!pm_b->pinned) pm_b->position += adjustVector;
        }
    }

    centroid = 0;
    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *bpm = it.operator->();
        centroid += bpm->position;
    }
    centroid /= (double)point_masses.size();
    radius = -1;
    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *bpm = it.operator->();
        double dist = (bpm->position - centroid).norm();
        if (dist > radius) radius = dist;
    }

}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Ball::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
  frame_num = 0;
  windOn = true;
}

void Ball::setupNormals() {
    for (auto face: faces) {
        PointMass *pm_A = &point_masses[face[0]];
        PointMass *pm_B = &point_masses[face[1]];
        PointMass *pm_C = &point_masses[face[2]];
        Vector3D normal = cross(pm_C - pm_B, pm_B - pm_A).unit();
        if (face.size() == 6) normal *= 1.5;
        for (auto faceIdx: face) {
            PointMass *pm = &point_masses[faceIdx];
            pm->ballnorm -= normal;
        }
    }
    for (auto pm: point_masses) {
        pm.ballnorm = pm.ballnorm.unit();
    }
}

void Ball::buildBallMesh() {
    if (point_masses.size() == 0) return;

    BallMesh *ballMesh = new BallMesh();
    vector<Triangle *> triangles;

    Vector3D uv_white = Vector3D(0.1, 0.2, 0.);
    Vector3D uv_black = Vector3D(0.9, 0.9, 0.);
    Vector3D uv;
    // Create vector of triangles
    for (int i = 0; i < faces.size(); i++) {
        vector<int> face = faces[i];
        for (int j = 1; j < face.size() - 1; j++) {
            PointMass *pm_A = &point_masses[face[0]];
            PointMass *pm_B = &point_masses[face[j]];
            PointMass *pm_C = &point_masses[face[j+1]];
            if (face.size() == 5){
                uv = uv_black;
            } else if (face.size() == 6){
                uv = uv_white;
            }
            triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                             uv, uv, uv));
        }
    }

    for (int i = 0; i < triangles.size(); i++) {
        Triangle *t = triangles[i];

        // Allocate new halfedges on heap
        Halfedge *h1 = new Halfedge();
        Halfedge *h2 = new Halfedge();
        Halfedge *h3 = new Halfedge();

        // Allocate new edges on heap
        Edge *e1 = new Edge();
        Edge *e2 = new Edge();
        Edge *e3 = new Edge();

        // Assign a halfedge pointer to the triangle
        t->halfedge = h1;

        // Assign halfedge pointers to point masses
        t->pm1->halfedge = h1;
        t->pm2->halfedge = h2;
        t->pm3->halfedge = h3;

        // Update all halfedge pointers
        h1->edge = e1;
        h1->next = h2;
        h1->pm = t->pm1;
        h1->triangle = t;

        h2->edge = e2;
        h2->next = h3;
        h2->pm = t->pm2;
        h2->triangle = t;

        h3->edge = e3;
        h3->next = h1;
        h3->pm = t->pm3;
        h3->triangle = t;
    }

    ballMesh->triangles = triangles;
    this->ballMesh = ballMesh;

    setupNormals();
}
