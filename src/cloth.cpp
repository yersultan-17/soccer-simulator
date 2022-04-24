#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <numeric>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  // wind
  this->clock = 0;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  // Let's set up the masses first
  for (int row = 0; row < num_height_points; ++row) {
    for (int col = 0; col < num_width_points; ++col) {
      vector<int> coords = {col, row};
      if (orientation == HORIZONTAL) {
        // vary over XZ plane, Y is set to 1
        double z = row * height / (num_height_points - 1);
        double x = col * width / (num_width_points - 1);
        bool is_pinned = false;
        for (const vector<int>& pinCoord : pinned) {
          if (pinCoord == coords) {
            is_pinned = true;
            break;
          }
        }
        point_masses.emplace_back(Vector3D(x, 1.0, z), is_pinned);
      } else {
        // vary over XY plane, Z is random number between -1/1000 and 1/1000
        double y = row * height / (num_height_points - 1);
        double x = col * width / (num_width_points - 1);
        double z = ((1/500.0) * ((double)rand() / (double)(RAND_MAX))) - 1/1000.0;
        bool is_pinned = false;
        for (const vector<int>& pinCoord : pinned) {
          if (pinCoord == coords) {
            is_pinned = true;
            break;
          }
        }
        point_masses.emplace_back(Vector3D(x, y, z), is_pinned);
      }
    }
  }

  // Let's set up the springs
  for (int row = 0; row < num_height_points; ++row) {
    for (int col = 0; col < num_width_points; ++col) {
      // add structural
      PointMass *cur = &point_masses[row * num_width_points + col];
      if (0 <= (row - 1) && (row - 1) < num_height_points && 0 <= col && col < num_width_points) {
        PointMass *above = &point_masses[(row - 1) * num_width_points + col];
        springs.emplace_back(cur, above, STRUCTURAL);
      }
      if (0 <= row && row < num_height_points && 0 <= col - 1 && col - 1 < num_width_points) {
        PointMass *left = &point_masses[row * num_width_points + col - 1];
        springs.emplace_back(cur, left, STRUCTURAL);
      }

      // add shearing
      if (0 <= (row - 1) && (row - 1) < num_height_points && 0 <= col - 1 && col - 1 < num_width_points) {
        PointMass *upperLeft = &point_masses[(row - 1) * num_width_points + col - 1];
        springs.emplace_back(cur, upperLeft, SHEARING);
      }
      if (0 <= (row - 1) && (row - 1) < num_height_points && 0 <= col + 1 && col + 1 < num_width_points) {
        PointMass *upperRight = &point_masses[(row - 1) * num_width_points + col + 1];
        springs.emplace_back(cur, upperRight, SHEARING);
      }

      // add bending
      if (0 <= (row - 2) && (row - 2) < num_height_points && 0 <= col && col < num_width_points) {
        PointMass *above = &point_masses[(row - 2) * num_width_points + col];
        springs.emplace_back(cur, above, BENDING);
      }
      if (0 <= row && row < num_height_points && 0 <= col - 2 && col - 2 < num_width_points) {
        PointMass *left = &point_masses[row * num_width_points + col - 2];
        springs.emplace_back(cur, left, BENDING);
      }
    }
  }
}

bool isSpringEnabled(Spring *s, ClothParameters *cp) {
  return ((s->spring_type == STRUCTURAL && cp->enable_structural_constraints)
          || (s->spring_type == SHEARING && cp->enable_shearing_constraints)
          || (s->spring_type == BENDING && cp->enable_bending_constraints));
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects,
                     float windSpeed, const Vector3D& windDirection) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Wind EC: keeping track of global clock of wind that resets periodically
  clock += delta_t;
  if (clock >= PI / 10) clock = 0;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D totalExtAcceleration = accumulate(external_accelerations.begin(), external_accelerations.end(), Vector3D());
  Vector3D totalExtForce = totalExtAcceleration * mass;

  // Wind EC: strength of the wind that depends on windSpeed parameter
  double windStrength = (fabs(windSpeed) * 1.4 * (cos(clock * 20) + 1) + 2.8);
  windStrength = (windSpeed < 0) ? windStrength : -windStrength;
  Vector3D wind = windDirection * windStrength;

  for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
    PointMass *pm = it.operator->();
    pm->forces = Vector3D();
    pm->forces += Vector3D(totalExtForce);

    // Wind EC: introducing random small movements to the wind effect for each pm
    float adjust = (float)rand() / float(RAND_MAX) - 0.5;
    wind += (adjust * (float)(3 * rand() / RAND_MAX - 1));
    // Wind EC: adding windForce to the total forces of the pm
    if (windSpeed == 0) wind *= 0;
    Vector3D windForce = wind * mass;
    pm->forces += windForce;
  }
  for (auto it = springs.begin(); it != springs.end(); ++it) {
    Spring *s = it.operator->();
    if (isSpringEnabled(s, cp)) {
      PointMass *pm_a = s->pm_a, *pm_b = s->pm_b;
      double f = cp->ks * ((pm_a->position - pm_b->position).norm() - s->rest_length);
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
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
    PointMass *pm = it.operator->();
    if (pm->pinned) continue;
    double d = cp->damping / 100.0;
    Vector3D updated = pm->position + (1.0 - d) * (pm->position - pm->last_position) + (pm->forces / mass) * delta_t * delta_t;
    pm->last_position = pm->position;
    pm->position = updated;
  }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (auto & pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
    PointMass *pm = it.operator->();
    for (auto obj: *collision_objects) {
      obj->collide(*pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (auto it = springs.begin(); it != springs.end(); ++it) {
    Spring *s = it.operator->();
    PointMass *pm_a = s->pm_a;
    PointMass *pm_b = s->pm_b;
    double len = (pm_a->position - pm_b->position).norm();
    if (len > 1.1 * s->rest_length) {
      if (pm_a->pinned && pm_b->pinned) continue;
      int numPinned = 2 - (int)pm_a->pinned - (int)pm_b->pinned;
      double adjust = (len - 1.1 * s->rest_length) / numPinned;
      Vector3D adjustVector = adjust * (pm_a->position - pm_b->position).unit();
      if (!pm_a->pinned) pm_a->position -= adjustVector;
      if (!pm_b->pinned) pm_b->position += adjustVector;
    }
  }


}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
    PointMass *pm = it.operator->();
    float pm_hash = hash_position(pm->position);
    if(map.find(pm_hash) == map.end()){
      map[pm_hash] = new vector<PointMass *>({pm});
    } else {
      map[pm_hash]->push_back(pm);
    }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float pm_hash = hash_position(pm.position);
  if (!map.count(pm_hash)) return;
  vector<PointMass *> *pm_vec = map.at(pm_hash);
  Vector3D correction = Vector3D();
  float candid_amount = 0;
  for (PointMass *candidate : *pm_vec) {
      if (candidate != &pm) {
          double distance = (pm.position - candidate->position).norm();
          if (distance > 2 * thickness) continue;
          candid_amount += 1;
          correction += (pm.position - candidate->position).unit() * (2 * thickness - distance);
      }
  }
  if (candid_amount == 0) return;
  correction = correction / (candid_amount * simulation_steps);
  Vector3D cur_position = pm.position;
  pm.position = pm.position + correction;
  //pm.last_position = cur_position;
}

int cantorPair(int k1, int k2) {
  return ((k1 + k2) * (k1 + k2 + 1)) / 2 + k2;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3 * width / num_width_points;
  double h = 3 * height / num_height_points;
  double t = max(w, h);
  auto x_coord = int((pos.x - fmod(pos.x, w)) / w);
  auto y_coord = int((pos.y - fmod(pos.y, h)) / h);
  auto z_coord = int((pos.z - fmod(pos.z, t)) / t);
  return cantorPair(cantorPair(x_coord, y_coord), z_coord);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */

      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;

      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;

      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);


      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
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

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
