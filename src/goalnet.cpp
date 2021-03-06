#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <numeric>

#include "goalnet.h"
#include "ball.h"

using namespace std;

Goalnet::Goalnet(double width, double height, int num_width_points,
                 int num_height_points, float thickness) {
    this->width = width;
    this->height = height;
    this->num_width_points = num_width_points;
    this->num_height_points = num_height_points;
    this->thickness = thickness;

    buildGrid();
    buildGoalnetMesh();
}

Goalnet::~Goalnet() {
    point_masses.clear();
    springs.clear();

    if (goalnetMesh) {
        delete goalnetMesh;
    }
}

void Goalnet::buildGrid() {
    int a = num_width_points / 4;
    int l = num_width_points - 2 * a;
    Vector3D start = {-5,-2,70};
    for (int row = 0; row < num_height_points - a; ++row) {
        for (int col = 0; col < num_width_points; ++col) {
            Vector3D position = start;
            position.x += max(0, min(l, col - a)) * 0.5;
            position.z += 0.5 * min(col, a) - 0.5 * max(0, col - (l + a));
            position.y += row * 0.5;
            vector<int> coords = {col, row};
            bool is_pinned = false;
            for (const vector<int>& pinCoord : pinned) {
                if (pinCoord == coords || row == num_height_points-a-1) {
                    is_pinned = true;
                    break;
                }
            }
            point_masses.emplace_back(position, is_pinned);
        }
    }

    //start = {-5,-2,30};
    start.y = (num_height_points - 2 * a - 1)  * 0.5 - 1;


    for (int row = 0; row < a; ++row) {
        for (int col = 0; col < num_width_points; ++col) {
            Vector3D position = start;

            int x_diff = min(max(0, col - a), l); // x diff is between a, w - 2a
            int y_diff = 0.5 * min(col, a) - 0.5 * max(0, col - (l + a));

            position.x += 0.5 * x_diff;
            position.z += 0.5 * (row + 1);
            position.y += y_diff;

            vector<int> coords = {col, row};
            bool is_pinned = false;
            for (const vector<int>& pinCoord : pinned) {
                if (col == a || col == l + a || row == 0) {
                    is_pinned = true;
                    break;
                }
            }
            point_masses.emplace_back(position, true); // pinning all top ones
        }
    }


    // Let's set up the springs
    for (int row = 0; row < num_height_points; ++row) {
        for (int col = 0; col < num_width_points; ++col) {
            // add structural
            PointMass *cur = &point_masses[row * num_width_points + col];
            if (0 <= (row - 1) && (row - 1) < num_height_points && 0 <= col && col < num_width_points) {
                if (row < num_height_points - a || (row > num_height_points - a && col >= 1)) {
                    PointMass *above = &point_masses[(row - 1) * num_width_points + col];
                    springs.emplace_back(cur, above, STRUCTURAL);
                }
            }
            if (0 <= row && row < num_height_points && 0 <= col - 1 && col - 1 < num_width_points) {
                PointMass *left = &point_masses[row * num_width_points + col - 1];
                if (row < num_height_points - a || (row >= num_height_points - a && col >= 1)) {
                    springs.emplace_back(cur, left, STRUCTURAL);
                }
            }
        }
    }

    // ORIGINAL
//    int a = num_width_points / 4;
//    int l = num_width_points - 2 * a;
//    Vector3D start = {-5,-2,70};
//    for (int row = 0; row < num_height_points; ++row) {
//        for (int col = 0; col < num_width_points; ++col) {
//            Vector3D position = start;
//            position.x += max(0, min(l, col - a)) * 0.5;
//            position.z += 0.5 * min(col, a) - 0.5 * max(0, col - (l + a));
//            position.y += row * 0.5;
//            vector<int> coords = {col, row};
//            bool is_pinned = false;
//            for (const vector<int>& pinCoord : pinned) {
//                if (pinCoord == coords || row == num_height_points - 1) {
//                    is_pinned = true;
//                    break;
//                }
//            }
//            point_masses.emplace_back(position, is_pinned);
//        }
//    }
//
//    // Let's set up the springs
//    for (int row = 0; row < num_height_points; ++row) {
//        for (int col = 0; col < num_width_points; ++col) {
//            // add structural
//            PointMass *cur = &point_masses[row * num_width_points + col];
//            if (0 <= (row - 1) && (row - 1) < num_height_points && 0 <= col && col < num_width_points) {
//                PointMass *above = &point_masses[(row - 1) * num_width_points + col];
//                springs.emplace_back(cur, above, STRUCTURAL);
//            }
//            if (0 <= row && row < num_height_points && 0 <= col - 1 && col - 1 < num_width_points) {
//                PointMass *left = &point_masses[row * num_width_points + col - 1];
//                springs.emplace_back(cur, left, STRUCTURAL);
//            }
//        }
//    }
}

void Goalnet::simulate(double frames_per_sec, double simulation_steps, GoalnetParameters *gnp,
                       vector<Vector3D> external_accelerations,
                       vector<CollisionObject *> *collision_objects,
                       Ball *ball,
                       float windSpeed, Vector3D& windDirection) {
    double mass = width * height * gnp->density / num_width_points / num_height_points;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D totalExtAcceleration = accumulate(external_accelerations.begin(), external_accelerations.end(), Vector3D());
    Vector3D totalExtForce = totalExtAcceleration * mass;

    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *pm = it.operator->();
        pm->forces = Vector3D();
        pm->forces += Vector3D(totalExtForce);
    }
    for (auto it = springs.begin(); it != springs.end(); ++it) {
        Spring *s = it.operator->();
        PointMass *pm_a = s->pm_a, *pm_b = s->pm_b;
        double f = gnp->ks * ((pm_a->position - pm_b->position).norm() - s->rest_length);
        Vector3D force = (pm_a->position - pm_b->position).unit();
        force *= f;
        pm_a->forces -= force;
        pm_b->forces += force;
    }


    // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *pm = it.operator->();
        if (pm->pinned) continue;
        double d = gnp->damping / 100.0;
        Vector3D updated = pm->position + (1.0 - d) * (pm->position - pm->last_position) + (pm->forces / mass) * delta_t * delta_t;
        pm->last_position = pm->position;
        pm->position = updated;
    }

    for (auto it = point_masses.begin(); it != point_masses.end(); ++it) {
        PointMass *pm = it.operator->();
        for (auto obj: *collision_objects) {
            obj->collide(*pm);
        }
        ball->collide(*pm);
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

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Goalnet::reset() {
    PointMass *pm = &point_masses[0];
    for (int i = 0; i < point_masses.size(); i++) {
        pm->position = pm->start_position;
        pm->last_position = pm->start_position;
        pm++;
    }
}

void Goalnet::buildGoalnetMesh() {
    if (point_masses.size() == 0) return;

    ClothMesh *goalnetMesh = new ClothMesh();
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

            Vector3D uv = Vector3D(0.1, 0.2, 0.);


            // Both triangles defined by vertices in counter-clockwise orientation
            triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                             uv, uv, uv));
            triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
                                             uv, uv, uv));
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

    goalnetMesh->triangles = triangles;
    this->goalnetMesh = goalnetMesh;
}

