#pragma once

#include "vcl/vcl.hpp"
using namespace vcl;

struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

struct Cup{
    mesh_drawable body;
    mesh_drawable seat;
};

struct obstacles_parameters
{
    // float z_ground = -2.87; // todo: do we need table's z coordinate as an obstacle?
    vec3 cup1 = { 0, 0.15, -0.5};
    vec3 cup2 = { -1.2, 0.5, -0.5};
    vec3 cup3 = { -0.6, -0.5, -0.5};
};


void apply_constraints(std::vector<particle_structure>& popcorns, std::vector<mesh_drawable> cups, std::map<size_t, vec3> const& positional_constraints, obstacles_parameters const& obstacles);
void simulate(std::vector<particle_structure>& particles, std::vector<Cup>& cups, float dt);
