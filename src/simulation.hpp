#pragma once

#include "vcl/vcl.hpp"

struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed

    vcl::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

void simulate(std::vector<particle_structure>& particles, float dt);


