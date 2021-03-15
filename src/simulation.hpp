#pragma once

#include "vcl/vcl.hpp"
using namespace vcl;

// Particle structure used for popcorns
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

// Structure of our cup = body (cylinder) + seat (circle)
struct Cup{
    mesh_drawable body;
    mesh_drawable seat;
};

// SPH Particle
struct sph_particle_element
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Force

    float rho;      // density at this particle position
    float pressure; // pressure at this particle position

    sph_particle_element() : p{0,0,0},v{0,0,0},f{0,0,0},rho(0),pressure(0) {}
};

// SPH simulation parameters
struct sph_parameters_structure
{
    // Influence distance of a particle (size of the kernel)
    float h = 0.12f;

    // Rest density (normalized to 1 - real unit should be 1000kg/m^2)
    float rho0 = 1;

    // Total mass of a particle (consider rho0 h^2)
    float m = rho0*h*h;

    // viscosity parameter
    //float nu = 0.02f;
    float nu = 2;

    // Stiffness converting density to pressure
    float stiffness = 0.1f;

};

void simulate(std::vector<particle_structure>& particles, std::vector<Cup>& cups, float dt, bool &animate, bool &animate2);
void simulate(float dt, vcl::buffer<sph_particle_element>& particles, sph_parameters_structure const& sph_parameters); // SPH
