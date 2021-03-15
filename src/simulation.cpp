#include "simulation.hpp"

using namespace vcl;


void collision_sphere_plane(vcl::vec3& p, vcl::vec3& v, float r, vcl::vec3 const& n, vcl::vec3 const& p0)
{
    float const epsilon = 1e-5f;
    float const alpha_n = 0.95f;  // attenuation normal
    float const alpha_t = 0.90f;  // attenuation tangential

    float const s = dot(p-p0,n) - r;
    if( s<-epsilon )
    {
        vec3 const vn = dot(v,n) * n;
        vec3 const vt = v - vn;

        p = p - (s * n);
        v = -alpha_n * vn + alpha_t * vt;
    }
}

void collision_sphere_sphere(vcl::vec3& p1, vcl::vec3& v1, float r1, vcl::vec3& p2, vcl::vec3& v2, float r2)
{
    float const epsilon = 1e-5f;
    float const alpha = 0.95f;

    vec3 const p12 = p1-p2;
    float const d12 = norm(p12);

    if(d12 < r1+r2)
    {
        vec3 const u12 = p12/d12;
        float const collision_depth = r1+r2-d12;

        p1 += (collision_depth/2.0f+epsilon) * u12 ;
        p2 -= (collision_depth/2.0f+epsilon) * u12 ;

        if(norm(v1-v2)>0.2f){
            float const j = dot(v1-v2,u12);
            v1 = v1 - alpha*j*u12;
            v2 = v2 + alpha*j*u12;
        }
        else // Contact
        {
            v1 = v1/1.2f;
            v2 = v2/1.2f;
        }
    }
}

void simulate(std::vector<particle_structure>& particles, std::vector<Cup>& cups, float dt_true, bool &animate, bool &animate2)
{
	vec3 const g = {0,0,-9.81f};
	size_t const N_substep = 10;
	float const dt = dt_true/N_substep;
	for(size_t k_substep=0; k_substep<N_substep; ++k_substep)
	{
		size_t const N = particles.size();
		for (size_t k = 0; k < N; ++k)
		{
			particle_structure& particle = particles[k];

			vec3 const f = particle.m * g;

			particle.v = (1-0.9f*dt)*particle.v + dt*f;
			particle.p = particle.p + dt*particle.v;
		}

		// Collisions between spheres
		for(size_t k1=0; k1<N; ++k1)
		{
			for(size_t k2=k1+1; k2<N; ++k2)
			{
				particle_structure& p1 = particles[k1];
				particle_structure& p2 = particles[k2];

				collision_sphere_sphere(p1.p,p1.v,p1.r, p2.p,p2.v,p2.r);
			}
		}

		// Collisions with plane
		const std::vector<vec3> face_normal  = {{0, 1,0}, { 1,0,0}, {0,0, 1}, {0,-1,0}, {-1,0,0}, {0,0,-1}};
		const std::vector<vec3> face_position = {{0,-1,0}, {-1,0,0}, {0,0,-1}, {0, 1,0}, { 1,0,0}, {0,0, 1}};
		const size_t N_face = face_normal.size();
		for(size_t k=0; k<N; ++k){
			particle_structure& part = particles[k];
			for(size_t k_face=0; k_face<N_face; ++k_face)
				collision_sphere_plane(part.p, part.v, part.r, face_normal[k_face], face_position[k_face]);
		}

		// Collisions with cups
		mat3 rot_diag = {
                0, 0, 1,
                1, 1, 0,
                -1, 0, 0
        };

        mat3 rot_x = {
                0, 0, 1,
                0, 1, 0,
                -1, 0, 0
        };

        float x0, y0, z0;
        for (size_t k = 0; k < N; ++k){
            particle_structure& particle = particles[k];
            float x = particle.p[0], y = particle.p[1], z = particle.p[2];
            // cup 1
            x0 = 0., y0 = 0.15, z0 = -1.04;
            if((x0-x)*(x0-x) + (y0-y)*(y0-y) <= 0.2*0.2 && z>=z0 && z<=z0+0.55){ // collision detection
                cups[0].body.transform.rotate = rotation(rot_diag);
                cups[0].body.transform.translate = {0,0.15,-0.92};
                cups[0].seat.transform.rotate = rotation(rot_diag);
                cups[0].seat.transform.translate = {0,0.15,-0.92};
                animate = true;
            }
            // cup 2
            x0 = -0.6, y0 = -0.35, z0 = -1.04;
            if((x0-x)*(x0-x) + (y0-y)*(y0-y) <= 0.2*0.2 && z>=z0 && z<=z0+0.55){ // collision detection
                cups[1].body.transform.rotate = rotation(rot_x);
                cups[1].body.transform.translate = {-0.6,-0.35,-0.92};
                cups[1].seat.transform.rotate = rotation(rot_x);
                cups[1].seat.transform.translate = {-0.6,-0.35,-0.92};
                animate2 = true;
            }
        }
    }
}


// SPH simulation

// Convert a density value to a pressure
float density_to_pressure(float rho, float rho0, float stiffness) {
    return stiffness * (rho - rho0);
}

float W_laplacian_viscosity(vec3 const &p_i, vec3 const &p_j, float h) {
    float const r = norm(p_i - p_j);
    assert_vcl_no_msg(r <= h);
    return 45 / (3.14159f * std::pow(h, 6.0f)) * (h - r);
}

vec3 W_gradient_pressure(vec3 const &p_i, vec3 const &p_j, float h) {
    float const r = norm(p_i - p_j);
    assert_vcl_no_msg(r <= h);
    return -45 / (3.14159f * std::pow(h, 6.0f)) * std::pow(h - r, 2) * (p_i - p_j) / r;
}

float W_density(vec3 const &p_i, const vec3 &p_j, float h) {
    float const r = norm(p_i - p_j);
    assert_vcl_no_msg(r <= h);
    return 315.0 / (64.0 * 3.14159f * std::pow(h, 9)) * std::pow(h * h - r * r, 3.0f);
}


void update_density(buffer<sph_particle_element> &particles, float h, float m) {

    size_t const N = particles.size();

    for (size_t i = 0; i < N; ++i)
        particles[i].rho = 0.0f;

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            vec3 const &pi = particles[i].p;
            vec3 const &pj = particles[j].p;

            float const r = norm(pi - pj);
            if (r < h)
                particles[i].rho += m * W_density(pi, pj, h);
        }
    }
}

// Convert the particle density to pressure
void update_pressure(buffer<sph_particle_element> &particles, float rho0, float stiffness) {
    const size_t N = particles.size();
    for (size_t i = 0; i < N; ++i)
        particles[i].pressure = density_to_pressure(particles[i].rho, rho0, stiffness);
}

// Compute the forces and update the acceleration of the particles
void update_force(buffer<sph_particle_element> &particles, float h, float m, float nu) {
    // gravity
    const size_t N = particles.size();
    for (size_t i = 0; i < N; ++i)
        particles[i].f = m * vec3{0, 0, -9.81f};

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            if (i == j)
                continue;

            const vec3 &pi = particles[i].p;
            const vec3 &pj = particles[j].p;
            float r = norm(pi - pj);

            if (r < h) {
                const vec3 &vi = particles[i].v;
                const vec3 &vj = particles[j].v;

                const float pressure_i = particles[i].pressure;
                const float pressure_j = particles[j].pressure;

                const float rho_i = particles[i].rho;
                const float rho_j = particles[j].rho;

                vec3 force_pressure = {0, 0, 0};
                vec3 force_viscosity = {0, 0, 0};

                force_pressure =
                        -m / rho_i * (pressure_i + pressure_j) / (2 * rho_j) * W_gradient_pressure(pi, pj, h);
                force_viscosity = nu * m * m * (vj - vi) / rho_j * W_laplacian_viscosity(pi, pj, h);

                particles[i].f += force_pressure / 20 + force_viscosity / 20;
            }

        }
    }
}


// Simulate SPH
void simulate(float dt, buffer<sph_particle_element> &particles, sph_parameters_structure const &sph_parameters) {

    // Update values
    update_density(particles, sph_parameters.h,
                   sph_parameters.m);                   // First compute updated density
    update_pressure(particles, sph_parameters.rho0, sph_parameters.stiffness);       // Compute associated pressure
    update_force(particles, sph_parameters.h, sph_parameters.m, sph_parameters.nu);  // Update forces

    // Numerical integration
    float const damping = 0.005f;
    size_t const N = particles.size();
    float const m = sph_parameters.m;
    for (size_t k = 0; k < N; ++k) {
        vec3 &p = particles[k].p;
        vec3 &v = particles[k].v;
        vec3 &f = particles[k].f;

        v = (1 - damping) * v + dt * f / m;
        p = p + dt * v;
    }

    // Collision
    float const epsilon = 1e-3f;
    for (size_t k = 0; k < N; ++k) {
        vec3 &p = particles[k].p;
        vec3 &v = particles[k].v;

        // small perturbation to avoid alignment
        if (p.y < -1) {
            p.y = -1 + epsilon * rand_interval();
            v.y *= -0.5f;
            v.z -= 0.1f;
        }
        if (p.x < -1) {
            p.x = -1 + epsilon * rand_interval();
            v.x *= -0.5f;
            v.z -= 0.1f;
        }
        if (p.x > 1) {
            p.x = 1 - epsilon * rand_interval();
            v.x *= -0.5f;
            v.z -= 0.1f;
        }
        if (p.z < 0) {
            p.z = 0 + epsilon * rand_interval();
            v.x *= 0.1f;
            v.y *= 0.1f;
            v.z *= -0.5f;
        }
    }
}
