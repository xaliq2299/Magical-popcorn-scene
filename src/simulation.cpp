#include "simulation.hpp"

using namespace vcl;




void simulate(std::vector<particle_structure>& particles, float dt)
{
	vec3 const g = {0,0,-9.81f};
    vec3 const n = {0,0,1};
	size_t const N = particles.size();
	float alpha = 0.9;
    float beta = 0.9;
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure& particle = particles[k];

		vec3 const f = particle.m * g;

		particle.v = (1-0.9f*dt)*particle.v + dt*f;
		if(particle.p[2] + dt*particle.v[2] >-1+particle.r) {
            particle.p = particle.p + dt*particle.v;
		}
		else {
            vec3 v_n = dot(particle.v,n) * n;
            vec3 v_t = particle.v - v_n;
            particle.v =  alpha*v_t - beta*v_n;
            //particle.p = particle.p + dt*particle.v;
            if(particle.p[2] <(-1+particle.r)){
                particle.p[2] = -1+particle.r+0.01;
            }
		}

	}


    for (size_t i = 0; i < N; ++i) {
        particle_structure &particle_1 = particles[i];
        for (size_t j = i; j < N; ++j) {
            particle_structure &particle_2 = particles[j];

            float dist = norm(particle_1.p - particle_2.p);

            if(dist < particle_1.r + particle_2.r + 0.01 && norm(particle_2.p - particle_1.p) != 0.) {
                //we have collistion
                vec3 u = normalize(particle_2.p - particle_1.p);
                float J = (2*particle_1.m*particle_2.m/(particle_1.m + particle_2.m)) *dot((particle_2.v - particle_1.v), u);
                particle_1.v = particle_1.v + (J/particle_1.m)*u;
                particle_2.v = particle_2.v - (J/particle_2.m)*u;
                particle_2.p += abs(dist - particle_1.r - particle_2.r - 0.01)*u;
            }
        }
    }
        // To do :
	//  Handle collision ...

}
