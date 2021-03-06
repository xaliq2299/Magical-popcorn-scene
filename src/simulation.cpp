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

void simulate(std::vector<particle_structure>& particles, std::vector<Cup>& cups, float dt_true)
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

		// Collisions with cube
		const std::vector<vec3> face_normal  = {{0, 1,0}, { 1,0,0}, {0,0, 1}, {0,-1,0}, {-1,0,0}, {0,0,-1}};
		const std::vector<vec3> face_position = {{0,-1,0}, {-1,0,0}, {0,0,-1}, {0, 1,0}, { 1,0,0}, {0,0, 1}};
		const size_t N_face = face_normal.size();
		for(size_t k=0; k<N; ++k){
			particle_structure& part = particles[k];
			for(size_t k_face=0; k_face<N_face; ++k_face)
				collision_sphere_plane(part.p, part.v, part.r, face_normal[k_face], face_position[k_face]);
		}

		// Collisions with cups (obstacles)
        // rotation
        mat3 rot = {
                0, 0, 1,
                0, 1, 0,
                -1, 0, 0
        };

        // todo: handle the problem of popcorn being inside the cup
        // todo: add smoke effect

        // todo: change the following code so that it becomes less redundant
        float x0, y0, z0;
        for (size_t k = 0; k < N; ++k){
            particle_structure& particle = particles[k];
            float x = particle.p[0], y = particle.p[1], z = particle.p[2];
            // cup 1
            x0 = 0., y0 = 0.15, z0 = -1.04;
            if((x0-x)*(x0-x) + (y0-y)*(y0-y) <= 0.2*0.2 && z>=z0 && z<=z0+0.55){ // collision detection
                cups[0].body.transform.rotate = rotation(rot);
                cups[0].body.transform.translate = {0,0.15,-0.92};
                cups[0].seat.transform.rotate = rotation(rot);
                cups[0].seat.transform.translate = {0,0.15,-0.92};
            }
            // cup 2
            x0 = -0.6, y0 = -0.35, z0 = -1.04;
            if((x0-x)*(x0-x) + (y0-y)*(y0-y) <= 0.2*0.2 && z>=z0 && z<=z0+0.55){ // collision detection
                cups[1].body.transform.rotate = rotation(rot);
                cups[1].body.transform.translate = {-0.6,-0.35,-0.92};
                cups[1].seat.transform.rotate = rotation(rot);
                cups[1].seat.transform.translate = {-0.6,-0.35,-0.92};
            }
        }
        // particle.v = -particle.v;
    }
}


void apply_constraints(std::vector<particle_structure>& popcorns, std::map<size_t, vec3> const& positional_constraints, obstacles_parameters const& obstacles)
{
    // Fixed positions of the cloth
    // for(const auto& constraints : positional_constraints)
    //        position[constraints.first] = constraints.second;
    const float epsilon = 5e-3f;
    size_t const N = popcorns.size();
    for (size_t k = 0; k < N; ++k)
    {
        particle_structure& popcorn = popcorns[k];
        if(popcorn.p[2] < -1.02 + epsilon){ // todo: obstacles.z_ground
                std::cout << "Inside if\n";
                popcorn.p[2] = -100;
        }
    }

/*
     To do: apply external constraints
     for(int i=0;i<popcorns.size();i++){
         if(popcorns[i].p[2] < obstacles.z_ground) {
             std::cout << "Inside if\n";
             popcorns[i].p[2] = -5;
         }
     }

     float n = sqrt(position[i][0]*position[i][0]+position[i][1]*position[i][1]+position[i][2]*position[i][2]);
        if(norm(obstacles.sphere_center,position[i]) <= obstacles.sphere_radius+0.01){
            position[i] += abs(0.03+obstacles.sphere_radius-norm(obstacles.sphere_center,position[i]))*(position[i]-obstacles.sphere_center)/norm(position[i],obstacles.sphere_center);
        }
*/
}
