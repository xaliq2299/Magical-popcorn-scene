#include "vcl/vcl.hpp"
#include <iostream>

#include "simulation.hpp"


using namespace vcl;

struct gui_parameters {
	bool display_frame = true;
	bool add_sphere = true;
};

struct user_interaction_parameters {
	vec2 mouse_prev;
	timer_fps fps_record;
    bool display_transparent_billboard = true;
	mesh_drawable global_frame;
	gui_parameters gui;
	bool cursor_on_gui;
	
};
user_interaction_parameters user;

struct scene_environment
{
	camera_around_center camera;
	mat4 projection;
	vec3 light;
};

// SPH parameters
bool animate = false;
bool animate2 = false;
sph_parameters_structure sph_parameters; // Physical parameter related to SPH
buffer<sph_particle_element> sph_particles;      // Storage of the particles
buffer<sph_particle_element> sph_particles2;
mesh_drawable water_particle; // Sphere used to display a particle

const vec3 shift = {0.4,0.3 ,-1};
//const vec3 shift = {0.3,0.2 ,-1};
const vec3 shift2 = {-0.27,-0.4 ,-1};
const vec3 insideCup =  { 0, 0.15, -0.65};
const vec3 insideCup2 =  { -0.6,-0.35, -0.65};

// Some scene elements and their parameters
scene_environment scene;
mesh_drawable table;
mesh_drawable sphere;
mesh_drawable pan;
mesh_drawable blueDisk;
mesh_drawable blueDisk2;
bool first_time = true;
const vec3 pan_position = {-1,-1,-1.08};
std::vector<Cup> cups;
std::vector<particle_structure> vibrating_popcorns; // vibrating popcorns
const double v_factor = 1.8;
std::vector<particle_structure> particles;

timer_event_periodic timer(0.5f);


// smoke parameters
struct particle_bubble
{
    vec3 p0;
    float t0;
    vec3 color;
    float radius;
    float phase;
};
struct particle_billboard
{
    vec3 p0;
    float t0;
};
// Visual elements of the scene related to the billboard/smoke
mesh_drawable quad;   // used to display the sprites

// smoke-related functions
particle_billboard create_new_billboard(float t);
vec3 compute_billboard_position(particle_billboard const& billboard, float t_current);
template <typename T> void remove_old_particles(std::vector<T>& particles, float t_current, float t_max);

// Particles and their timer
std::vector<particle_bubble> bubbles;
timer_event_periodic timer_bubble(0.15f);
std::vector<particle_billboard> billboards;
timer_event_periodic timer_billboard(0.05f);

// some functionalities
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);
void initialize_data();
void display_scene();
void display_interface();
void emit_particle();


int main(int, char* argv[])
{
	std::cout << "Run " << argv[0] << std::endl;

	int const width = 1280, height = 1024;
	GLFWwindow* window = create_window(width, height);
	window_size_callback(window, width, height);
	std::cout << opengl_info_display() << std::endl;

	imgui_init(window);
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);
	
	std::cout<<"Initialize data ..."<<std::endl;
	initialize_data();

	std::cout<<"Start animation loop ..."<<std::endl;
	user.fps_record.start();
	timer.start();
	glEnable(GL_DEPTH_TEST);
	while (!glfwWindowShouldClose(window))
	{
		scene.light = scene.camera.position();
		user.fps_record.update();
		timer.update();
		
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);
		imgui_create_frame();
		if(user.fps_record.event) {
			std::string const title = "VCL Display - "+str(user.fps_record.fps)+" fps";
			glfwSetWindowTitle(window, title.c_str());
		}

		ImGui::Begin("GUI",NULL,ImGuiWindowFlags_AlwaysAutoResize);
		user.cursor_on_gui = ImGui::IsAnyWindowFocused();

		if(user.gui.display_frame) draw(user.global_frame, scene);

        emit_particle();
        display_interface();

        std::map<size_t,vec3> positional_constraints;
        float const dt = 0.01f * timer.scale;
        simulate(particles, cups, dt, animate, animate2);
        display_scene();

        // SPH simulation
        if(animate){
            float const dt2 = 0.005f * timer.scale;
            simulate(dt2, sph_particles, sph_parameters);
        }

        if(animate2) {
            float const dt2 = 0.005f * timer.scale;
            simulate(dt2, sph_particles2, sph_parameters);
        }

        ImGui::End();
        imgui_render_frame(window);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void emit_particle()
{
	// Emit particle with random velocity
	// Assume first that all particles have the same radius and mass
	static buffer<vec3> const color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};
	if (timer.event && user.gui.add_sphere) {
		float const theta = rand_interval(0, 2*pi);
		vec3 v = vec3(1.0f*std::cos(theta), 1.0f*std::sin(theta), 4.0f);
        // add speed
		v = v_factor*v;
		particle_structure particle;
		// starting position
		particle.p = pan_position;
		particle.r = 0.045f;
		particle.c = color_lut[int(rand_interval()*color_lut.size())];
		particle.v = v;
		particle.m = 0.5f; //

        particles.push_back(particle);
	}
}


void initialize_sph()
{
    // Initial particle spacing (relative to h)
    float const c = 0.09f;//0.7f;
    float h = sph_parameters.h;
    h+=0.3f;
    float z = 0.3f;
    // Fill a square with particles
    sph_particles.clear();
    sph_particles2.clear();
    for(float x=h; x<1.0f-h; x=x+c*h)
    {
        for(float y=-1.0f+h; y<1.0f-h; y=y+c*h)
        {
            sph_particle_element particle;
            particle.p = {x+h/8.0*rand_interval(),y+h/8.0*rand_interval(),z+h/8.0*rand_interval()}; // a zero value in z position will lead to a 2D simulation
            particle.p /= 5;
            sph_particles.push_back(particle);
            sph_particles2.push_back(particle);
        }
    }
}


void initialize_data()
{
	GLuint const shader_mesh = opengl_create_shader_program(opengl_shader_preset("mesh_vertex"), opengl_shader_preset("mesh_fragment"));
	GLuint const shader_uniform_color = opengl_create_shader_program(opengl_shader_preset("single_color_vertex"), opengl_shader_preset("single_color_fragment"));
	GLuint const texture_white = opengl_texture_to_gpu(image_raw{1,1,image_color_type::rgba,{255,255,255,255}});
	mesh_drawable::default_shader = shader_mesh;
	mesh_drawable::default_texture = texture_white;
	curve_drawable::default_shader = shader_uniform_color;
	segments_drawable::default_shader = shader_uniform_color;
	
	user.global_frame = mesh_drawable(mesh_primitive_frame());
	user.gui.display_frame = false;
	scene.camera.distance_to_center = 2.5f;
	scene.camera.look_at({5,5,5}, {0,0,0}, {0,0,2});

	// popcorn
	mesh popcorn = mesh_load_file_obj("assets/Rock.obj");
    sphere = mesh_drawable(popcorn);
    sphere.texture = opengl_texture_to_gpu(image_load_png("assets/popcorn.png"));

	// table and pan meshes
    mesh table_m = mesh_load_file_obj("assets/Wood_Table.obj");
    mesh pan_m = mesh_load_file_obj("assets/pan.obj");

    // rotation
    mat3 rot = {
            1.0,0,0,
            0,float(cos(1.5708)),-1*float(sin(1.5708)),
            0,float(sin(1.5708)),float(cos(1.5708))
    };
    for(int i=0; i<pan_m.position.size(); i++) {
        pan_m.position.at(i) = rot*pan_m.position.at(i);
    }

    for(int i=0; i<table_m.position.size(); i++) {
        table_m.position.at(i) = rot*table_m.position.at(i);
    }

    // scaling
    pan_m.position /= 20;
    table_m.position *= 4;

    // mesh_draw
    pan = mesh_drawable(pan_m);
    pan.texture = opengl_texture_to_gpu(image_load_png("assets/pan.png"));
    table = mesh_drawable(table_m);
    table.transform.translate = {-0.5,-0.5,-2.93};
    pan.transform.translate = pan_position;
    table.texture = opengl_texture_to_gpu(image_load_png("assets/wood.png"));

    // cups
    for(int i=0;i<2;i++){
        Cup cup;
	    cup.body = mesh_drawable(mesh_primitive_cylinder(0.2f));
        cup.seat = mesh_drawable(mesh_primitive_disc(0.2f));
    	cups.push_back(cup);
        cups[i].body.texture = opengl_texture_to_gpu(image_load_png("assets/cup_body.png"));
    	cups[i].seat.texture = opengl_texture_to_gpu(image_load_png("assets/cup_seat.png"));
   	}
	cups[0].body.transform.translate = cups[0].seat.transform.translate = {0, 0.15, -1.04};
    cups[1].body.transform.translate = cups[1].seat.transform.translate = {-0.6,-0.35,-1.04};
    cups[0].body.transform.scale = cups[0].seat.transform.scale = 0.5;
    cups[1].body.transform.scale = cups[1].seat.transform.scale = 0.5;

    // adding vibrating popcorns
    for(int i=0;i<70;i++){
        particle_structure particle;
        // starting position
        particle.p = pan_position;
        particle.r = 0.045f;
        particle.m = 0.5f;
        vibrating_popcorns.push_back(particle);
    }

    // SPH initialize
    initialize_sph();
    water_particle = mesh_drawable(mesh_primitive_cube());
    water_particle.transform.scale = 0.08f;
    water_particle.shading.phong = {10, 0, 0};
    water_particle.shading.color = {0, 0, 1};
    blueDisk = mesh_drawable(mesh_primitive_disc());
    blueDisk.shading.color = {0,0,1};

    blueDisk2 = mesh_drawable(mesh_primitive_disc());
    blueDisk2.shading.color = {0,0,1};

    // Smoke: billboard texture and associated quadrangle
    GLuint const texture_billboard = opengl_texture_to_gpu(image_load_png("assets/smoke.png"));
    float const L = 0.3f; // size of the quad
    quad = mesh_drawable(mesh_primitive_quadrangle({-L,-L,0},{L,-L,0},{L,L,0},{-L,L,0}));
    quad.texture = texture_billboard;
}


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

// Smoke: Billboards
particle_billboard create_new_billboard(float t)
{
    particle_billboard billboard;
    billboard.t0 = t;
    float const theta  = rand_interval(0.0f, 2*pi);
    billboard.p0 = 0.5f*vec3(std::cos(theta), 0.0f, std::sin(theta))-vec3(1,1.2,0.92);
    return billboard;
}

vec3 compute_billboard_position(particle_billboard const& billboard, float t_current)
{
    float const t = t_current - billboard.t0;

    float const theta = std::atan2(billboard.p0.x, billboard.p0.z);
    float const x = t/2*std::sin(theta);
    float const z = t/2*std::cos(theta);
    float const y = -5*(t/3)*(t/3)+2.5f*(t/3);

    return billboard.p0 + vec3(x,y,z);
}


void display_scene()
{
    // displaying the popcorns going out from the pan
	size_t const N = particles.size();
	for(size_t k=0; k<N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.shading.color = {1,1,1};
		sphere.transform.translate = particle.p;
		sphere.transform.scale = particle.r;
		if(first_time){
            first_time = false;
		}

		draw(sphere, scene);
	}

    // displaying vibrating popcorns
    for(int i=0;i<vibrating_popcorns.size();i++) {
        particle_structure const& particle = vibrating_popcorns[i];
        sphere.transform.translate = {RandomFloat(-0.9, -1.34), RandomFloat(-0.9, -1.2), -0.92};
        sphere.transform.scale = particle.r;
        draw(sphere, scene);
    }

    draw(table, scene); // displaying table
	draw(pan, scene); // displaying pan
	// displaying cups
    for(int i=0;i<cups.size();i++) {
        draw(cups[i].body, scene);
        draw(cups[i].seat, scene);
    }

    // SPH display
    // remove this to remove the spheres of the particles of fluid
    if(animate){
        for (size_t k = 0; k < sph_particles.size(); ++k) {
            vec3 const& p = sph_particles[k].p;
            water_particle.transform.translate = p + shift;
            draw(water_particle, scene);
        }
    }
    else {
        blueDisk.transform.translate = insideCup;
        blueDisk.transform.scale = 0.1f;
        draw(blueDisk, scene);
    }

    // for the second cup
    if(animate2){
        for (size_t k = 0; k < sph_particles2.size(); ++k) {
            vec3 const& p = sph_particles2[k].p;
            water_particle.transform.translate = p + shift2;
            draw(water_particle, scene);
        }
    }
    else {
        blueDisk2.transform.translate = insideCup2;
        blueDisk2.transform.scale = 0.1f;
        draw(blueDisk2, scene);
    }

    // Smoke
    timer_billboard.update();
    if(timer_billboard.event)
        billboards.push_back( create_new_billboard(timer_billboard.t) );
    // Enable transparency using alpha blending (if display_transparent_billboard is true)
    if(user.display_transparent_billboard){
        glEnable(GL_BLEND);
        glDepthMask(false);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    else
        glDisable(GL_BLEND);

    for(size_t k = 0; k < billboards.size(); ++k)
    {
        vec3 const p = compute_billboard_position(billboards[k], timer_billboard.t);
        quad.transform.translate = p;
        quad.transform.rotate = scene.camera.orientation();

        float const alpha = (timer_billboard.t-billboards[k].t0)/3.0f;
        quad.shading.alpha = (1-alpha)*std::sqrt(alpha);

        draw(quad, scene);
    }
    glDepthMask(true);
    remove_old_particles(billboards, timer_billboard.t, 3.0f);
}


// Generic function allowing to remove particles with a lifetime greater than t_max
template <typename T>
void remove_old_particles(std::vector<T>& particles, float t_current, float t_max)
{
    for (auto it = particles.begin(); it != particles.end();)
    {
        if (t_current - it->t0 > t_max)
            it = particles.erase(it);
        if(it!=particles.end())
            ++it;
    }
}


void display_interface()
{
	ImGui::Checkbox("Frame", &user.gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &timer.event_period, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &user.gui.add_sphere);
}

void window_size_callback(GLFWwindow* , int width, int height)
{
	glViewport(0, 0, width, height);
	float const aspect = width / static_cast<float>(height);
	scene.projection = projection_perspective(50.0f*pi/180.0f, aspect, 0.1f, 100.0f);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
	vec2 const& p0 = user.mouse_prev;
	glfw_state state = glfw_current_state(window);

	auto& camera = scene.camera;
	if(!user.cursor_on_gui){
		if(state.mouse_click_left && !state.key_ctrl)
			scene.camera.manipulator_rotate_trackball(p0, p1);
		if(state.mouse_click_left && state.key_ctrl)
			camera.manipulator_translate_in_plane(p1-p0);
		if(state.mouse_click_right)
			camera.manipulator_scale_distance_to_center( (p1-p0).y );
	}

	user.mouse_prev = p1;
}

void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
	opengl_uniform(shader, "projection", current_scene.projection);
	opengl_uniform(shader, "view", scene.camera.matrix_view());
	opengl_uniform(shader, "light", scene.light, false);
}

