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


scene_environment scene;
mesh_drawable table;
mesh_drawable sphere;
mesh_drawable pan;
bool first_time = true;
obstacles_parameters obstacles;
std::vector<mesh_drawable> cups;

//Parameters
const vec3 pan_position = {-1,-1,-1.08};
const double v_factor = 1.8;

timer_event_periodic timer(0.5f);
std::vector<particle_structure> particles;


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
//        bool run = true;
//        if(run){
            float const dt = 0.01f * timer.scale;
//            size_t const N_substeps = 5;
//            for(size_t k_substep=0; k_substep<N_substeps; ++k_substep){
//                compute_forces(cloth.forces, cloth.position, cloth.velocity, cloth.normal, cloth.parameters, user.gui.wind_magnitude);
//                numerical_integration(cloth.position, cloth.velocity, cloth.forces, m, dt);
//                std::cout << "Called apply constraints\n";
//            }
        simulate(particles, cups, dt);
//        }
//        apply_constraints(particles, positional_constraints, obstacles); //  todo: precise whether per each user.gui.run or no

        display_scene();


        ImGui::End();
        imgui_render_frame(window);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

//    std::cout << "After finishing simulation\n";
//    for(int i=0;i<particles.size();i++)
//        std::cout << particles[i].p[2] << '\n';

    imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void emit_particle()
{
	// Emit particle with random velocity
	//  Assume first that all particles have the same radius and mass
	static buffer<vec3> const color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};
	if (timer.event && user.gui.add_sphere) {
		float const theta = rand_interval(0, 2*pi);
		vec3 v = vec3(1.0f*std::cos(theta), 1.0f*std::sin(theta), 4.0f);
        //add speed
		v = v_factor*v;
		particle_structure particle;
		//starting position
		particle.p = pan_position;
		particle.r = 0.045f;
		particle.c = color_lut[int(rand_interval()*color_lut.size())];
		particle.v = v;
		particle.m = 0.5f; //

        particles.push_back(particle);
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

	//popcorn
	//sphere = mesh_drawable(mesh_primitive_sphere());
	mesh popcorn = mesh_load_file_obj("assets/Rock.obj");
    sphere = mesh_drawable(popcorn);
    sphere.texture = opengl_texture_to_gpu(image_load_png("assets/popcorn2.png"));
//    sphere = mesh_drawable(mesh_primitive_sphere());


	//Mesh
    mesh table_m = mesh_load_file_obj("assets/Wood_Table.obj");
    mesh pan_m = mesh_load_file_obj("assets/pan.obj");

    //rotation
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

    //translation

    //scaling
    pan_m.position /=20;
    table_m.position *= 4;


    //mesh_draw
    pan = mesh_drawable(pan_m);
    pan.texture = opengl_texture_to_gpu(image_load_png("assets/pan.png"));
    table = mesh_drawable(table_m);
    table.transform.translate = {-0.5,-0.5,-2.93};
    pan.transform.translate = pan_position;
    table.texture = opengl_texture_to_gpu(image_load_png("assets/wood.png"));

    for(int i=0;i<1;i++){
	    mesh_drawable cup = mesh_drawable(mesh_primitive_cylinder());
    	cups.push_back(cup);
    	cups[i].texture = opengl_texture_to_gpu(image_load_png("assets/glass.png"));
   	}
	cups[0].transform.translate = {0, 0.15,-1.04};
//    cups[1].transform.translate = {0,0.15,-1.54-0.05};
//	cups[1].transform.translate = {-1.2,0.5,-1.04};
//	cups[2].transform.translate = {-0.6,-0.5,-1.04};
	cups[0].transform.scale = 0.5;
//	cups[1].transform.scale = 0.5;
//	cups[2].transform.scale = 0.5;
}

void display_scene()
{
	size_t const N = particles.size();
	for(size_t k=0; k<N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.shading.color = {1,1,1};
		sphere.transform.translate = particle.p;
		sphere.transform.scale = particle.r;
		if(first_time){
            first_time = false;
//            sphere.texture = opengl_texture_to_gpu(image_load_png("assets/b_w.png"));
		}

		draw(sphere, scene);
	}
    draw(table, scene);
	draw(pan, scene);
    for(int i=0;i<cups.size();i++)
	    draw(cups[i], scene);
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



