#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cgra/bone.hpp>
#include <fstream>
#include <sstream>
#include <memory.h>
#include <iterator>
#include <array>
#include <algorithm>

#include "opengl.hpp"
#include "imgui.h"

#include "cgra/matrix.hpp"
#include "cgra/wavefront.hpp"

#include "ex1.hpp"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/spline.hpp"
#include "skeleton.hpp"
#include "printer.h"
#include "spline.hpp"
#include <glm/gtx/string_cast.hpp>

cgra::Program Application::m_program;
glm::mat4 Application::m_model;
cgra::Mesh Application::m_bone_mesh;
cgra::Mesh Application::m_sphere_mesh_cyan;
cgra::Mesh Application::m_sphere_mesh_red;
cgra::Mesh Application::m_sphere_mesh_green;
cgra::Mesh Application::m_bone_segment_mesh;

void Application::init() {
	set_shaders(CGRA_SRCDIR "/res/shaders/simple.vs.glsl", CGRA_SRCDIR "/res/shaders/simple.fs.glsl");

	m_view = glm::mat4(1);
	m_view[3] = glm::vec4(0, 0, -10, 1);
	m_program.setViewMatrix(m_view);

	glm::vec3 rotation(1.0f, 1.0f, 0.0f);
	m_rotationMatrix = glm::rotate(glm::mat4(1.0f), 0.0f, glm::vec3(rotation[0], rotation[1], rotation[2]));

//	m_bone_mesh = loadObj(CGRA_SRCDIR "/res/models/frustrum-small.obj", 0);
//	m_bone_segment_mesh = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", 0);
	m_cube_mesh = loadObj(CGRA_SRCDIR "/res/models/cube.obj", glm::vec3(1.0, 0.0, 0.0));
	m_sphere_mesh_cyan = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.0, 1.0, 1.0));
	m_sphere_mesh_yellow = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 1.0, 0.0));
	m_sphere_mesh_red = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 0.0, 0.0));
	m_sphere_mesh_green = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.0, 1.0, 0.0));

	keyframes.emplace_back(-2.5, -1, 0);
	keyframes.emplace_back(0, 1, 0);
	keyframes.emplace_back(2.5, -1, 0);
	keyframes.emplace_back(5, 1, 0);

	speed_curve.emplace_back(-5.2, -3, 0);
	speed_curve.emplace_back(-5.2, -2, 0);
	speed_curve.emplace_back(-6.2, -3, 0);
	speed_curve.emplace_back(-6.2, -2, 0);

	update_speed_spline();
	update_spline();

}


void Application::update_speed_spline() {
	speed_points.clear();
	show_spline(speed_curve, catmull_divisions, speed_points, false);
}

void Application::update_spline() {
	new_points.clear();
	show_spline(keyframes, catmull_divisions, new_points, true);
}

float clip(float n, float lower, float upper) {
	return std::min(lower, std::max(n, upper));
}

float clip2(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

void Application::update_position() {
	float speed_mod = speed  * ((speed_points[((int) point_index)].x + 6.2f) / (speed_points[((int) point_index)].y + 3.0f));
	if (point_index + speed >= new_points.size()) {
		speed_mod = 0.0f;//speed - ((point_index + speed) - new_points.size());
		point_index = 0.0f;
	}
	point_index += speed_mod;
}

cgra::Mesh Application::loadObj(const char *filename, glm::vec3 colour) {
	std::ifstream obj_file(filename);
	if (!obj_file.is_open()) {
		std::cerr << "File not open\n";

		// Return a blank mesh if no file is found
		cgra::Matrix<double> vertices(0, 3);
		cgra::Matrix<unsigned int> triangles(0, 3);
		cgra::Mesh empty_mesh;
		empty_mesh.setData(vertices, triangles, glm::vec3(0));
		return empty_mesh;
	}

	unsigned int num_vertices = 0;
	unsigned int num_triangles = 0;

	// Count the number of vertices and faces
	for (std::string line; std::getline(obj_file, line);) {
		std::istringstream line_parser(line);
		char c = (char) line_parser.get();
		if (c == 'v') {
			num_vertices++;
		}
		if (c == 'f') {
			num_triangles++;
		}
	}
	// Reset the file to the top
	obj_file.clear();
	obj_file.seekg(0, std::ios::beg);

	cgra::Matrix<double> vertices(num_vertices, 3);
	cgra::Matrix<unsigned int> triangles(num_triangles, 3);
	cgra::Matrix<unsigned int> lines(0, 2);

	float vertices_raw[num_vertices][3];
	unsigned int triangles_raw[num_triangles][3];

	// Copy the vertices and faces out into 2d arrays
	int index_vertices = 0;
	int index_triangles = 0;
	for (std::string line; std::getline(obj_file, line);) {
		std::istringstream line_parser(line);
		char c = (char) line_parser.get();
		if (c == 'v') {
			line_parser >> vertices_raw[index_vertices][0];
			line_parser >> vertices_raw[index_vertices][1];
			line_parser >> vertices_raw[index_vertices][2];
			index_vertices++;
		}
		if (c == 'f') {
			line_parser >> triangles_raw[index_triangles][0];
			line_parser >> triangles_raw[index_triangles][1];
			line_parser >> triangles_raw[index_triangles][2];
			index_triangles++;
		}
	}

	// Write the vertices to the matrix
	for (size_t i = 0; i < num_vertices; i++) {
		for (int j = 0; j < 3; j++) {
			vertices(i, j) = vertices_raw[i][j];
		}
	}

	// Write the triangles to the matrix
	for (size_t i = 0; i < num_triangles; i++) {
		for (int j = 0; j < 3; j++) {
			triangles(i, j) = triangles_raw[i][j] - 1;
		}
	}

	obj_file.close();
	cgra::Mesh new_mesh;

	new_mesh.setData(vertices, triangles, colour);
	// Create and return a new mesh
	return new_mesh;
}

void Application::show_spline(std::vector<glm::vec3> &controls, int num_points, std::vector<glm::vec3> &points, bool main_spline) {

	Spline s(controls, num_points, main_spline, speed_points);

	for (float t = 0; t < 1; t += 1.0f / ((float) num_points)) {
		glm::vec3 pt = s.map(t);

		if (!main_spline) {
//			printer::print(pt);
			/*pt.x = clip(pt.x, -5.3f, -6.1f);
			pt.y = clip(pt.y, -2.1f, -2.9f);*/
			pt.x = std::max(pt.x, -6.2f);
			pt.y = std::max(pt.y, -3.0f);

			glm::vec3 prev = (points.empty()) ? controls[1] : points[points.size() - 1];
//			printer::print(pt);
			if (pt.x > prev.x) {
//				printer::print(std::to_string(pt.x) + " is greater than " + std::to_string(prev.x));
				pt.x = prev.x;
			}
			if (pt.y > prev.y) {
				pt.y = prev.y;
			}
//			printer::print(pt);
//			printer::print("");

		}

		points.push_back(pt);
	}

}

void Application::set_shaders(const char *vertex, const char *fragment) {
	m_program = cgra::Program::load_program(vertex, fragment);
}

glm::vec3 calculate_normalised_vector(glm::vec2 mouse_pos, glm::vec2 view_port_size) {
	// Convert to model coordinates
	glm::vec3 direction{mouse_pos.x / view_port_size.x * 2 - 1,
	                    -(mouse_pos.y / view_port_size.y * 2 - 1),
	                    0};

	float length = glm::length(direction);

	// Normalise
	if (length <= 1.0) {
		direction.z = (float) sqrt(1 - length);
	} else {
		direction = glm::normalize(direction);
	}
	return direction;
}

void Application::apply_arcball(glm::vec2 current_mouse_XY) {

	glm::vec3 current_arcball_direction = calculate_normalised_vector(current_mouse_XY, m_viewportSize);
	glm::vec3 previous_arcball_direction = calculate_normalised_vector(m_mousePosition, m_viewportSize);

	float angle = previous_arcball_direction.x * current_arcball_direction.x +
	              previous_arcball_direction.y * current_arcball_direction.y +
	              previous_arcball_direction.z * current_arcball_direction.z;

	glm::vec3 normal = glm::cross(previous_arcball_direction, current_arcball_direction);
	// If there is nothing to rotate around
	if (glm::length(normal) == 0) {
		return;
	}
	// Invert the matrix for camera coordinates
	glm::mat4 arcball_rotate = glm::inverse(glm::rotate(glm::mat4(), angle / -20, normal));
	// Apply matrix
	arcball_rotate *= m_rotationMatrix;
	m_rotationMatrix = arcball_rotate;
}

void Application::drawScene() {
	// Calculate the aspect ratio of the viewport;
	// width / height
	float aspectRatio = m_viewportSize.x / m_viewportSize.y;
	// Calculate the projection matrix with a field-of-view of 45 degrees
	m_proj = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);

	// Set the projection matrix
	m_program.setProjectionMatrix(m_proj);
//	glm::mat4 model_transform = m_rotationMatrix * glm::mat4(1.0f);

	/************************************************************
	 *                                                          *
	 * Use `m_translation`, `m_scale`, and `m_rotationMatrix`   *
	 * to create the  `modelTransform` matrix.                  *
	 * The following glm functions will be useful:              *
	 *    `glm::translate`                                      *
	 *    `glm::scale`                                          *
	 ************************************************************/

	m_model = glm::mat4(1.0f);

//	m_model *= glm::scale(m_model, glm::vec3(m_scale));

	for (auto new_point : new_points) {
		draw(m_sphere_mesh_cyan, new_point, glm::vec3(0.02), glm::mat4(1.0), glm::vec3(0), glm::vec3(m_scale),
		     m_rotationMatrix);
	}
	for (auto keyframe : keyframes) {
		draw(m_sphere_mesh_yellow, keyframe, glm::vec3(0.05), glm::mat4(1.0), glm::vec3(0), glm::vec3(m_scale),
		     m_rotationMatrix);
	}

	for (auto speed_point : speed_points) {
		draw(m_sphere_mesh_green, speed_point, glm::vec3(0.01), glm::mat4(1.0), glm::vec3(0), glm::vec3(1.0),
		     glm::mat4(1.0f));
	}
	for (auto i : speed_curve) {
		draw(m_sphere_mesh_red, i, glm::vec3(0.025), glm::mat4(1.0), glm::vec3(0), glm::vec3(1.0), glm::mat4(1.0f));
	}

	draw(m_cube_mesh, new_points[(int) (point_index)], glm::vec3(0.1), glm::mat4(1.0), glm::vec3(0), glm::vec3(m_scale),
	     m_rotationMatrix);
}

void Application::draw(cgra::Mesh mesh,
                       glm::vec3 position,
                       glm::vec3 scale,
                       glm::mat4 rotate,
                       glm::vec3 global_translation,
                       glm::vec3 global_scale,
                       glm::mat4 global_rotation) {
	glm::mat4 model_transform = m_model;

	model_transform = glm::translate(model_transform, glm::vec3(0, 0, 0));

	// Scale so that translation works properly
	model_transform = glm::scale(model_transform, glm::vec3(global_scale));

	model_transform *= global_rotation;

	model_transform *= rotate;

	model_transform = glm::translate(model_transform, position);

	model_transform = glm::scale(model_transform, scale);

	m_program.setModelMatrix(model_transform);

	mesh.draw();

}

void Application::draw_bone(cgra::Mesh mesh,
                            glm::vec3 scale,
                            glm::mat4 rotate,
                            glm::vec3 global_translation,
                            glm::vec3 global_scale,
                            glm::mat4 global_rotation) {

	glm::mat4 model_transform(1.0f);

	model_transform = glm::translate(model_transform, global_translation);

	// Scale so that translation works properly
	model_transform = glm::scale(model_transform, glm::vec3(global_scale));

	model_transform *= global_rotation;

	model_transform *= rotate;

//	model_transform *= glm::scale(model_transform, glm::vec3(1.0f / global_scale));

	model_transform = glm::scale(model_transform, scale);

	m_program.setModelMatrix(model_transform);

	mesh.draw();
}

void Application::draw(cgra::Mesh mesh, glm::vec3 scale, glm::mat4 model_transform) {

	model_transform *= glm::scale(model_transform, scale);

	m_program.setModelMatrix(model_transform);

	mesh.draw();
}

void Application::doGUI() {
	ImGui::SetNextWindowSize(ImVec2(450, 450), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Shapes");

	ImGui::SliderFloat("Speed", &speed, 0.0f, 2.5f);

	float coords[3] = {0, 0, 0};
	ImGui::InputFloat3("New Keyframe Coords", &coords[0]);

	if (ImGui::Button("Add Keyframe")) {
		keyframes.emplace_back(coords[0], coords[1], coords[2]);
//		catmull_divisions += 50;
		update_spline();
	}
	ImGui::End();
}


// Input Handlers

void Application::onMouseButton(int button, int action, int) {
	if (button >= 0 && button < 3) {
		// Set the 'down' state for the appropriate mouse button
		m_mouseButtonDown[button] = action == GLFW_PRESS;
	}
}

glm::vec3 Application::screen_to_world_coord(double mouse_x, double mouse_y) {
	mouse_y = m_viewportSize.y - mouse_y;
	glm::vec4 viewport(0, 0, m_viewportSize.x, m_viewportSize.y);
	// converts screen points to in-world coordinates using glm and the three matrices
	glm::vec3 worldPos = glm::unProject(glm::vec3(mouse_x, mouse_y, m_depth), m_view * m_model, m_proj, viewport);
	return worldPos;
}

void Application::manipulate(glm::vec3 mouse_point) {
	float threshold = 0.5;
	if (selected == -1) {
		select_keyframe = false;
		for (int i = 0; i < keyframes.size(); ++i) {
			float dist = glm::distance(keyframes[i], mouse_point);
			if (dist < threshold) {
				selected = i;
				select_keyframe = true;
				threshold = dist;
			}
		}
//		if (!select_keyframe) {
//			threshold = 0.5;
		for (int i = 0; i < speed_curve.size(); ++i) {
			float dist = glm::distance(speed_curve[i], mouse_point);
			if (dist < threshold) {
				selected = i;
				threshold = dist;
			}
		}
//		}
	} else if (select_keyframe) {
		keyframes[selected] = mouse_point;
	} else {

		if (selected == 1 || selected == 2) {
//			speed_curve[selected].y = clip(mouse_point.y, -3, -2);
		} else {
			speed_curve[selected] = mouse_point;
		}
	}
}

void Application::onCursorPos(double xpos, double ypos) {

	// Make a vec2 with the current mouse position
	glm::vec2 currentMousePosition(xpos, ypos);

	// Get the difference from the previous mouse position
//    glm::vec2 mousePositionDelta = currentMousePosition - m_mousePosition;

	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_LEFT]) {
		if (m_depth == -1) {
			// read the depth to the last pixel on the screen
			glReadPixels(int(xpos), int(m_viewportSize.y - ypos), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &m_depth);
		}
		manipulate(glm::vec3(screen_to_world_coord(xpos, ypos)));
		left_held = true;
	} else {
		m_depth = -1;
		selected = -1;
	}
	if (left_held) {
		update_spline();
		update_speed_spline();
		left_held = false;
	}
	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_MIDDLE]) {
		apply_arcball(currentMousePosition);
	}
	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_RIGHT]) {
		// TODO - make this work
		/*if (m_depth == -1) {
			glReadPixels(int(xpos), int(m_viewportSize.y - ypos), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &m_depth);
		}
		glm::vec3 new_point = screen_to_world_coord(xpos, ypos);
		new_point.z = 0;

		keyframes.push_back(new_point);
		update_spline();*/
	}


	// Update the mouse position to the current one
	m_mousePosition = currentMousePosition;
}

void Application::onKey(int key, int scancode, int action, int mods) {
	// `(void)foo` suppresses unused variable warnings
	(void) key;
	(void) scancode;
	(void) action;
	(void) mods;
}

void Application::onScroll(double xoffset, double yoffset) {
	// `(void)foo` suppresses unused variable warnings
	(void) xoffset;
	(void) yoffset;
	if (yoffset > 0) {
		m_scale += 0.1;
	} else if (yoffset < 0) {
		m_scale = (float) std::max(m_scale - 0.1, 0.05);
	}

}
