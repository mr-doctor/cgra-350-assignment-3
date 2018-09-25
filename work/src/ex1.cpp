#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <zconf.h>
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
#include "skeleton.hpp"
#include "printer.h"
#include <glm/gtx/string_cast.hpp>

cgra::Program Application::m_program;
cgra::Mesh Application::m_bone_mesh;
cgra::Mesh Application::m_sphere_mesh;
cgra::Mesh Application::m_bone_segment_mesh;

void Application::init() {

	set_shaders(CGRA_SRCDIR "/res/shaders/simple.vs.glsl", CGRA_SRCDIR "/res/shaders/simple.fs.glsl");

	glm::mat4 viewMatrix(1);
	viewMatrix[3] = glm::vec4(0, 0, -10, 1);
	m_program.setViewMatrix(viewMatrix);

	glm::vec3 rotation(1.0f, 1.0f, 0.0f);
	m_rotationMatrix = glm::rotate(glm::mat4(1.0f), 0.0f, glm::vec3(rotation[0], rotation[1], rotation[2]));

//	m_bone_mesh = loadObj(CGRA_SRCDIR "/res/models/frustrum-small.obj", 0);
//	m_bone_segment_mesh = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", 0);
	m_sphere_mesh = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.0, 1.0, 1.0));
	m_sphere_mesh_2 = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 1.0, 0.0));

	keyframes.emplace_back(-7.5, -1, 0);
	keyframes.emplace_back(-5, 1, 0);
	keyframes.emplace_back(-2.5, -1, 0);
	keyframes.emplace_back(0, 1, 0);
	keyframes.emplace_back(2.5, -1, 0);
	keyframes.emplace_back(5, 1, 0);
	keyframes.emplace_back(7.5, -1, 0);

	/*for (int i = 0; i < m_skeleton.m_bones.size(); ++i) {
		bone b = m_skeleton.m_bones[i];
		b.rotate = glm::lookAt(glm::vec3(0, 0, -1), b.boneDir, glm::vec3(0, 1, 0));
		b.rotate[0][0] += 1;
	}*/
	for (int i = 0; i < keyframes.size() - 3; i+=1) {
		show_spline(keyframes[i], keyframes[i+1], keyframes[i+2], keyframes[i+3], 50);
	}

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

void Application::show_spline(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, float num_points) {
	float t0 = 0.0f;
	float t1 = get_t(t0, p0, p1);
	float t2 = get_t(t1, p1, p2);
	float t3 = get_t(t2, p2, p3);

	for (float t = t1; t < t2; t += ((t2 - t1) / num_points)) {
		glm::vec3 A1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
		glm::vec3 A2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
		glm::vec3 A3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;

		glm::vec3 B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
		glm::vec3 B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

		glm::vec3 C = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2;

		new_points.push_back(C);
	}

}

float Application::get_t(float t, glm::vec3 p0, glm::vec3 p1) {
	float a = glm::pow((p1.x - p0.x), 2.0f) + glm::pow((p1.y - p0.y), 2.0f);
	float b = glm::pow(a, 0.5f);
	float c = glm::pow(b, 0.5f);

	return (c + t);
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
	glm::mat4 projectionMatrix = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);

	// Set the projection matrix
	m_program.setProjectionMatrix(projectionMatrix);
//	glm::mat4 model_transform = m_rotationMatrix * glm::mat4(1.0f);

	/************************************************************
	 *                                                          *
	 * Use `m_translation`, `m_scale`, and `m_rotationMatrix`   *
	 * to create the  `modelTransform` matrix.                  *
	 * The following glm functions will be useful:              *
	 *    `glm::translate`                                      *
	 *    `glm::scale`                                          *
	 ************************************************************/

	glm::mat4 model_transform(1.0f);
	model_transform *= m_rotationMatrix;

	for (unsigned int i = 0; i < new_points.size(); i++) {
		draw(m_sphere_mesh, new_points[i], glm::vec3(0.02), glm::mat4(1.0), glm::vec3(0), glm::vec3(1), m_rotationMatrix);
	}
	for (unsigned int i = 0; i < keyframes.size(); i++) {
		draw(m_sphere_mesh_2, keyframes[i], glm::vec3(0.05), glm::mat4(1.0), glm::vec3(0), glm::vec3(1), m_rotationMatrix);
	}

//	m_skeleton.renderSkeleton(model_transform, m_translation, glm::vec3(m_scale), m_rotationMatrix, core);

}

void Application::draw(cgra::Mesh mesh,
                       glm::vec3 position,
                       glm::vec3 scale,
                       glm::mat4 rotate,
                       glm::vec3 global_translation,
                       glm::vec3 global_scale,
                       glm::mat4 global_rotation) {
	glm::mat4 model_transform(1.0f);

	model_transform = glm::translate(model_transform, glm::vec3(0, 0, 0));

	// Scale so that translation works properly
	model_transform = glm::scale(model_transform, glm::vec3(global_scale));

	model_transform *= global_rotation;

	model_transform *= rotate;

	model_transform = glm::translate(model_transform, position);

//	model_transform *= glm::scale(model_transform, glm::vec3(1.0f / global_scale));

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

void Application::do_T() {
	for (size_t i = 0; i < m_skeleton.m_bones.size(); ++i) {
		m_skeleton.m_bones[i].rotation = glm::vec3(0, 0, 0);
	}
}


int pose = 0;

void Application::doGUI() {
	ImGui::SetNextWindowSize(ImVec2(450, 450), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Shapes");

	ImGui::End();
}


// Input Handlers

void Application::onMouseButton(int button, int action, int) {
	if (button >= 0 && button < 3) {
		// Set the 'down' state for the appropriate mouse button
		m_mouseButtonDown[button] = action == GLFW_PRESS;
	}
}

void Application::onCursorPos(double xpos, double ypos) {

	// Make a vec2 with the current mouse position
	glm::vec2 currentMousePosition(xpos, ypos);

	// Get the difference from the previous mouse position
//    glm::vec2 mousePositionDelta = currentMousePosition - m_mousePosition;

	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_LEFT]) {
		apply_arcball(currentMousePosition);
	} else if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_MIDDLE]) {

	} else if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_RIGHT]) {

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
