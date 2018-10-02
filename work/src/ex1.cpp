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
cgra::Mesh Application::m_bone_mesh_yellow;
cgra::Mesh Application::m_sphere_mesh_cyan;
cgra::Mesh Application::m_sphere_mesh_red;
cgra::Mesh Application::m_sphere_mesh_green;
cgra::Mesh Application::m_bone_segment_mesh;
bone * Application::selected_bone;

void Application::init(std::string amc_file, std::string keyframe_file) {
	set_shaders(CGRA_SRCDIR "/res/shaders/simple.vs.glsl", CGRA_SRCDIR "/res/shaders/simple.fs.glsl");

	m_view = glm::mat4(1);
	m_view[3] = glm::vec4(0, 0, -10, 1);
	m_program.setViewMatrix(m_view);

	glm::vec3 rotation(1.0f, 1.0f, 0.0f);
	m_rotationMatrix = glm::rotate(glm::mat4(1.0f), 0.0f, glm::vec3(rotation[0], rotation[1], rotation[2]));

	m_bone_mesh = loadObj(CGRA_SRCDIR "/res/models/frustrum-small.obj", glm::vec3(0.1, 0.1, 0.1));
	m_bone_segment_mesh = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.2, 0.2, 0.2));
	m_bone_mesh_yellow = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 1.0, 0.0));

	m_skeleton = Skeleton(CGRA_SRCDIR "/res/models/" + amc_file);

	num_keyframes = parseKeyframes(CGRA_SRCDIR "/res/keyframes/" + keyframe_file);

	m_cube_mesh = loadObj(CGRA_SRCDIR "/res/models/cube.obj", glm::vec3(1.0, 0.0, 0.0));
	m_sphere_mesh_cyan = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.0, 1.0, 1.0));
	m_sphere_mesh_yellow = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 1.0, 0.0));
	m_sphere_mesh_red = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(1.0, 0.0, 1.0));
	m_sphere_mesh_green = loadObj(CGRA_SRCDIR "/res/models/sphere.obj", glm::vec3(0.0, 1.0, 0.0));

	for (auto bone : m_skeleton.m_bones) {
		bone.rotation = glm::vec3(0, 0, 0);
	}
}

void Application::line_to_rotations(std::string line) {
	std::istringstream iss(line);
	std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
									 std::istream_iterator<std::string>());
	m_skeleton.m_bones[m_skeleton.findBone(results[0])].frames.emplace_back(
			std::atof(results[1].c_str()),
			std::atof(results[2].c_str()),
			std::atof(results[3].c_str())
	);
}

void Application::update() {
	if (num_keyframes == 0) {
		return;
	}

	float speed_mod = speed;
	if (keyframe_index + speed >= num_keyframes) {
		speed_mod = 0.0f;
		keyframe_index = 0.0f;
	}
	if (play) {
		keyframe_index += speed_mod;
		for (auto &bone : m_skeleton.m_bones) {
			float idx = 0;

			float t = std::modf(keyframe_index, &idx);

			int i = int(idx);

			if (idx < bone.frames.size() - 1) {
				glm::vec3 temp = bone.frames[i];
				glm::quat start_frame = glm::quat_cast(
						glm::eulerAngleXYZ(temp.x, temp.y, temp.z));

				temp = bone.frames[i + 1];

				glm::quat end_frame = glm::quat_cast(
						glm::eulerAngleXYZ(temp.x, temp.y, temp.z));
				bone.rotation = glm::eulerAngles(glm::slerp(start_frame, end_frame, t));
			} else {
				bone.rotation = bone.frames[bone.frames.size() - 1];
			}
		}
	}
}

int Application::parseKeyframes(std::string filename) {
	std::ifstream kf_file(filename);

	if (!kf_file.is_open()) {
		std::cerr << "File not open\n";
		exit(1);
	}

	int frames = 0;
	for (std::string line; std::getline(kf_file, line);) {
		std::ifstream pose_file(CGRA_SRCDIR "/res/keyframes/" + line);
		if (!pose_file.is_open()) {
			std::cerr << "File not open\n";
			exit(1);
		}
		std::vector<std::string> curr_frame;
		for (std::string line2; std::getline(pose_file, line2);) {
			line_to_rotations(line2);
		}
		frames++;
		pose_file.close();
	}

	kf_file.close();
	return frames;
}

float clip(float n, float lower, float upper) {
	return std::min(lower, std::max(n, upper));
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
	glm::mat4 viewMatrix(1);
	viewMatrix[3] = glm::vec4(0, 0, -10, 1);
	m_program.setViewMatrix(viewMatrix);

	m_view = viewMatrix;

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

	m_model = m_rotationMatrix * glm::mat4(1.0f);

//	m_model *= glm::scale(m_model, glm::vec3(m_scale));

	m_skeleton.renderSkeleton(m_model);
}

void Application::draw(cgra::Mesh mesh, glm::vec3 scale, glm::mat4 model_transform) {

	model_transform = glm::scale(model_transform, scale);

	m_program.setModelMatrix(model_transform);

	mesh.draw();
}

void Application::saveFile(const char *str) {
	std::ofstream out_file(str);

	if (!out_file.is_open()) {
		return;
	}

	for (auto bone : m_skeleton.m_bones) {
		out_file << bone.name << " " << bone.rotation.x << " " << bone.rotation.y << " " << bone.rotation.z
				 << std::endl;
	}

	out_file.close();
}

void addBones(bone &bone) {
	for (auto &m_bone : bone.children) {
		if (ImGui::TreeNode(m_bone->name.c_str())) {
			if (m_bone->name != "root") {
				float rotations[3] = {m_bone->rotation[0], m_bone->rotation[1], m_bone->rotation[2]};
				if (ImGui::InputFloat3("Rotations", &rotations[0])) {
					for (int i = 0; i < 3; ++i) {
						m_bone->rotation[i] = rotations[i];
					}
				}
			}
			addBones(*m_bone);
			ImGui::TreePop();
		}
	}
}

void Application::doGUI() {
	ImGui::SetNextWindowSize(ImVec2(450, 450), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Controls");

	if (ImGui::Button("PLAY")) {
		play = true;
	}
	if (ImGui::Button("PAUSE")) {
		play = false;
	}
	if (ImGui::Button("STOP")) {
		play = false;
		for (auto &bone : m_skeleton.m_bones) {
			bone.rotation = glm::vec3(0);
		}
	}

	ImGui::SliderFloat("Speed", &speed, 0.0f, 2.5f);
	if (selected_bone != nullptr) {
		bone* b = selected_bone;
		ImGui::Text("%s (%f,%f,%f)", b->name.c_str(), b->rotation.x, b->rotation.y, b->rotation.z);
	} else {
		ImGui::Text("No joint selected");
	}

	if (ImGui::TreeNode("Bones")) {
		addBones(m_skeleton.m_bones[m_skeleton.findBone("root")]);
		ImGui::TreePop();
	}

	static std::string filename;
	filename.resize(20);

	if (save) {
		ImGui::OpenPopup("Save This Pose");

		if (ImGui::BeginPopupModal("Save This Pose", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
			ImGui::InputText("File .pos", &filename[0], filename.size());

			if (ImGui::Button("Save", ImVec2(150, 0))) {
				saveFile((CGRA_SRCDIR "/res/keyframes/" + filename).c_str());
				ImGui::CloseCurrentPopup();
				save = false;
			}

			if (ImGui::Button("Cancel", ImVec2(150, 0))) {
				ImGui::CloseCurrentPopup();
				save = false;
			}

			ImGui::EndPopup();
		}
	}


	ImGui::End();
}


// Input Handlers

void Application::onMouseButton(int button, int action, int) {

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

		if (control_held) {
			auto xpos = int(m_mousePosition.x);
			auto ypos = int(m_viewportSize.y - m_mousePosition.y);
			glm::vec4 viewport(0, 0, m_viewportSize.x, m_viewportSize.y);
			float m_depth;
			glReadPixels(xpos, ypos, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &m_depth);
			glm::vec3 worldPos = glm::unProject(glm::vec3(xpos, ypos, m_depth), m_view, m_proj, viewport);
			manipulate(worldPos);
		}
	}

	if (button >= 0 && button < 3) {
		// Set the 'down' state for the appropriate mouse button
		m_mouseButtonDown[button] = action == GLFW_PRESS;
	}
}

void Application::find_bone(glm::vec3 pos) {
	float threshold = 0.5f;
	bone* current = nullptr;
	bone* last_selected = nullptr;
	for (auto &bone : m_skeleton.m_bones) {
		if (bone.selected) {
			last_selected = &bone;
		}
		bone.selected = false;
		float distance = glm::distance2(bone.world_pos, pos);
		if (distance < threshold) {
			threshold = distance;
			current = &bone;
		}
	}
	if (current == nullptr || last_selected == current) return;
	current->selected = true;
}

void Application::manipulate(glm::vec3 mouse_point) {
	find_bone(mouse_point);
	int count = 0;
	for (auto &bone : m_skeleton.m_bones) {
		if (bone.selected) {
			selected_bone = &bone;
			break;
		}
		count++;
	}
	if (count == m_skeleton.m_bones.size()) {
		selected_bone = nullptr;
	}
}

void Application::onCursorPos(double xpos, double ypos) {

	// Make a vec2 with the current mouse position
	glm::vec2 currentMousePosition(xpos, ypos);

	// Get the difference from the previous mouse position
//    glm::vec2 mousePositionDelta = currentMousePosition - m_mousePosition;

	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_LEFT]) {
		if (!control_held) {
			for (auto &b: m_skeleton.m_bones) {
				if (&b == selected_bone) {
					b.rotation[current_axis] += (xpos - m_mousePosition.x) / 20;
				}
			}
		}
	}
	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_MIDDLE]) {
		apply_arcball(currentMousePosition);
	}
	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_RIGHT]) {
	}


	// Update the mouse position to the current one
	m_mousePosition = currentMousePosition;
}

void Application::onKey(int key, int scancode, int action, int mods) {
	(void) scancode;
	(void) action;
	(void) mods;
	if (key == GLFW_KEY_A && action == GLFW_PRESS) {
		play = true;
	}
	if (key == GLFW_KEY_S && action == GLFW_PRESS) {
		save = true;
	}
	if (key == GLFW_KEY_X && action == GLFW_PRESS) {
		current_axis ++;
		if (current_axis > 2) {
			current_axis = 0;
		}
	}
	control_held = mods == GLFW_MOD_CONTROL;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#else
	//Glfw seemed to invert itself if no other key is held
	if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) {
		control_held = mods != GLFW_MOD_CONTROL;
	}
#endif
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