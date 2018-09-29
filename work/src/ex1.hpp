#pragma once

#include <cgra/bone.hpp>
#include "cgra/mesh.hpp"
#include "cgra/shader.hpp"

#include "glm/glm.hpp"
#include "skeleton.hpp"

class Application {
public:
    // The window object managed by GLFW
    GLFWwindow *m_window;

    // The shader program used for drawing
    static cgra::Program m_program;
    // The mesh data
    cgra::Mesh m_mesh;

    // The current size of the viewport
    glm::vec2 m_viewportSize;
    // The current mouse position
    glm::vec2 m_mousePosition;

    // The translation of the mesh as a vec3
    glm::vec3 m_translation;
    // The scale of the mesh
    float m_scale;

    // A 4x4 matrix representing the rotation of the
    // mesh
    glm::mat4 m_rotationMatrix;

    // Whether or not the left, middle or right buttons are down.
    bool m_mouseButtonDown[3];

    Application(GLFWwindow *win)
        : m_window(win),
          m_viewportSize(1, 1), m_mousePosition(0, 0),
          m_translation(0), m_scale(1), m_rotationMatrix(1) {
        m_mouseButtonDown[0] = false;
        m_mouseButtonDown[1] = false;
        m_mouseButtonDown[2] = false;
    }

    void setWindowSize(int width, int height) {
        m_viewportSize.x = float(width);
        m_viewportSize.y = float(height);
    }

    void init();

    void drawScene();
    void doGUI();

    void onKey(int key, int scancode, int action, int mods);

    void onMouseButton(int button, int action, int mods);

    void onCursorPos(double xpos, double ypos);

    void onScroll(double xoffset, double yoffset);

	void apply_arcball(glm::vec2 current_mouse_XY);

	void set_shaders(const char *vertex, const char *fragment);

	cgra::Mesh loadObj(const char *filename, glm::vec3 colour);

	static cgra::Mesh m_bone_mesh;
	static cgra::Mesh m_sphere_mesh_cyan;
	cgra::Mesh m_sphere_mesh_yellow;
	static cgra::Mesh m_sphere_mesh_red;
	static cgra::Mesh m_sphere_mesh_green;
	cgra::Mesh m_cube_mesh;
	static cgra::Mesh m_bone_segment_mesh;

	static void draw_bone(cgra::Mesh mesh, glm::vec3 scale, glm::mat4 rotate,
	                      glm::vec3 global_translation,
	                      glm::vec3 global_scale, glm::mat4 global_rotation);

	static void draw(cgra::Mesh mesh, glm::vec3 scale, glm::mat4 model_transform);


	static void draw(cgra::Mesh mesh, glm::vec3 position, glm::vec3 scale, glm::mat4 rotate, glm::vec3 global_translation,
					 glm::vec3 global_scale, glm::mat4 global_rotation);

	std::vector<glm::vec3> keyframes;
	std::vector<glm::vec3> speed_curve;

	void show_spline(std::vector<glm::vec3> &controls, int num_points, std::vector<glm::vec3> &points, bool main_spline);

	std::vector<glm::vec3> new_points;
	std::vector<glm::vec3> speed_points;

	void update_position();

	float point_index = 0;
	float speed = 1.0;
	float m_depth = -1;


	int selected = -1;

	glm::vec3 screen_to_world_coord(double mouse_x, double mouse_y);

	glm::mat4 m_view;
	glm::mat4 m_proj;
	static glm::mat4 m_model;

	void manipulate(glm::vec3 mouse_point);

	void update_spline();

	bool left_held = false;

	void update_speed_spline();

	bool select_keyframe = false;
	int catmull_divisions = 100;
};
