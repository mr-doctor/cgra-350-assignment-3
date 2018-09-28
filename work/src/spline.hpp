#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <array>
#include <glm/gtx/spline.hpp>
#include <glm/gtx/string_cast.hpp>

#include "opengl.hpp"
#include "glm/glm.hpp"
#include "printer.h"

class Spline {
public:
	std::vector<float> arc_len;
	int points;
	float length;
	std::vector<glm::vec3> control_points;

	Spline(std::vector<glm::vec3> controls, int num_points) {

		control_points = std::move(controls);

		points = num_points;

		arc_len.push_back(0.0f);

		glm::vec3 o_point = catmull(0.0f);
		float net_len = 0.0f;
		for (int i = 1; i <= num_points; i++) {
			glm::vec3 cur_point = catmull(((float)i)  / ((float)num_points));
			cur_point.z = 0;
			glm::vec3 d_point(o_point - cur_point);

			net_len += glm::distance(o_point, cur_point);

			arc_len.push_back(net_len);

			o_point = cur_point;
		}
		length = net_len;
	}

	glm::vec3 catmull(float t);

	glm::vec3 map(float u);

};