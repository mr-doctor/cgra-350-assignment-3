#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <array>
#include <glm/gtx/spline.hpp>

#include "opengl.hpp"
#include "glm/glm.hpp"

class Spline {
public:
	glm::vec3 A;
	glm::vec3 B;
	glm::vec3 C;
	glm::vec3 D;
	std::vector<float> arc_len;
	int points;
	float length;

	Spline(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, int num_points) {
		A = a;
		B = b;
		C = c;
		D = d;

		points = num_points;

		arc_len.push_back(0.0f);

		glm::vec3 o_point = glm::catmullRom(a, b, c, d, 0.0f);
		float net_len = 0.0f;

		for (int i = 1; i <= num_points; i++) {
			glm::vec3 cur_point = glm::catmullRom(a, b, c, d, ((float)i)  / ((float)num_points));
			cur_point.z = 0;
			glm::vec3 d_point(o_point - cur_point);

			net_len += glm::distance(o_point, cur_point);

			arc_len.push_back(net_len);

			o_point = cur_point;
		}
		length = net_len;
	}

	glm::vec3 map(float u);

};