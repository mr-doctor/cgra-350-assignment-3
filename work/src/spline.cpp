
#include "spline.hpp"
#include "printer.h"

glm::vec3 Spline::map(float u) {
	float target = u * length;

	int index = static_cast<int>(std::lower_bound(arc_len.begin(), arc_len.end(), target) - arc_len.begin());

	if (arc_len[index] > target) {
		index--;
	}

	float len_prev = arc_len[index];
	if (len_prev == target) {
		return catmull(index / (float) points);
	} else {
		return catmull((index + (target - len_prev) / (arc_len[index + 1] - len_prev)) / (float) points);
	}
}

glm::vec3 Spline::catmull(float t) {
	float index;
	float t_p = std::modf(t * ((control_points.size()-2) - 1), &index);

	int index_p = ((int) index) + 1;

	return glm::catmullRom(control_points[index_p - 1],
	                       control_points[index_p],
	                       control_points[index_p + 1],
	                       control_points[index_p + 2], t_p);
}