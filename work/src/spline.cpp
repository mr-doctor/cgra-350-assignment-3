
#include "spline.hpp"
#include "printer.h"

glm::vec3 Spline::map(float u) {
	float target = u * arc_len[len];

	int index = static_cast<int>(std::lower_bound(arc_len.begin(), arc_len.end(), target) - arc_len.begin());

	if (arc_len[index] > target) {
		index--;
	}

	float len_prev = arc_len[index];
	if (len_prev == target) {
		return glm::catmullRom(A, B, C, D, ((float) index) / ((float) len));
	} else {
		return glm::catmullRom(A, B, C, D, (((float) index) + (target - len_prev) / (arc_len[index + 1] - len_prev)) / ((float) len));
	}
}