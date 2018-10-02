//
// Created by Daniel on 26/08/2018.
//

#include <iostream>
#include <glm/vec3.hpp>
#include "printer.h"
#include "skeleton.hpp"

void printer::print(const std::string &string) {
	std::cout << string << "\n" << std::flush;
}

void printer::print(glm::mat4 matrix) {
	int row_sum;
	for (int i = 0; i < 4; i++) {
		row_sum = 0;
		for (int j = 0; j < 4; j++) {
			//print the row of first matrix
			std::cout << matrix[i][j] << "   ";
			row_sum += matrix[i][j];

		}
		std::cout << std::endl;
	}
}

void printer::print(glm::vec3 vector) {
	std::cout << "[" << vector.x << ", " << vector.y << ", " << vector.z << "]" << std::endl;
}

void printer::print(float f) {
	std::cout << f << std::endl;
}

/*void printer::print(bool b) {
	std::cout << ((b) ? "true" : "false") << std::endl;
}*/

void printer::print(Skeleton skeleton) {
	for (const auto &bone : skeleton.m_bones) {
		print(bone);
	}
}

void printer::print(bone bone) {
	std::string children = " { \n";
	for (auto child : bone.children) {
		children += "   " + child->name + "\n";
	}
	children += "}";
	std::cout << bone.name << " - " << bone.id << ((bone.children.empty()) ? "" : children) << std::endl;
}