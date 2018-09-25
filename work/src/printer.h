//
// Created by Daniel on 26/08/2018.
//

#ifndef CGRA_PROJECT_A1_PRINTER_H
#define CGRA_PROJECT_A1_PRINTER_H


#include "skeleton.hpp"

class printer {
public:
	static void print(bone bone);

	static void print(Skeleton skeleton);

	static void print(float f);

	static void print(glm::vec3 vector);

	static void print(const std::string &string);

	static void print(glm::mat4 matrix);
};


#endif //CGRA_PROJECT_A1_PRINTER_H
