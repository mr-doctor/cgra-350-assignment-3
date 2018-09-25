#include "bone.hpp"
namespace cgra {

	void Bone::connect_child(Bone child_bone) {
		for (auto child : children) {
			if (child_bone.id == child.id) {
				return;
			}
		}
		children.push_back(child_bone);
	}

	Bone Bone::get_bone(std::string bone_name) {
		for (auto child : children) {
			if (child.name == bone_name) {
				return child;
			}
		}
		return cgra::Bone(false, -1, "", glm::vec3(0), 0.0f, glm::vec3(0));;
	}

	Bone Bone::get_bone(int bone_id) {
		for (auto child : children) {
			if (child.id == bone_id) {
				return child;
			}
		}
		return cgra::Bone(false, -1, "", glm::vec3(0), 0.0f, glm::vec3(0));;
	}

}