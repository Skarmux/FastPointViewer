#pragma once
#include <glm/glm.hpp>

class Plane
{
public:
	
	Plane(void) {};

	void set3Points(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{

		glm::vec3 v = p1 - p0;
		glm::vec3 u = p2 - p0;

		this->normal = glm::normalize(glm::cross(v, u));

		this->d = -glm::dot(normal,p0);
	}

	// returns the distance between a point and this plane
	float distance(glm::vec3 point) 
	{
		// ax + by + cz + d equals 0 when point (x,y,z) is directly on the plane
		return (normal.x * point.x) + (normal.y * point.y) + (normal.z * point.z) + d;
	}

private:

	glm::vec3 normal;
	float d;

};

