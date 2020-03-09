#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

#include "Plane.h"

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	UP,
	DOWN
};

enum Frustum_Culling {
	INSIDE,
	OUTSIDE,
	INTERSECT
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 0.2f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;
const float ASPECT = 800.0f / 600.0f;
const float NEAR_DIST = 0.01f;
const float FAR_DIST = 100.0f;

class Camera
{
public:

	// Constructor with vectors
	Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),float yaw = YAW,float pitch = PITCH,float aspect = ASPECT) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
	{
		Position	= position;
		WorldUp		= up;
		Yaw			= yaw;
		Pitch		= pitch;
		Aspect		= aspect;

		Hnear = 2.0f * (float) tan(glm::radians(Zoom) / 2.0f) * NEAR_DIST;
		Wnear = Hnear * Aspect;
		Hfar = 2.0f * (float) tan(glm::radians(Zoom) / 2.0f) * FAR_DIST;
		Wfar = Hfar * Aspect;

		updateCameraVectors();
	}

	// Returns the view matrix calculated using Euler Angles and the LookAt Matrix
	glm::mat4 getViewMatrix()
	{
		//return glm::lookAt(Position, glm::vec3(0.0f), Up);
		return glm::lookAt(Position, Position + Front, Up);
	}

	// Returns the perspective projection matrix
	glm::mat4 getPerspectiveMatrix()
	{
		return glm::perspective(glm::radians(Zoom), Aspect, NEAR_DIST, FAR_DIST);
	}

	// Enable position updates for benchmark runs
	void setPosition(glm::vec3 pos)
	{
		Position = pos;
	}

	// return true if point is within visible area
	bool pointInFrustum(glm::vec3 point)
	{
		bool result = true;
		float distance;

		for (int i = 0; i < 6; i++) {
			distance = planes[i].distance(point);
			if (distance < 0) {
				return false;
			}
		}

		return result;
	}

	// return if sphere is within visible area
	int sphereInFrustum(glm::vec3 point, float radius)
	{
		int result = INSIDE;
		float distance;

		for (int i = 0; i < 6; i++) {
			distance = planes[i].distance(point);
			if (distance < -radius) {
				return OUTSIDE;
			}
			else if (distance < radius){
				result = INTERSECT;
			}
		}

		return result;
	}

	float distance(glm::vec3 point)
	{
		return glm::distance(Position, point);
	}

	// Processes input received from any keyboard-like input system. 
	// Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
	void ProcessKeyboard(Camera_Movement direction, float deltaTime)
	{
		float velocity = MovementSpeed * deltaTime;
		if (direction == FORWARD)
			Position += Front * velocity;
		if (direction == BACKWARD)
			Position -= Front * velocity;
		if (direction == LEFT)
			Position -= Right * velocity;
		if (direction == RIGHT)
			Position += Right * velocity;
		if (direction == UP)
			Position += WorldUp * velocity;
		if (direction == DOWN)
			Position -= WorldUp * velocity;

		updateCameraVectors();
	}

	// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
	void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
	{
		xoffset *= MouseSensitivity;
		yoffset *= MouseSensitivity;

		Yaw += xoffset;
		Pitch += yoffset;

		// Make sure that when pitch is out of bounds, screen doesn't get flipped
		if (constrainPitch)
		{
			if (Pitch > 89.9f)
				Pitch = 89.9f;
			if (Pitch < -89.9f)
				Pitch = -89.9f;
		}

		// Update Front, Right and Up Vectors using the updated Euler angles
		updateCameraVectors();
	}

	void setFrustumVertices(float * vertices) {
		
		float frustum[] = {
			ftl.x, ftl.y, ftl.z, 1.0f, 0.0f, 0.0f,
			ftr.x, ftr.y, ftr.z, 1.0f, 0.0f, 0.0f,
			fbl.x, fbl.y, fbl.z, 1.0f, 0.0f, 0.0f,
			fbr.x, fbr.y, fbr.z, 1.0f, 0.0f, 0.0f,
			ntl.x, ntl.y, ntl.z, 1.0f, 0.0f, 0.0f,
			ntr.x, ntr.y, ntr.z, 1.0f, 0.0f, 0.0f,
			nbl.x, nbl.y, nbl.z, 1.0f, 0.0f, 0.0f,
			nbr.x, nbr.y, nbr.z, 1.0f, 0.0f, 0.0f
		};

		memcpy(vertices, frustum, sizeof(frustum));
	}

private:

	// Camera Attributes
	glm::vec3 Position;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	// Euler Angles
	float Yaw;
	float Pitch;
	float Aspect;
	// Camera options
	float MovementSpeed;
	float MouseSensitivity;
	float Zoom;
	// Frustum
	glm::vec3 ntl, ntr, nbl, nbr;
	glm::vec3 ftl, ftr, fbl, fbr;
	float Hnear, Wnear, Hfar, Wfar;

	enum { L, R, T, B, F, N };
	Plane planes[6];

	// Calculates the front vector from the Camera's (updated) Euler Angles
	void updateCameraVectors()
	{
		// Calculate the new Front vector
		glm::vec3 front;
		front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
		front.y = sin(glm::radians(Pitch));
		front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
		Front	= glm::normalize(front);

		// Also re-calculate the Right and Up vector
		Right	= glm::normalize(glm::cross(Front, WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
		Up		= glm::normalize(glm::cross(Right, Front));

		// Update View Frustum Planes
		glm::vec3 nearCenter = Position + Front * NEAR_DIST;
		ntl = nearCenter + (Up * Hnear / 2.0f) - (Right * Wnear / 2.0f);
		ntr = ntl + (Right*Wnear);
		nbr = ntr - (Up*Hnear);
		nbl = nbr - (Right*Wnear);

		glm::vec3 farCenter = Position + Front * FAR_DIST;
		ftl = farCenter + (Up * Hfar / 2.0f) - (Right * Wfar / 2.0f);
		ftr = ftl + (Right*Wfar);
		fbr = ftr - (Up*Hfar);
		fbl = fbr - (Right*Wfar);		

		planes[T].set3Points(ntr, ntl, ftl); // top
		planes[B].set3Points(nbl, nbr, fbr); // bottom
		planes[L].set3Points(ntl, nbl, fbl); // left
		planes[R].set3Points(nbr, ntr, fbr); // right
		planes[F].set3Points(ftr, ftl, fbl); // far
		planes[N].set3Points(ntl, ntr, nbr); // near
	}

};
