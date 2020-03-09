
#include <mutex>

#include "FPVConfig.h"
#include "Camera.h"



using namespace std;

class Loader
{
private:
	const float* Data;
	const float* Tree;
	const float* FBuffer;
	const float* BBuffer;
	mutex* FBufferMtx;
	mutex* BBufferMtx;
	Camera* LCamera;
	int* NumberLoadedPoints;
	// LUCY DATASET HOTFIX
// scaling lucy model down into viewable size
	const float model_scaling_factor = 0.0005f;
	// build transformation matrix for lucy dataset
	glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), glm::vec3(model_scaling_factor));
	glm::mat4 rotate_matrix_a = glm::rotate(scale_matrix, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	glm::mat4 rotate_matrix_b = glm::rotate(rotate_matrix_a, glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	glm::mat4 transformation_matrix = glm::translate(
		rotate_matrix_b,
		glm::vec3(-690.7556152344, 121.5314025879, -192.6265869141));
public:
	bool isActive;

	Loader::Loader(const float* tree, const float* data,
		const float* fbuffer, const float* bbuffer,
		mutex* front_buffer_mtx, mutex* back_buffer_mtx,
		Camera* camera, int* nbLoadPoints)
	{
		Data	= data;
		Tree	= tree;
		FBuffer	= fbuffer;
		BBuffer = bbuffer;
		FBufferMtx = front_buffer_mtx;
		BBufferMtx = back_buffer_mtx;
		LCamera = camera;
		NumberLoadedPoints = nbLoadPoints;
	}

	void operator()() {
		isActive = true;
		float* ptr = (float*) FBuffer;
		while (isActive) {
			// fill front buffer
			FBufferMtx->lock();
			ptr = load_kdt((float *) Tree + 3, ptr, 0);
			*NumberLoadedPoints = (ptr - FBuffer) / 6;
			ptr = (float*) BBuffer;
			FBufferMtx->unlock();

			// fill back buffer
			BBufferMtx->lock();
			ptr = load_kdt((float *) Tree + 3, ptr, 0);
			*NumberLoadedPoints = (ptr - BBuffer) / 6;
			ptr = (float*) FBuffer;
			BBufferMtx->unlock();
		}
	}

	float* load_kdt(float* tree_ptr, float* buffer_ptr, int depth) {

		float first_val;
		memcpy(&first_val, tree_ptr, sizeof(float));

		if (first_val > 0)
		{
			// NODE

			// read node point (sphere center) (same as in oview function)
			{
				// load point into buffer
				int point_offset;
				memcpy(&point_offset, (tree_ptr + 1), sizeof(int));
				memcpy(buffer_ptr, (Data + point_offset), sizeof(float) * 3);
			}
			glm::vec3 point_scaled = transformation_matrix * glm::vec4(buffer_ptr[0], buffer_ptr[1], buffer_ptr[2], 1.0f);
			float radius = first_val;
			float radius_scaled = radius * model_scaling_factor;

			

			// Check if sphere is inside/outside/intersecting the frustum view
			int result = LCamera->sphereInFrustum(point_scaled, radius_scaled);

			if (result == OUTSIDE)
			{
				return buffer_ptr;
			}
			else if (use_lod)
			{
				// apply level of detail checks
				// current LOD technique can't be combined with the inside flag

				float distance = LCamera->distance(point_scaled);

				if (depth > 18) {
					if (distance - radius > first_lod_dist) {
						return buffer_ptr; // OUTSIDE
					}
					else if (distance + radius > first_lod_dist) {
						result = INTERSECT;
					}
				}
				else if (depth > 14) {
					if (distance - radius > second_lod_dist) {
						return buffer_ptr; // OUTSIDE
					}
					else if (distance + radius > second_lod_dist) {
						result = INTERSECT;
					}
				}
			}

			if (depth < 14) //max_depth
			{
				// progress further into recursion
				int l_off, r_off;
				memcpy(&l_off, (tree_ptr + 2), sizeof(int));
				memcpy(&r_off, (tree_ptr + 3), sizeof(int));
				buffer_ptr[3] = 0.0f;
				buffer_ptr[4] = 1.0f;
				buffer_ptr[5] = 0.0f;
				buffer_ptr += 6;

				buffer_ptr = load_kdt((float*) Tree + l_off, buffer_ptr, depth + 1);
				buffer_ptr = load_kdt((float*) Tree + r_off, buffer_ptr, depth + 1);
			}

		}
		else
		{
			// LEAF
			int size, point_offset;
			memcpy(&size, (tree_ptr + 1), sizeof(int));
			for (int i = 0; i < size; i++)
			{
				memcpy(&point_offset, (tree_ptr + 2 + (i)), sizeof(int));
				memcpy(buffer_ptr, (Data + point_offset), sizeof(float) * 3);
				buffer_ptr[3] = 0.0f;
				buffer_ptr[4] = 1.0f;
				buffer_ptr[5] = 0.0f;
				buffer_ptr += 6;
			}
		}

		return buffer_ptr;
	}
	
	
};
