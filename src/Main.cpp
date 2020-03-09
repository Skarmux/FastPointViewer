
#include "FPVConfig.h"
#include "Camera.h"
#include "Shader.h"
//#include "Loader.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <map>
#include <tuple>
#include <chrono>
#include <mutex>
#include <thread>

#include <math.h>
#include <stdio.h>

#define BOOST_DATE_TIME_NO_LIB
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace std;

const float * datamap_ptr;
const float * treemap_ptr;

// CONFIGURATION
char tree_filepath[] = "/home/skarmux/Documents/Projects/FastPointViewer/res/models/lucy_SPHERE_B20.bin";
char data_filepath[] = "/home/skarmux/Documents/Projects/FastPointViewer/res/models/lucy_DATA.bin";
char vertex_shader[] = "/home/skarmux/Documents/Projects/FastPointViewer/res/shaders/4.3.shader.vs";
char fragment_shader[] = "/home/skarmux/Documents/Projects/FastPointViewer/res/shaders/4.3.shader.fs";
const bool use_lod = true;
const float lod_factor = 1.9f;
const float first_lod_dist = pow(lod_factor, 4) / 100;
const float second_lod_dist = pow(lod_factor, 5) / 100;
const int szLoadBuffer = 50 * MEGABYTE; //size of front and back buffer

// BUFFER
float oviewPoints[SCR_WIDTH * SCR_HEIGHT * 3];
const float* ptr_fnt;
const float* ptr_bck;
const float* ptr_frustum;
mutex fnt_buffer_mtx;
mutex bck_buffer_mtx;

// vertex buffer object ID's
unsigned int VBOs[5], VAOs[5], EBOs[5];

// enums
enum State { NORMAL, LOCKED, BENCHMARK };
enum BVType { SPHERE, AABB };
int state = NORMAL;
int max_depth;
int num_elements;
int bvtype;

// debugging values and time
int nbOviewPoints = 0;
int nbLoadPoints = 0;
float deltaTime = 0.0f;
float lastFrame = 0.0f;
float buffertime = 0.0f;
float max_buffertime = 0.0f;
float avg_buffertime = 0.0f;
unsigned int buffer_use = 0;
unsigned int max_buffer_use = 0;
unsigned int ctr_buffertime = 0;

// initialize camera
Camera camera = Camera(glm::vec3(0.0f, 0.0f, 1.2f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

bool worker_active;
bool worker_paused;

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

//Loader loader(treemap_ptr, datamap_ptr, ptr_fnt, ptr_bck, &fnt_buffer_mtx, &bck_buffer_mtx, &camera, &nbLoadPoints);
//thread loaderThread(loader);
//thread loaderThread;

// ------------------------------------------------------------------------------------------------------------------------

// process all input: query GLFW whether relevant keys are pressed/released this frame
// and react accordingly
void processInput(GLFWwindow* window)
{
	// window events
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		worker_active = false;
		glfwSetWindowShouldClose(window, true);
	}

	// toggle frustum display
	if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_PRESS)
	{
		// [F3] Benchmark
		state = BENCHMARK;
		camera = Camera(glm::vec3(0.0f, 0.0f, 0.15f));
		avg_buffertime = buffertime;
	}

	if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS)
	{
		// [F2] Lock View
		if (state != LOCKED) {
			camera.setFrustumVertices((float*)ptr_frustum);
		}
		state = LOCKED;
	}

	if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS)
	{
		// [F1] Normal
		state = NORMAL;
	}

	// camera movement
	if (state != BENCHMARK)
	{
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
			camera.ProcessKeyboard(FORWARD, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
			camera.ProcessKeyboard(BACKWARD, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
			camera.ProcessKeyboard(LEFT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
			camera.ProcessKeyboard(RIGHT, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
			camera.ProcessKeyboard(UP, deltaTime);
		if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
			camera.ProcessKeyboard(DOWN, deltaTime);
	}
}

// glfw: whenever the mouse moves, this callback is called
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (state != BENCHMARK) {
		if (firstMouse)
		{
			lastX = xpos;
			lastY = ypos;
			firstMouse = false;
		}

		float xoffset = xpos - lastX;
		float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

		lastX = xpos;
		lastY = ypos;

		camera.ProcessMouseMovement(xoffset, yoffset);
	}
}

float* load_oview(float* tree_ptr, float * buffer_ptr, int height) {

	{ // Load point into buffer
		int point_offset; // is point index times three
		memcpy(&point_offset, (tree_ptr + 1), sizeof(int));
		memcpy(buffer_ptr, (datamap_ptr + point_offset), sizeof(float) * 3);
		buffer_ptr += 3;
	}

	// travel down the tree
	if (height > 0) {
		int l_off, r_off;
		memcpy(&l_off, (tree_ptr + 2), sizeof(int));
		memcpy(&r_off, (tree_ptr + 3), sizeof(int));
		buffer_ptr = load_oview((float*)(treemap_ptr + l_off), buffer_ptr, height - 1);
		buffer_ptr = load_oview((float*)(treemap_ptr + r_off), buffer_ptr, height - 1);
	}

	return buffer_ptr;
}

float* read_kdtree(float* tree_ptr, float * buffer_ptr, int depth) {

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
			memcpy(buffer_ptr, (datamap_ptr + point_offset), sizeof(float) * 3);
		}
		glm::vec3 point_scaled = transformation_matrix * glm::vec4(buffer_ptr[0], buffer_ptr[1], buffer_ptr[2], 1.0f);
		float radius = first_val;
		float radius_scaled = radius * model_scaling_factor;

		// Check if sphere is inside/outside/intersecting the frustum view
		int result = camera.sphereInFrustum(point_scaled, radius_scaled);

		if (result == OUTSIDE)
		{
			return buffer_ptr;
		}
		else if (use_lod)
		{
			// apply level of detail checks
			// current LOD technique can't be combined with the inside flag

			float distance = camera.distance(point_scaled);

			if (depth > 18) {
				if (distance - radius_scaled > first_lod_dist) {
					return buffer_ptr; // OUTSIDE
				}
				else if (distance + radius_scaled > first_lod_dist) {
					result = INTERSECT;
				}
			}
			else if (depth > 14) {
				if (distance - radius_scaled > second_lod_dist) {
					return buffer_ptr; // OUTSIDE
				}
				else if (distance + radius_scaled > second_lod_dist) {
					result = INTERSECT;
				}
			}
		}

		if (depth < max_depth)
		{
			// progress further into recursion
			int l_off, r_off;
			memcpy(&l_off, (tree_ptr + 2), sizeof(int));
			memcpy(&r_off, (tree_ptr + 3), sizeof(int));
			buffer_ptr[3] = 0.0f;
			buffer_ptr[4] = 1.0f;
			buffer_ptr[5] = 0.0f;
			buffer_ptr += 6;

			buffer_ptr = read_kdtree((float*)treemap_ptr + l_off, buffer_ptr, depth + 1);
			buffer_ptr = read_kdtree((float*)treemap_ptr + r_off, buffer_ptr, depth + 1);
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
			memcpy(buffer_ptr, (datamap_ptr + point_offset), sizeof(float) * 3);
			buffer_ptr[3] = 0.0f;
			buffer_ptr[4] = 1.0f;
			buffer_ptr[5] = 0.0f;
			buffer_ptr += 6;
		}
	}

	return buffer_ptr;
}

void treeWorker() {

	worker_active = true;
	worker_paused = false;
	float * ptr = (float*) ptr_fnt;
	float start_time;

	while (worker_active) {
		if (state != LOCKED)
		{
			// fill front buffer
			fnt_buffer_mtx.lock();
			start_time = glfwGetTime();
			ptr = read_kdtree((float *)treemap_ptr + 3, ptr, 0);
			nbLoadPoints = (ptr - ptr_fnt) / 6;
			ptr = (float*)ptr_bck;
			buffertime = glfwGetTime() - start_time;
			ctr_buffertime += 1;
			avg_buffertime += (buffertime - avg_buffertime) / ctr_buffertime;
			fnt_buffer_mtx.unlock();

			// fill back buffer
			bck_buffer_mtx.lock();
			start_time = glfwGetTime();
			ptr = read_kdtree((float *)treemap_ptr + 3, ptr, 0);
			nbLoadPoints = (ptr - ptr_bck) / 6;
			ptr = (float*)ptr_fnt;
			buffertime = glfwGetTime() - start_time;
			ctr_buffertime += 1;
			avg_buffertime += (buffertime - avg_buffertime) / ctr_buffertime;
			bck_buffer_mtx.unlock();
		}
	}
}

int main() {
	// glf context setup
	GLFWwindow* window;
	{
		// glfw: initialize and configure
		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4); // maximum OpenGL version
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); // minimum OpenGL version
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

		// glfw: window creation
		//window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FastPointViewer", glfwGetPrimaryMonitor(), NULL);
		window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "FastPointViewer", NULL, NULL);
		if (window == NULL) {
			std::cout << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return -1;
		}

		glfwMakeContextCurrent(window);
		glfwSetCursorPosCallback(window, mouse_callback);

		// tell GLFW to not capture our mouse cursor
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	}

	// glad: load all OpenGL function pointers
	{
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
			return -1;
		}
	}
		
	// build and compile shader programs
	Shader ourShader( vertex_shader, fragment_shader);

	// Open files for memory mapping
	boost::interprocess::file_mapping tree_filemap(tree_filepath, boost::interprocess::read_only);
	boost::interprocess::file_mapping data_filemap(data_filepath, boost::interprocess::read_only);
	boost::interprocess::mapped_region tree_region(tree_filemap, boost::interprocess::read_only);
	boost::interprocess::mapped_region data_region(data_filemap, boost::interprocess::read_only);
	treemap_ptr = static_cast<const float*>(tree_region.get_address());
	datamap_ptr = static_cast<const float*>(data_region.get_address());
	memcpy(&max_depth, treemap_ptr, sizeof(int));
	memcpy(&num_elements, treemap_ptr + 1, sizeof(int));
	memcpy(&bvtype, treemap_ptr + 2, sizeof(int));
	// generate array/vertex/element buffer objects
	glGenVertexArrays(5, VAOs);
	glGenBuffers(5, VBOs);
	glGenBuffers(5, EBOs);

	// VAO[0] :: overview point cloud
	{
		//int tree_level = log2 (SCR_WIDTH*SCR_HEIGHT);
		int tree_level = 14; // manually set to 14
		float * points_end = load_oview((float*) treemap_ptr + 3, oviewPoints, tree_level-1);
		nbOviewPoints = (points_end - oviewPoints) / 3;
		glBindVertexArray(VAOs[0]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(oviewPoints), oviewPoints, GL_STATIC_DRAW);
		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
	}

	// VAO[1-2] Loaded Point Cloud Front/Back Buffer initialization
	{
		glBindVertexArray(VAOs[1]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]);
		glBufferData(GL_ARRAY_BUFFER, szLoadBuffer, NULL, GL_DYNAMIC_DRAW);
		ptr_fnt = (float *) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		glUnmapBuffer(GL_ARRAY_BUFFER);
		// position attributes
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// color attributes
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		glBindVertexArray(VAOs[2]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[2]);
		glBufferData(GL_ARRAY_BUFFER, szLoadBuffer, NULL, GL_DYNAMIC_DRAW);
		ptr_bck = (float *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		glUnmapBuffer(GL_ARRAY_BUFFER);
		// position attributes
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// color attributes
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}
	
	// VAO[4] Frustum Outlines
	{
		glBindVertexArray(VAOs[4]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[4]);
		glBufferData(GL_ARRAY_BUFFER, 48*sizeof(float), NULL, GL_DYNAMIC_DRAW);
		ptr_frustum = (float *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		glUnmapBuffer(GL_ARRAY_BUFFER);

		unsigned int indices[] = {
			4,5,5,7,7,6,6,4,
			0,1,1,3,3,2,2,0,
			0,4,1,5,
			2,6,3,7
		};
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[4]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// enable use of z buffering
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	// launch subthread
	thread loaderThread(treeWorker);

	//---------------------------------------------------------------------------------------------

	// RENDER LOOP
	
	while (!glfwWindowShouldClose(window))
	{
		// per-frame time logic
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// input handling (also makes the camera update)
		processInput(window);

		// render background color
		glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// steadily rotate model during benchmark
		if (state == BENCHMARK) {
			glm::mat4 rotate_matrix = glm::rotate(glm::mat4(1.0f), deltaTime/2.0f, glm::vec3(0.0f,1.0f,0.0f));
			transformation_matrix = rotate_matrix * transformation_matrix;
		}

		// activate main shader
		ourShader.use();

		// pass projection matrix to shader
		glm::mat4 projection = camera.getPerspectiveMatrix();
		ourShader.setMat4("projection", projection);

		// camera/view transformation
		glm::mat4 view = camera.getViewMatrix();
		ourShader.setMat4("view", view);

		// VAO[0] draw overview point cloud
		{
			ourShader.setMat4("model", transformation_matrix);
			glBindVertexArray(VAOs[0]);
			glDrawArrays(GL_POINTS, 0, nbOviewPoints);
		}

		// VAO[1/2] draw loaded detail point cloud
		{
			if (fnt_buffer_mtx.try_lock()) {
				glBindVertexArray(VAOs[1]);
				glDrawArrays(GL_POINTS, 0, nbLoadPoints);
				fnt_buffer_mtx.unlock();
			}
			else {
				bck_buffer_mtx.lock();
				glBindVertexArray(VAOs[2]);
				glDrawArrays(GL_POINTS, 0, nbLoadPoints);
				bck_buffer_mtx.unlock();
			}
			cout << nbOviewPoints << endl;
		}

		// draw Frustum Frame
		if (state == LOCKED) {
			ourShader.setMat4("model", glm::mat4(1.0f));
			glBindVertexArray(VAOs[4]);
			glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
		}

		// reset VAO (optional)
		glBindVertexArray(0);

		// glfw: swap buffers (back/front) and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();

	loaderThread.join();

	return 0;
}

