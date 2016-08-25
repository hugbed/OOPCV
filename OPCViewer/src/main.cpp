#include <iostream>

// todo : this is ugly
#define _WIN
#define _VR

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>
#include <assert.h>
#include <Shader.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Mesh.h>

#include "minimalOpenGL.h"

// VR
#ifdef _VR
#include "minimalOpenVR.h"
#endif

#include "Camera.h"
#include "FileIO.h"
#include "PointCloudSubscriber.h"

#define PI (3.1415927f)

using std::cout;
using std::endl;

struct TimeTracker {
	GLfloat deltaTime = 0.0f;
	GLfloat lastFrameTime = 0.0f;
} timeTracker;

struct InputTracker {
	bool keys[1024];
	GLfloat g_lastX = 400, g_lastY = 300;
	bool g_firstMouse = true;
} inputTracker;

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

#ifdef _VR
vr::IVRSystem* hmd = nullptr;
#endif

int main();

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void doMovement();

int main() {
    std::cout << "Starting GLFW context, OpenGL 3.3" << std::endl;

	uint32_t framebufferWidth = 1280, framebufferHeight = 720;
# ifdef _VR
	const int numEyes = 2;
	hmd = initOpenVR(framebufferWidth, framebufferHeight);
	assert(hmd);
# else
	const int numEyes = 1;
# endif

	GLFWwindow *window = nullptr;
	const int windowHeight = 720;
	const int windowWidth = (framebufferWidth * windowHeight) / framebufferHeight;
    if (!GL::initWindow(windowWidth, windowHeight, window)) return -1;
    if (!GL::initGLEW()) return -1;
	GL::initViewport(window);

	/////////////////////////////////////////////////////////////////
	// Set the required callback functions
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	//////////////////////////////////////////////////////////////////////
	// Allocate the frame buffer. This code allocates one framebuffer per eye.
	// That requires more GPU memory, but is useful when performing temporal 
	// filtering or making render calls that can target both simultaneously.
	GLuint framebuffer[numEyes];
	GLuint colorRenderTarget[numEyes], depthRenderTarget[numEyes];
	GL::initFrameBuffersAndTextures(numEyes, framebufferWidth, framebufferHeight, framebuffer, colorRenderTarget, depthRenderTarget);

	/////////////////////////////////////////////////////////////////
	// Load vertex array buffers
	Mesh* mesh = nullptr;

	glm::vec3 trianglePositions[] = {
		glm::vec3(0.0f,  0.0f,  0.0f),
		glm::vec3(2.0f,  5.0f, -15.0f),
		glm::vec3(-1.5f, -2.2f, -2.5f),
		glm::vec3(-3.8f, -2.0f, -12.3f),
		glm::vec3(2.4f, -0.4f, -3.5f),
		glm::vec3(-1.7f,  3.0f, -7.5f),
		glm::vec3(1.3f, -2.0f, -2.5f),
		glm::vec3(1.5f,  2.0f, -2.5f),
		glm::vec3(1.5f,  0.2f, -1.5f),
		glm::vec3(-1.3f,  1.0f, -1.5f)
	};

	PointCloudSubscriber subscriber("10.51.87.103");

	/////////////////////////////////////////////////////////////////////
	// Create the main shader
	Shader shader((FileIO::getCurrentDirectory() + "/shaders/points.vs").c_str(), (FileIO::getCurrentDirectory() + "/shaders/points.fs").c_str());

	/////////////////////////////////////////////////////////////////////
	// OpenGL settings
    glClearColor(0.f, 0.f, 0.f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);

	/////////////////////////////////////////////////////////////////////
	// Camera constants
	const float nearPlaneZ = -0.1f;
	const float farPlaneZ = -100.0f;
	const float verticalFieldOfView = 45.0f * PI / 180.0f;

# ifdef _VR
	vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
# endif

    // Game loop
    while (!glfwWindowShouldClose(window))
    {
		assert(glGetError() == GL_NONE);

		GLfloat currentFrameTime = (GLfloat)glfwGetTime();
		timeTracker.deltaTime = currentFrameTime - timeTracker.lastFrameTime;
		timeTracker.lastFrameTime = currentFrameTime;

		// Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
		glfwPollEvents();
		doMovement();

		/////////////////////////////////////////////////////////////////////
		// Update transforms
		glm::mat4 projection[numEyes], headToEye[numEyes], bodyToHead;

#   ifdef _VR
		getEyeTransformations(hmd, trackedDevicePose, numEyes, nearPlaneZ, farPlaneZ, projection, headToEye, bodyToHead);
#   else
		projection[0] = glm::perspective(verticalFieldOfView, (float)framebufferWidth / (float)framebufferHeight, -nearPlaneZ, -farPlaneZ);
#   endif

		/////////////////////////////////////////////////////////////////////
		// Draw each eye on framebuffer texture
		for (int eye = 0; eye < numEyes; ++eye) {
			glBindFramebuffer(GL_FRAMEBUFFER, framebuffer[eye]);
			glViewport(0, 0, framebufferWidth, framebufferHeight);

			// Clear the colorbuffer
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#		ifdef _VR										   // world to camera body
			glm::mat4 view = headToEye[eye] * bodyToHead * glm::inverse(glm::translate(glm::mat4(), camera.Position));

			glm::vec3 right(view[0].x, -view[0].y, -view[0].z);
			glm::vec3 front(view[2].x, view[2].y, -view[2].z);
			glm::vec3 up(view[1]);
			camera.Up = up;
			camera.Front = front;
			camera.Right = right;
#		else
			const glm::mat4 view = camera.GetViewMatrix();
#		endif			

			shader.use();

			/////////////////////////////////////////////////////////////////////
			// Assign uniforms

			// View, projection
			GLint viewLoc = glGetUniformLocation(shader.program, "view");
			GLint projectionLoc = glGetUniformLocation(shader.program, "projection");
			glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection[eye]));
			glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

			// Point size
			int viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);
			GLint pointSizeLoc = glGetUniformLocation(shader.program, "pointSize");
			glUniform1f(pointSizeLoc, 0.005f);

			// Height of near plane
			float heightOfNearPlane = (float)abs(viewport[3] - viewport[1]) / (2 * (float)tan(0.5*camera.Zoom*PI / 180.0));
			GLint heightOfNearPlaneLoc = glGetUniformLocation(shader.program, "heightOfNearPlane");
			glUniform1f(heightOfNearPlaneLoc, heightOfNearPlane);

			/////////////////////////////////////////////////////////////////////
			// Draw models

			if (!subscriber.queue.empty()) {
				Mesh newMesh = subscriber.queue.pop();

				if (mesh == nullptr) {
					mesh = new Mesh(newMesh.vertices, newMesh.indices);
				}
				else {
					mesh->updateData(newMesh.vertices, newMesh.indices);
				}
			}

			// Draw triangles
			glm::mat4 model;
			model = glm::rotate(glm::mat4(), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
			GLint modelLoc = glGetUniformLocation(shader.program, "model");
			glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

			if (mesh != nullptr) {
				mesh->Draw(shader);
			}

			//for(GLuint i = 0; i < 10; i++)
			//{
			//	model = glm::translate(model, trianglePositions[i]);
			//	GLfloat angle = 20.0f * i;
			//	model = glm::rotate(model, angle, glm::vec3(1.0f, 0.3f, 0.5f));
			//	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
			//	glPointSize(10.0f);
			//	currentMesh.Draw(shader);
			//}

#		ifdef _VR
			{
				const vr::Texture_t tex = { reinterpret_cast<void*>(intptr_t(colorRenderTarget[eye])), vr::API_OpenGL, vr::ColorSpace_Gamma };
				vr::VRCompositor()->Submit(vr::EVREye(eye), &tex);
			}
#       endif

		} // for each eye

		////////////////////////////////////////////////////////////////////////
#	ifdef _VR
		// Tell the compositor to begin work immediately instead of waiting for the next WaitGetPoses() call
		vr::VRCompositor()->PostPresentHandoff();
#   endif

		// Mirror to the window
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, GL_NONE);
		glViewport(0, 0, windowWidth, windowHeight);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBlitFramebuffer(0, 0, framebufferWidth, framebufferHeight, 0, 0, windowWidth, windowHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, GL_NONE);

        // Swap the screen buffers
        glfwSwapBuffers(window);
    }

#   ifdef _VR
	if (hmd != nullptr) {
		vr::VR_Shutdown();
	}
#   endif

	delete mesh;

    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();

    return 0;
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    std::cout << key << std::endl;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (action == GLFW_PRESS)
		inputTracker.keys[key] = true;
    else if (action == GLFW_RELEASE)
		inputTracker.keys[key] = false;
}

void doMovement()
{
    if(inputTracker.keys[GLFW_KEY_W])
        camera.ProcessKeyboard(FORWARD, timeTracker.deltaTime);
    if(inputTracker.keys[GLFW_KEY_S])
        camera.ProcessKeyboard(BACKWARD, timeTracker.deltaTime);
    if(inputTracker.keys[GLFW_KEY_A])
        camera.ProcessKeyboard(LEFT, timeTracker.deltaTime);
    if(inputTracker.keys[GLFW_KEY_D])
        camera.ProcessKeyboard(RIGHT, timeTracker.deltaTime);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(inputTracker.g_firstMouse)
    {
		inputTracker.g_lastX = (GLfloat)xpos;
		inputTracker.g_lastY = (GLfloat)ypos;
		inputTracker.g_firstMouse = false;
    }

    GLfloat xoffset = (GLfloat)xpos - inputTracker.g_lastX;
    GLfloat yoffset = inputTracker.g_lastY - (GLfloat)ypos;
	inputTracker.g_lastX = (GLfloat)xpos;
	inputTracker.g_lastY = (GLfloat)ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll((GLfloat)yoffset);
}