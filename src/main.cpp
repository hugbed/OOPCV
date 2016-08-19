#include <iostream>

// todo : this is ugly
#define _WIN
#define _VR

#ifdef _WIN
#include <windows.h>
#endif

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

#include "matrix.h"
#include "Camera.h"

using std::cout;
using std::endl;

const GLuint WIDTH = 800, HEIGHT = 600;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
bool keys[1024];
GLfloat g_lastX = 400, g_lastY = 300;
bool g_firstMouse = true;

#ifdef _VR
vr::IVRSystem* hmd = nullptr;
#endif

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void doMovement();

void _check_gl_error() {
    GLenum err (glGetError());

    while(err!=GL_NO_ERROR) {
        std::string error;

        switch(err) {
            case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
            case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
            case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
            case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
            default:break;
        }

        std::cerr << "GL_" << error.c_str() << std::endl;
        err=glGetError();
    }
}

std::string getCurrentDirectory() {
#ifdef _WIN
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");
	return std::string(buffer).substr(0, pos);
#else
	return std::string(".")
#endif
}

bool initWindow(int windowWidth, int windowHeight, GLFWwindow *&window)
{
    // Init GLFW
    glfwInit();
    // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    // Create a GLFWwindow object that we can use for GLFW's functions
    window = glfwCreateWindow(windowWidth, windowHeight, "LearnOpenGL", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

	glfwMakeContextCurrent(window);
	
	// Negative numbers allow buffer swaps even if they are after the vertical retrace,
	// but that causes stuttering in VR mode
	glfwSwapInterval(0);

    return true;
}

bool initGLEW()
{
    // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
    glewExperimental = GL_TRUE;
    // Initialize GLEW to setup the OpenGL Function pointers
    if (glewInit() != GLEW_OK)
    {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return false;
    }
	// Clear startup errors
	while (glGetError() != GL_NONE) {}

    return true;
}

void initViewport(GLFWwindow *window)
{
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
}

Mesh createTriangleMesh()
{
    Vertex vertices[] = {
            // Positions                          // Colors
            { glm::vec3( 0.5f, -0.5f, 0.0f),  glm::vec3(1.0f, 0.0f, 0.0f) },  // Bottom Right
            { glm::vec3(-0.5f, -0.5f, 0.0f),  glm::vec3(0.0f, 1.0f, 0.0f) },  // Bottom Left
            { glm::vec3( 0.0f,  0.5f, 0.0f),  glm::vec3(0.0f, 0.0f, 1.0f) }   // Top
    };

    GLuint indices[] = {  // Note that we start from 0!
        0, 1, 2  // First Triangle
    };

    Mesh triangleMesh(std::vector<Vertex>(vertices, vertices + sizeof(vertices) / sizeof(vertices[0])),
                      std::vector<GLuint>(indices, indices + sizeof(indices) / sizeof(indices[0])));

    return triangleMesh;
}

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

	const int windowHeight = 720;
	const int windowWidth = (framebufferWidth * windowHeight) / framebufferHeight;
    GLFWwindow *window = nullptr;
    //if (!initWindow(windowWidth, windowHeight, window)) return -1;
	window = initOpenGL(windowWidth, windowHeight, "minimalOpenGL");

    // Set the required callback functions
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    if (!initGLEW()) return -1;

    initViewport(window);

	//////////////////////////////////////////////////////////////////////
	// Allocate the frame buffer. This code allocates one framebuffer per eye.
	// That requires more GPU memory, but is useful when performing temporal 
	// filtering or making render calls that can target both simultaneously.

	GLuint framebuffer[numEyes];
	glGenFramebuffers(numEyes, framebuffer);

	GLuint colorRenderTarget[numEyes], depthRenderTarget[numEyes];
	glGenTextures(numEyes, colorRenderTarget);
	glGenTextures(numEyes, depthRenderTarget);
	for (int eye = 0; eye < numEyes; ++eye) {
		glBindTexture(GL_TEXTURE_2D, colorRenderTarget[eye]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, framebufferWidth, framebufferHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

		glBindTexture(GL_TEXTURE_2D, depthRenderTarget[eye]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, framebufferWidth, framebufferHeight, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);

		glBindFramebuffer(GL_FRAMEBUFFER, framebuffer[eye]);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorRenderTarget[eye], 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthRenderTarget[eye], 0);
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	/////////////////////////////////////////////////////////////////
	// Load vertex array buffers
	Mesh triangleMesh = createTriangleMesh();

	/////////////////////////////////////////////////////////////////////
	// Create the main shader
	Shader shader((getCurrentDirectory() + "/shaders/points.vs").c_str(), (getCurrentDirectory() + "/shaders/points.fs").c_str());

    glClearColor(0.f, 0.f, 0.f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Transform
    glm::mat4 view;
    view = glm::lookAt(glm::vec3(0.0f, 0.0f, 3.0f),
                       glm::vec3(0.0f, 0.0f, 0.0f),
                       glm::vec3(0.0f, 1.0f, 0.0f));

    glm::vec3 trianglePositions[] = {
            glm::vec3( 0.0f,  0.0f,  0.0f),
            glm::vec3( 2.0f,  5.0f, -15.0f),
            glm::vec3(-1.5f, -2.2f, -2.5f),
            glm::vec3(-3.8f, -2.0f, -12.3f),
            glm::vec3( 2.4f, -0.4f, -3.5f),
            glm::vec3(-1.7f,  3.0f, -7.5f),
            glm::vec3( 1.3f, -2.0f, -2.5f),
            glm::vec3( 1.5f,  2.0f, -2.5f),
            glm::vec3( 1.5f,  0.2f, -1.5f),
            glm::vec3(-1.3f,  1.0f, -1.5f)
    };


# ifdef _VR
	vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
# endif

    // Game loop
    while (!glfwWindowShouldClose(window))
    {
		assert(glGetError() == GL_NONE);

		const float nearPlaneZ = -0.1f;
		const float farPlaneZ = -100.0f;
		const float verticalFieldOfView = 45.0f * PI / 180.0f;
		Matrix4x4 eyeToHead[numEyes], projectionMatrix[numEyes], headToBodyMatrix;
		glm::mat4 eyeToHeadGLM[numEyes], projectionGLM[numEyes], headToBodyMatrixGLM;
#   ifdef _VR
		getEyeTransformations(hmd, trackedDevicePose, nearPlaneZ, farPlaneZ, headToBodyMatrix.data, eyeToHead[0].data, eyeToHead[1].data, projectionMatrix[0].data, projectionMatrix[1].data);
		//getEyeTransformations(hmd, trackedDevicePose, nearPlaneZ, farPlaneZ, glm::value_ptr(headToBodyMatrixGLM), glm::value_ptr(eyeToHeadGLM[0]), glm::value_ptr(eyeToHeadGLM[1]), glm::value_ptr(projectionGLM[0]), glm::value_ptr(projectionGLM[1]));
#   else
		projectionMatrix[0] = Matrix4x4::perspective(float(framebufferWidth), float(framebufferHeight), nearPlaneZ, farPlaneZ, verticalFieldOfView);
		projectionGLM[0] = glm::perspective(verticalFieldOfView, framebufferWidth / framebufferHeight, nearPlaneZ, farPlaneZ);
#   endif

        GLfloat currentFrame = (GLfloat)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();
        doMovement();

		//view = camera.GetViewMatrix();
		view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f);

		for (int eye = 0; eye < numEyes; ++eye) {
			glBindFramebuffer(GL_FRAMEBUFFER, framebuffer[eye]);
			glViewport(0, 0, framebufferWidth, framebufferHeight);

			// Clear the colorbuffer
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			shader.use();

			GLint modelLoc = glGetUniformLocation(shader.program, "model");
			GLint viewLoc = glGetUniformLocation(shader.program, "view");
			GLint projectionLoc = glGetUniformLocation(shader.program, "projection");
			glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

			int viewport[4];
			glGetIntegerv(GL_VIEWPORT,viewport);
			float heightOfNearPlane = (float)abs(viewport[3]-viewport[1]) /
									  (2*(float)tan(0.5*camera.Zoom*3.1416/180.0));

			GLint pointSizeLoc = glGetUniformLocation(shader.program, "pointSize");
			GLint heightOfNearPlaneLoc = glGetUniformLocation(shader.program, "heightOfNearPlane");
			glUniform1f(pointSizeLoc, 0.1f);
			glUniform1f(heightOfNearPlaneLoc, heightOfNearPlane);

			for(GLuint i = 0; i < 10; i++)
			{
				glm::mat4 model = glm::mat4();
				model = glm::translate(model, trianglePositions[i]);
				GLfloat angle = 20.0f * i;
				model = glm::rotate(model, angle, glm::vec3(1.0f, 0.3f, 0.5f));
				glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
				glPointSize(10.0f);
				triangleMesh.Draw(shader);
			}

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

    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
    return 0;

    return 0;
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    std::cout << key << std::endl;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (action == GLFW_PRESS)
        keys[key] = true;
    else if (action == GLFW_RELEASE)
        keys[key] = false;
}

void doMovement()
{
    if(keys[GLFW_KEY_W])
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(keys[GLFW_KEY_S])
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(keys[GLFW_KEY_A])
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(keys[GLFW_KEY_D])
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(g_firstMouse)
    {
        g_lastX = (GLfloat)xpos;
        g_lastY = (GLfloat)ypos;
        g_firstMouse = false;
    }

    GLfloat xoffset = (GLfloat)xpos - g_lastX;
    GLfloat yoffset = g_lastY - (GLfloat)ypos;
    g_lastX = (GLfloat)xpos;
    g_lastY = (GLfloat)ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll((GLfloat)yoffset);
}