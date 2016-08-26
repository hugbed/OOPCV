/**
\file minimalOpenGL/minimalOpenGL.h
\author Morgan McGuire, http://graphics.cs.williams.edu
Distributed with the G3D Innovation Engine http://g3d.cs.williams.edu

Minimal headers emulating a basic set of 3D graphics classes.
This does not depend on any vector math library.

This requires the headers and source (or static binary) from GLEW

http://glew.sourceforge.net/

and from GLFW

http://www.glfw.org/

(All of which are distributed with G3D)

All 3D math from http://graphicscodex.com
*/
#pragma once

#ifdef __APPLE__
#   define _OSX
#elif defined(_WIN64)
#   ifndef _WINDOWS
#       define _WINDOWS
#   endif
#elif defined(__linux__)
#   define _LINUX
#endif

#include <GL/glew.h>
#ifdef _WINDOWS
#   include <GL/wglew.h>
#elif defined(_LINUX)
#   include <GL/xglew.h>
#endif
#include <GLFW/glfw3.h> 


#ifdef _WINDOWS
// Link against OpenGL
#   pragma comment(lib, "opengl32")
#   pragma comment(lib, "glew32s")
#   pragma comment(lib, "glfw3")
#endif

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <vector>


namespace GL
{
	void checkError() {
		GLenum err(glGetError());

		while (err != GL_NO_ERROR) {
			std::string error;

			switch (err) {
			case GL_INVALID_OPERATION:      error = "INVALID_OPERATION";      break;
			case GL_INVALID_ENUM:           error = "INVALID_ENUM";           break;
			case GL_INVALID_VALUE:          error = "INVALID_VALUE";          break;
			case GL_OUT_OF_MEMORY:          error = "OUT_OF_MEMORY";          break;
			case GL_INVALID_FRAMEBUFFER_OPERATION:  error = "INVALID_FRAMEBUFFER_OPERATION";  break;
			default:break;
			}

			std::cerr << "GL_" << error.c_str() << std::endl;
			err = glGetError();
		}
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

	void initFrameBuffersAndTextures(GLfloat numEyes, GLfloat width, GLfloat height, GLuint *framebuffers, GLuint *colorRenderTarget, GLuint *depthRenderTarget)
	{
		glGenFramebuffers(numEyes, framebuffers);

		glGenTextures(numEyes, colorRenderTarget);
		glGenTextures(numEyes, depthRenderTarget);
		for (int eye = 0; eye < numEyes; ++eye) {
			glBindTexture(GL_TEXTURE_2D, colorRenderTarget[eye]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

			glBindTexture(GL_TEXTURE_2D, depthRenderTarget[eye]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);

			glBindFramebuffer(GL_FRAMEBUFFER, framebuffers[eye]);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorRenderTarget[eye], 0);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthRenderTarget[eye], 0);
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	Mesh createTriangleMesh()
	{
		Vertex vertices[] = {
			// Positions                    // Colors
			{ glm::vec3(0.5f, -0.5f, 0.0f),  glm::vec3(255.0f, 0.0f, 0.0f) },  // Bottom Right
			{ glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 255.0f, 0.0f) },  // Bottom Left
			{ glm::vec3(0.0f,  0.5f, 0.0f),  glm::vec3(0.0f, 0.0f, 255.0f) }   // Top
		};

		GLuint indices[] = {  // Note that we start from 0!
			0, 1, 2  // First Triangle
		};

		Mesh triangleMesh(std::vector<Vertex>(vertices, vertices + sizeof(vertices) / sizeof(vertices[0])),
			std::vector<GLuint>(indices, indices + sizeof(indices) / sizeof(indices[0])));

		return triangleMesh;
	}

	Mesh createPlane()
	{
		std::vector<Vertex> vertices;
		std::vector<GLuint> indices;
		vertices.reserve(40);
		indices.reserve(40);

		int i = 0;
		for (float x = -20; x < 20; x++)
		{
			for (float z = -20; z < 20; z++)
			{
				Vertex vertex = { glm::vec3(x, 0.0f, z), glm::vec3(10.0f * (x + 20.0f), 10.0f * (z + 20.0f), 0.0f) };
				vertices.push_back(vertex);
				indices.push_back(i++);
			}
		}

		Mesh mesh(vertices, indices);

		return mesh;
	}
}