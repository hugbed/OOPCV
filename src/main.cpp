#include <iostream>

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

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void doMovement();

void _check_gl_error(bool silent) {
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

        if (!silent) {
            std::cerr << "GL_" << error.c_str() << std::endl;
        }
        err=glGetError();
    }
}

bool initWindow(GLFWwindow *&window)
{
    // Init GLFW
    glfwInit();
    // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    // Create a GLFWwindow object that we can use for GLFW's functions
    window = glfwCreateWindow(WIDTH, HEIGHT, "LearnOpenGL", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
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
    _check_gl_error(true);

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

    GLFWwindow *window = nullptr;
    if (!initWindow(window)) return -1;

    // Set the required callback functions
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    if (!initGLEW()) return -1;

    initViewport(window);

    Shader shader("shaders/points.vs", "shaders/points.fs");

    Mesh triangleMesh = createTriangleMesh();

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

    // Game loop
    while (!glfwWindowShouldClose(window))
    {
        GLfloat currentFrame = (GLfloat)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();
        doMovement();

        // Clear the colorbuffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.use();

        // compute view matrix
        view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f);

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

        // Swap the screen buffers
        glfwSwapBuffers(window);
    }

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