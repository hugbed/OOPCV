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

const GLuint WIDTH = 800, HEIGHT = 600;

using std::cout;
using std::endl;

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);

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
}

bool createVAO(GLuint &VAO)
{
    GLfloat vertices[] = {
            // Positions         // Colors
            0.5f, -0.5f, 0.0f,   1.0f, 0.0f, 0.0f,  // Bottom Right
           -0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,  // Bottom Left
            0.0f,  0.5f, 0.0f,   0.0f, 0.0f, 1.0f   // Top
    };

    GLuint indices[] = {  // Note that we start from 0!
        0, 1, 2,  // First Triangle
    };
    GLuint VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

    glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO

    return true;
}

int main() {
    std::cout << "Starting GLFW context, OpenGL 3.3" << std::endl;

    GLFWwindow *window = nullptr;
    if (!initWindow(window)) return -1;

    // Set the required callback functions
    glfwSetKeyCallback(window, key_callback);

    if (!initGLEW()) return -1;

    initViewport(window);

    Shader shader("shaders/basic.vs", "shaders/basic.fs");

    GLuint VAO;
    createVAO(VAO);

    glClearColor(0.f, 0.f, 0.f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(10.f);

    // Transform
    glm::mat4 initTransform;

    // Game loop
    while (!glfwWindowShouldClose(window))
    {
        // Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        // Clear the colorbuffer
        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();

        glm::mat4 transform = glm::rotate(initTransform, glm::radians((GLfloat)glfwGetTime() * 10.0f), glm::vec3(0.0, 0.0, 1.0));
        GLint transformLocation = glGetUniformLocation(shader.program, "transform");
        glUniformMatrix4fv(transformLocation, 1, GL_FALSE, glm::value_ptr(transform));

        // Render
        // do great stuff here
        glBindVertexArray(VAO);
        glDrawElements(GL_POINTS, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

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
}
