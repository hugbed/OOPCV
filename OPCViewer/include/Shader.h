//
// Created by bedh2102 on 15/08/16.
//

#ifndef OPCVIEWER_SHADER_H
#define OPCVIEWER_SHADER_H

#include <GL/glew.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {

public:
    GLuint program;
    // Constructor generates the shader on the fly
    Shader(const GLchar* vertexPath, const GLchar* fragmentPath, const GLchar* geometryPath = nullptr);
	~Shader();

    // Uses the current shader
    void use() { glUseProgram(this->program); }

private:
    void checkCompileErrors(GLuint shader, std::string type);
};

#endif //OPCVIEWER_SHADER_H
