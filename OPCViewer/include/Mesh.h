//
// Created by bedh2102 on 16/08/16.
//

#ifndef OPCVIEWER_MESH_H
#define OPCVIEWER_MESH_H

#define GLEW_STATIC
#include "GL/glew.h"

#include <glm/glm.hpp>

#include "Shader.h"

#include <vector>


struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
};

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;

	Mesh(std::vector<Vertex> vertices, std::vector<GLuint> indices);
	~Mesh();
	void Mesh::updateData(std::vector<Vertex> vertices, std::vector<GLuint> indices);
    void Draw(Shader shader);

private:
    GLuint VAO, VBO, EBO;

    void setupMesh();
};



#endif //OPCVIEWER_MESH_H
