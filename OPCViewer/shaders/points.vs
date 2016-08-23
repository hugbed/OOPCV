#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

out vec3 fsColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform float heightOfNearPlane;
uniform float pointSize;

void main()
{
   gl_Position = projection * view * model * vec4(position, 1.0);
   gl_PointSize = (heightOfNearPlane * pointSize) / gl_Position.w;
   fsColor = color;
}