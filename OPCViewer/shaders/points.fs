#version 330 core
in vec3 fsColor;

out vec4 color;

void main()
{
   color = vec4(fsColor/255.0f, 1.0f);
}