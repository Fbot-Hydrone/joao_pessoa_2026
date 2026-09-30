#version 330 core
// flat.vert — position + per-vertex colour, no lighting (2D debug drawing)

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;

out vec4 color;

uniform mat4 camMatrix;
uniform float pointSize;

void main()
{
   gl_Position = camMatrix * vec4(aPos, 1.0);
   gl_PointSize = pointSize;
   color = aColor;
}
