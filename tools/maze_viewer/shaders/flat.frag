#version 330 core
// flat.frag — the vertex colour as is

in vec4 color;
out vec4 FragColor;

void main()
{
   FragColor = color;
}
