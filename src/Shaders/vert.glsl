#version 450 core

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

layout (location=2) uniform mat4 projection;
layout (location=3) uniform mat4 view;
layout (location=4) uniform mat4 model;

out VS_OUT
{   
    vec3 position;
    vec3 normal;
} vs_out;

void main(void)
{
    gl_Position = projection*view*model*vec4(position, 1.0);

    vs_out.position = position; 
    vs_out.normal = normal; 
}
