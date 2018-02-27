#version 450 core

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
layout (location=2) in vec4 color;

layout (location=3) uniform mat4 projection;
layout (location=4) uniform mat4 view;
layout (location=5) uniform mat4 model;
layout (location=6) uniform vec3 camPos;

out VS_OUT
{   
    vec4 color;
} vs_out;

void main(void)
{
    gl_Position = projection*view*model*vec4(position, 1.0);

    vec3 lightIntensities = vec3(0.6);
    vec3 ambient = vec3(0.5);

    vec3 l = normalize(mat3(view*model)*(camPos-position));
    float brightness = clamp(dot(mat3(view*model)*normal, l), 0.0, 1.0); 
    vs_out.color = vec4(vec3(ambient + brightness * lightIntensities) * color.rgb, color.a);
}
