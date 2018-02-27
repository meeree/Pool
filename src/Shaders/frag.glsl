#version 450 core

out vec4 color;

layout (location=2) uniform mat4 projection;
layout (location=3) uniform mat4 view;
layout (location=4) uniform mat4 model;

layout (location=5) uniform vec3 camPos;

in VS_OUT
{   
    vec3 position;
    vec3 normal;
} fs_in;

void main(void)
{
    vec3 lightIntensities = vec3(0.6);
    vec3 ambient = vec3(0.5);

    vec4 col = vec4(1.0);
    vec3 l = normalize(mat3(view*model)*(camPos-fs_in.position));
    float brightness = clamp(dot(mat3(view*model)*fs_in.normal, l), 0.0, 1.0); 
    color = vec4((ambient + brightness * lightIntensities) * col.rgb, col.a); 
}
