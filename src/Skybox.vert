#version 330
in vec3 a;

out vec3 texCoords;

uniform mat4 u_view_projection;

void main() {
    texCoords = a;
    gl_Position = u_view_projection * vec4(a, 1.f);
}