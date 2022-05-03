#version 330
in vec3 texCoords;

out vec4 out_color;

uniform samplerCube u_texture_cubemap;

void main() {
    out_color = texture(u_texture_cubemap, texCoords);
}