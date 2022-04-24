#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 t3 = v_tangent.xyz;
  if (v_tangent[3] != 0) t3 = t3 / v_tangent[3];
  vec3 n3 = v_normal.xyz;
  vec3 b3 = cross(n3, t3);
  mat3 tbn = mat3(t3, b3, n3);

  float width = u_texture_2_size[0];
  float height = u_texture_2_size[1];
  float u = v_uv[0];
  float v = v_uv[1];
  float k_h = u_height_scaling;
  float k_n = u_normal_scaling;

  float dU = (h(vec2(u + 1 / width, v)) - h(v_uv)) * k_h * k_n;
  float dV = (h(vec2(u, v + 1 / height)) - h(v_uv)) * k_h * k_n;
  vec3 n0 = vec3(-dU, -dV, 1);
  vec3 nd = tbn * n0;

  vec3 pos3 = v_position.xyz / v_position[3];
  vec3 l3 = normalize(u_light_pos - pos3);
  vec3 v3 = u_cam_pos - pos3;
  vec3 h3 = normalize(l3 + v3);

  float r = length(u_light_pos - pos3);
  // (Placeholder code. You will want to replace it.)
  vec4 a = 0.1 * vec4(1, 1, 1, 0) * vec4(1, 1, 1, 1);
  vec4 diffuse = u_color * ((vec4(u_light_intensity, 1) / (r * r)) * max(0, dot(nd, l3)));
  vec4 s = vec4(1, 1, 1, 0) * ((vec4(u_light_intensity, 1) / (r * r))) * pow(max(0, dot(nd, h3)), 100);
  out_color = a + diffuse + s;
  out_color.a = 1;
}

