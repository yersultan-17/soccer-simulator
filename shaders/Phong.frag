#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE

  vec3 pos3 = v_position.xyz / v_position[3];
  vec3 l3 = normalize(u_light_pos - pos3);
  vec3 v3 = u_cam_pos - pos3;
  vec3 h3 = normalize(l3 + v3);
  vec4 l4 = vec4(l3, 1);
  vec4 h4 = vec4(h3, 1);
  float r = length(u_light_pos - pos3);
  // (Placeholder code. You will want to replace it.)
  vec4 a = 0.3 * vec4(1, 1, 1, 0) * vec4(1, 1, 1, 1);
  vec4 diffuse = u_color * ((vec4(u_light_intensity, 1) / (r * r)) * max(0, dot(v_normal, l4)));
  vec4 s = vec4(1, 1, 1, 0) * ((vec4(u_light_intensity, 1) / (r * r))) * pow(max(0, dot(v_normal, h4)), 100);
  out_color = a + diffuse + s;
  out_color.a = 1;
}

