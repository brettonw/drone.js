attribute vec3 inputPosition;
attribute vec3 inputNormal;
attribute vec2 inputTexture;
attribute vec4 inputColor;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

varying vec3 normal;
varying vec2 texture;
varying vec4 color;
varying vec3 model;

void main(void) {
    vec4 vertexPosition = vec4(inputPosition, 1.0);
    vec4 modelPosition = modelMatrix * vertexPosition;
    vec4 viewPosition = viewMatrix * modelPosition;
    gl_Position = projectionMatrix * viewPosition;
    model = modelPosition.xyz;
    normal = inputNormal;
    texture = inputTexture;
    color = inputColor;
}
