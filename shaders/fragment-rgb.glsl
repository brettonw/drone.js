precision highp float;

uniform float outputAlpha;

varying vec3 model;

void main(void) {
    gl_FragColor = vec4 (model, outputAlpha);
}
