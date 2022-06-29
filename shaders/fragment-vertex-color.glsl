precision highp float;

uniform float outputAlpha;

varying vec4 color;

void main(void) {
    gl_FragColor = vec4 (color.r, color.g, color.b, color.a * outputAlpha);
}
