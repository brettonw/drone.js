precision highp float;

uniform mat4 normalMatrix;
uniform float outputAlpha;
uniform vec3 cameraPosition;

varying vec3 model;
varying vec3 normal;

void main(void) {
	vec3 viewVector = normalize (cameraPosition - model);
    vec3 normalVector = normalize ((normalMatrix * vec4 (normal, 0.0)).xyz);
	float cosViewNormalAngle = clamp (dot(normalVector, viewVector), 0.0, 1.0);
	float scale = 0.7 + (cosViewNormalAngle * 0.3);

    gl_FragColor = vec4 (model * scale, outputAlpha);
}
