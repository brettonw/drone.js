"use strict;"

if (! ("TWO_PI" in Math)) {
    Math.TWO_PI = Math.PI * 2.0;
}

if (! ("GRAVITY" in Math)) {
    Math.GRAVITY = 9.8;
}

if (! ("clamp" in Math)) {
    Math.clamp = function (value, min, max) {
        return (value < min) ? min : ((value > max) ? max : value);
    };
}