"use strict;"

let GroundConstraint = function () {
    let _ = Object.create (ClassBase);

    _.apply = function (particles, deltaTime) {
        for (let particle of particles) {
            if (particle.position[1] <= 0.0) {
                let up = [0, 1, 0];
                let xAxis = [1, 0, 0];
                let zAxis = [0, 0, 1];

                let verticalVelocity = Float3.dot (particle.velocity, up);
                let horizontalVelocity = [Float3.dot (particle.velocity, xAxis), 0, Float3.dot (particle.velocity, zAxis)];
                particle.applyAcceleration (Float3.scale (horizontalVelocity,  -1 / deltaTime));

                if (verticalVelocity <= 0) {
                    // compute the acceleration needed to stop the particle dead
                    let elasticity = 0.8;
                    let groundAccel = Float3.scale (up, -(1.0 + elasticity) * verticalVelocity / deltaTime);
                    particle.applyAcceleration (groundAccel);
                }
                particle.position[1] = 0.0;
            }
        }
    };

    return _;
} ();

