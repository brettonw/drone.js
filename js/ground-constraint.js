"use strict;"

let GroundConstraint = function () {
    let _ = Object.create (ClassBase);

    _.apply = function (particles, deltaTime) {
        let stun = false;
        for (let particle of particles) {
            if (particle.position[1] <= 0.0) {
                /*
                let verticalVelocity = particle.velocity[1];
                let horizontalVelocity = [particle.velocity[0], 0, particle.velocity[2]];
                particle.applyAcceleration (Float3.scale (horizontalVelocity,  -0.1 / deltaTime));

                if (verticalVelocity <= 0) {
                    // compute the acceleration needed to stop the particle dead
                    let elasticity = 0.8;
                    let groundAccel = [0, -(1.0 + elasticity) * verticalVelocity / deltaTime, 0];
                    particle.applyAcceleration (Float3.copy (groundAccel));
                }
                */
                particle.position[1] = 0.0;
                //stun = true;
            }
        }
        return stun;
    };

    return _;
} ();

