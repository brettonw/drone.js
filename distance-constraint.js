"use strict;"

let DistanceConstraint = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.a = parameters.a;
        this.b = parameters.b;
        this.length = Float3.norm (Float3.subtract (parameters.particles[this.a].position, parameters.particles[this.b].position))

        // good defaults, damping: 0.5, springConstant (aka: k): 2.0
        this.damping = Utility.defaultValue (parameters.damping, 0.4);
        this.springConstant = Utility.defaultValue (parameters.springConstant, 1.95);
        console.log ("DistanceConstraint: (" + this.a + " -> " + this.b + "), length: " + this.length.toFixed(3));
    };

    _.apply = function (particles, deltaTime) {
        let a = particles[this.a];
        let b = particles[this.b];
        let delta = Float3.subtract (a.position, b.position);
        let length = Float3.norm (delta);
        delta = Float3.scale (delta, 1 / length);

        // compute the relative velocity damping force to apply, the goal here is to halt all
        // relative motion between the particles with the application of this force
        let relativeVelocity = Float3.subtract (a.velocity, b.velocity);
        let springVelocity = Float3.dot(relativeVelocity, delta);
        let totalMass = a.mass + b.mass;
        let velocityDampingForceA = this.damping * (a.mass / totalMass) * springVelocity * totalMass / deltaTime;
        let velocityDampingForceB = this.damping * (b.mass / totalMass) * springVelocity * totalMass / deltaTime;

        // compute a spring force to make length be equal to constraint.length,
        // using Hooke's law, 2.0 seems to work well
        let springForce = this.springConstant * (length - this.length);

        // apply the forces
        let FA = springForce + velocityDampingForceA;
        let FB = springForce + velocityDampingForceB;
        a.applyForce(Float3.scale(delta, -FA));
        b.applyForce(Float3.scale(delta, FB))
    };

    return _;
} ();

