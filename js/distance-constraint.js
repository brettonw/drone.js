"use strict;"

let DistanceConstraint = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        let a = this.a = parameters.particles[parameters.a];
        let b = this.b = parameters.particles[parameters.b];
        let totalMass = this.totalMass = a.mass + b.mass;
        this.length = Float3.norm (Float3.subtract (a.position, b.position));

        this.springConstant = Utility.defaultValue (parameters.springConstant, 1e3);
        this.damping = Utility.defaultValue (parameters.damping, 0.2);
        //console.log ("DistanceConstraint: (" + parameters.a + " -> " + parameters.b + "), length: " + this.length.toFixed(3));
    };

    _.apply = function (deltaTime) {
        let a = this.a;
        let b = this.b;
        let delta = Float3.subtract (a.position, b.position);
        let deltaLength = Float3.norm (delta);

        // for Verlet (position-based) integration, we simply move the particles to where they
        // should be according to the constraint and the relative mass of the two particles. the
        // state information contained in the particles maintains basic laws, conservation of
        // momentum, etc.
        var changeLength = (this.length - deltaLength) / deltaLength;
        a.position = Float3.add(a.position, Float3.scale(delta, changeLength * (a.mass / this.totalMass)))
        b.position = Float3.add(b.position, Float3.scale(delta, -changeLength * (b.mass / this.totalMass)))

/*
        // compute the relative velocity damping force to apply, the goal here is to halt all
        // relative motion between the particles with the application of this force
        let relativeVelocity = Float3.subtract (a.velocity, b.velocity);
        let springVelocity = Float3.dot(relativeVelocity, delta);
        let totalMass = this.totalMass;
        let velocityDampingForce = this.damping * springVelocity * totalMass / deltaTime;
        let velocityDampingForceA = this.damping * (a.mass / totalMass) * springVelocity * totalMass / deltaTime;
        let velocityDampingForceB = this.damping * (b.mass / totalMass) * springVelocity * totalMass / deltaTime;

        // compute a spring force to make length be equal to constraint.length using Hooke's law
        let springForce = this.springConstant * (length - this.length);

        // apply the forces
        let F = springForce + velocityDampingForce;
        a.applyForce(Float3.scale(delta, -F));
        b.applyForce(Float3.scale(delta, F))
*/
    };

    return _;
} ();

