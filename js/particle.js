"use strict;"

let Particle = function () {
    let _ = Object.create (ClassBase);

    let validateFloat3 = function (float3) {
        for (let component of float3) {
            if (isNaN (component)) {
                console.log ("isNan");
            }
        }
    };

    _.construct = function (parameters) {
        this.position = Float3.copy (parameters.position);
        this.velocity = Float3.create ().fill (0);
        this.mass = parameters.mass;
        this.force = Float3.create ().fill (0);
        console.log ("Particle: " + Float3.str (this.position));
    };

    _.applyForce = function (force) {
        validateFloat3 (force);
        this.force = Float3.add (this.force, force);
        validateFloat3 (this.force);
        return this;
    };

    _.applyAcceleration = function (acceleration) {
        // check that acceleration is valid
        validateFloat3 (acceleration);
        let force = Float3.scale (acceleration, this.mass);
        this.applyForce (force);
        return this;
    };

    _.applyGravity = function (deltaTime) {
        this.applyAcceleration(Float3.copy ([0.0, -Math.GRAVITY, 0.0]));
        return this;
    };

    _.update = function (deltaTime) {
        // compute acceleration from the forces, convert it to the change in velocity, and then
        // clear out the forces so we don't accidentally keep reapplying them
        let deltaVelocity = Float3.scale(this.force, deltaTime / this.mass);
        this.force.fill (0);

        // using the midpoint method, compute the position change, assuming the delta velocity
        // change occurs uniformly over deltaTime
        this.position = Float3.add (this.position, Float3.scale (Float3.add (Float3.scale(deltaVelocity, 0.5), this.velocity), deltaTime));

        // update the velocity from the delta
        this.velocity = Float3.add (this.velocity, deltaVelocity);
    };

    return _;
} ();

