"use strict;"

let Particle = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.base = Float4.point (parameters.position);
        this.position = Float4x4.preMultiply (this.base, parameters.transform);
        this.velocity = Float3.create ().fill (0);
        this.mass = Utility.defaultValue (parameters.mass, 100.0);
        this.force = Float3.create ().fill (0);
        console.log ("Particle: " + Float3.str (this.position));
    };

    _.applyForce = function (force) {
        this.force = Float3.add (this.force, force);
        return this;
    };

    _.applyAcceleration = function (acceleration) {
        let force = Float3.scale (acceleration, this.mass);
        this.applyForce (force);
        return this;
    };

    _.applyGravity = function () {
        this.applyAcceleration(Float3.copy ([0.0, -Math.GRAVITY, 0.0]));
        return this;
    };

    _.applyDrag = function () {
        let windVelocity = [0, 0, 0]; //worldState.query (WorldState.WIND);

        // coefficient of drag for a sphere is 0.5, particles have an assumed radius of 0.1, so the
        // "frontal area" is Pi*r*r or about .01 * 3.14 = .031415 - we call that 0.1 for effect...
        const a = 0.1;
        const rho = 1.2;
        const cd = 0.5;

        // compute the total particle velocity relative to the wind, then compute the particle
        // response to the drag force.
        let totalVelocity = Float3.add (Float3.scale (windVelocity, -1.0), this.velocity);
        let v = Float3.norm (totalVelocity);
        if (v > 0) {
            let fd = 0.5 * cd * rho * v * v * a;
            let dragForceVector = Float3.scale (totalVelocity, -fd / v);
            this.applyForce (dragForceVector);
        }
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

