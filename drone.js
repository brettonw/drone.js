"use strict;"

let Particle = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.position = Float3.copy (parameters.position);
        this.velocity = Float3.create ().fill (0);
        this.mass = parameters.mass;
        this.force = Float3.create ().fill (0);
        console.log ("Particle: " + Float3.str (this.position));
    };

    _.applyForce = function (force) {
        this.force = Float3.add (this.force, force);
    };

    _.applyAcceleration = function (acceleration) {
        let force = Float3.scale (acceleration, this.getMass ());
        this.applyForce (force);
    };

    _.applyGravity = function (deltaTime) {
        this.applyAcceleration([0.0, -9.8, 0.0]);
    };

    _.getMass = function () {
        return this.mass * (this.position[1] >= 0) ? 1 : 1;
    };

    _.update = function (deltaTime) {
        // compute acceleration from the forces, convert it to the change in velocity, and then
        // clear out the forces so we don't accidentally keep reapplying them
        let deltaVelocity = Float3.scale(this.force, deltaTime / this.getMass ());
        this.force.fill (0);

        // using the midpoint method, compute the position change, assuming the delta velocity
        // change occurs uniformly over deltaTime
        this.position = Float3.add (this.position, Float3.scale (Float3.add (Float3.scale(deltaVelocity, 0.5), this.velocity), deltaTime));

        // update the velocity from the delta
        this.velocity = Float3.add (this.velocity, deltaVelocity);
    };

    return _;
} ();

let GroundConstraint = function () {
    let _ = Object.create (ClassBase);

    _.apply = function (particles, deltaTime) {
        for (let particle of particles) {
            if (particle.position[1] <= 0.0) {
                let up = [0, 1, 0];

                let groundVelocity = Float3.dot (particle.velocity, up);
                if (groundVelocity <= 0) {
                    // compute the acceleration needed to stop the particle dead
                    let elasticity = 0.8;
                    let groundAccel = Float3.scale (up, -(1.0 + elasticity) * groundVelocity / deltaTime);
                    particle.applyAcceleration (groundAccel);
                }
                /*
                // a little bit of friction...
                let groundAccel = Float3.copy ([(-0.75 / deltaTime) * particle.velocity[0], 0, (-0.75 / deltaTime) * particle.velocity[2]]);

                // and apply some forces to resolve the ground situation
                if (particle.velocity[1] < 0) {
                    let elasticity = 0.8;
                    groundAccel[1] = (particle.velocity[1] * -(1.0 + elasticity)) / deltaTime;
                }
                */
                particle.position[1] = 0.0;
                //particle.velocity.fill (0.0);
                //particle.force.fill (0.0);
            }
        }
    };

    return _;
} ();

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
        let totalMass = a.getMass () + b.getMass ();
        let velocityDampingForceA = this.damping * (a.getMass () / totalMass) * springVelocity * totalMass / deltaTime;
        let velocityDampingForceB = this.damping * (b.getMass () / totalMass) * springVelocity * totalMass / deltaTime;

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

let Drone = function () {
    let _ = Object.create (ClassBase);

    let computePosition = function (particles) {
        let position = Float3.create().fill (0);
        let totalMass = 0;
        for (let particle of particles) {
            position = Float3.add (position, Float3.scale (particle.position, particle.mass));
            totalMass += particle.mass;
        }
        position = Float3.scale (position, 1 / totalMass);
        return position;
    };

    _.reset = function (transformation) {
        // the drone uses a shape something like a squashed octahedron, with 6 points and 13 edges
        // that define a connected set of 4 tetrahedrons as stable shapes. we assume all of the
        // vertices have a mass of about 125g.
        let particles = this.particles = [
            Particle.new ({ position: [ 1.00,  0.00,  0.00], mass: 125 }), // 0
            Particle.new ({ position: [ 0.00,  0.00,  1.00], mass: 125 }), // 1
            Particle.new ({ position: [-1.00,  0.00,  0.00], mass: 125 }), // 2
            Particle.new ({ position: [ 0.00,  0.00, -1.00], mass: 125 }), // 3
            Particle.new ({ position: [ 0.00,  0.15,  0.00], mass: 125 }), // 4
            Particle.new ({ position: [ 0.00, -0.35,  0.00], mass: 125 })  // 5
        ];

        let constraints = this.constraints = [
            DistanceConstraint.new ({particles: particles, a: 4, b: 0}),
            DistanceConstraint.new ({particles: particles, a: 4, b: 1}),
            DistanceConstraint.new ({particles: particles, a: 4, b: 2}),
            DistanceConstraint.new ({particles: particles, a: 4, b: 3}),

            DistanceConstraint.new ({particles: particles, a: 4, b: 5}),

            DistanceConstraint.new ({particles: particles, a: 5, b: 0}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 1}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 2}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 3}),

            DistanceConstraint.new ({particles: particles, a: 0, b: 1}),
            DistanceConstraint.new ({particles: particles, a: 1, b: 2}),
            DistanceConstraint.new ({particles: particles, a: 2, b: 3}),
            DistanceConstraint.new ({particles: particles, a: 3, b: 0})
        ];

        // the points might have been defined in a "comfortable" way, where the centroid is not at
        // the origin, so we'll compute the centroid and relocate the points - so the position is at
        // the origin
        let position = computePosition (this.particles);
        for (let particle of particles) {
            particle.base = Float4.point (Float3.subtract (particle.position, position));
            particle.position = Float4x4.preMultiply (particle.base, transformation);
        }

        // start the model off with no velocity...
        this.position = Float3.copy (this.velocity = Float3.create ().fill (0));
    };

    _.reset2 = function (transformation) {
        // the drone uses a shape something like a squashed octahedron, with 6 points and 13 edges
        // that define a connected set of 4 tetrahedrons as stable shapes. we assume all of the
        // vertices have a mass of about 125g.
        let particles = this.particles = [
            Particle.new ({ position: [ 1.0, 0.5,  0.0], mass: 125 }), // 0
            Particle.new ({ position: [ 0.5, 0.5, -0.5], mass: 125 }), // 1
            Particle.new ({ position: [ 0.0, 0.5, -1.0], mass: 125 }), // 2
            Particle.new ({ position: [-0.5, 0.5, -0.5], mass: 125 }), // 3
            Particle.new ({ position: [-1.0, 0.5,  0.0], mass: 125 }), // 4
            Particle.new ({ position: [-0.5, 0.5,  0.5], mass: 125 }), // 5
            Particle.new ({ position: [ 0.0, 0.5,  1.0], mass: 125 }), // 6
            Particle.new ({ position: [ 0.5, 0.5,  0.5], mass: 125 }), // 7
            Particle.new ({ position: [ 0.0, 0.0,  0.0], mass: 125 })  // 8
        ];

        let constraints = this.constraints = [
            DistanceConstraint.new ({particles: particles, a: 0, b: 1}),
            DistanceConstraint.new ({particles: particles, a: 1, b: 2}),
            DistanceConstraint.new ({particles: particles, a: 2, b: 3}),
            DistanceConstraint.new ({particles: particles, a: 3, b: 4}),
            DistanceConstraint.new ({particles: particles, a: 4, b: 5}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 6}),
            DistanceConstraint.new ({particles: particles, a: 6, b: 7}),
            DistanceConstraint.new ({particles: particles, a: 7, b: 0}),

            DistanceConstraint.new ({particles: particles, a: 1, b: 3}),
            DistanceConstraint.new ({particles: particles, a: 3, b: 5}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 7}),
            DistanceConstraint.new ({particles: particles, a: 7, b: 1}),

            DistanceConstraint.new ({particles: particles, a: 5, b: 1}),

            DistanceConstraint.new ({particles: particles, a: 0, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 1, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 2, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 3, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 4, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 5, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 6, b: 8}),
            DistanceConstraint.new ({particles: particles, a: 7, b: 8})
        ];

        // the points might have been defined in a "comfortable" way, where the centroid is not at
        // the origin, so we'll compute the centroid and relocate the points - so the position is at
        // the origin
        let position = computePosition (this.particles);
        for (let particle of particles) {
            particle.base = Float4.point (Float3.subtract (particle.position, position));
            particle.position = Float4x4.preMultiply (particle.base, transformation);
        }

        // start the model off with no velocity...
        this.position = Float3.copy (this.velocity = Float3.create ().fill (0));
    };

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        let transform = this.transform = Utility.defaultValue (parameters.transformation, Float4x4.identity ());

        // the model is assumed to start at rest
        this.reset2 (transform);


        /*
        this.particles[0].applyForce ([0, 15000, 0]);
        this.particles[2].applyForce ([0, -15000, 0]);
        this.particles[3].applyForce ([50000, 0, 0]);
        */
        //this.particles[3].applyForce ([500, 50, 0]);
        //this.particles[5].applyForce ([0, 10000, 0]);
        /*
        this.particles[3].applyForce ([-1e3,  1e3, 0]);
        this.particles[4].applyForce ([-1e3, -1e3, 0]);
        this.particles[5].applyForce ([ 1e3, -1e3, 0]);
        this.particles[6].applyForce ([ 1e3,  1e3, 0]);

        this.particles[1].applyForce (Float3.scale ([-1e3,  1e3, 0], -1));
        this.particles[2].applyForce (Float3.scale ([-1e3, -1e3, 0], -1));
        this.particles[7].applyForce (Float3.scale ([ 1e3, -1e3, 0], -1));
        this.particles[0].applyForce (Float3.scale ([ 1e3,  1e3, 0], -1));
        */
        this.particles[3].applyForce ([ 1e4, 0, 0]);
        this.particles[7].applyForce ([ -1e4, 0, 0]);
    };

    _.updateCoordinateFrame = function () {
        let particles = this.particles;

        // compute the centroid
        let position = drone.position = computePosition (particles);
        this.velocity = Float3.subtract (position, this.position);
        //console.log ("Velocity: " + Float3.str (this.velocity));

        // the Y frame will be the average of pts 1, 3, 5, and 7 minus point 8
        let Ymid = Float3.scale (Float3.add (Float3.add (particles[1].position, particles[3].position), Float3.add (particles[5].position, particles[7].position)), 0.25);
        let Y = Float3.normalize (Float3.subtract (Ymid, particles[8].position));
        let X = Float3.normalize (Float3.subtract (particles[0].position, particles[4].position));
        let Z = Float3.normalize (Float3.subtract (particles[6].position, particles[2].position));

        let Z2 = Float3.cross (X, Y);
        let Z3 = Float3.normalize (Float3.add (Z, Z2));

        let X2 = Float3.cross (Y, Z3);

        let transform = this.transform = Float4x4.inverse (Float4x4.viewMatrix (X2, Y, Z3, position));

        // reset all of the points to their base, transformed by the transform
        for (let particle of particles) {
            particle.position = Float4x4.preMultiply (Float4.point (particle.base), transform);
        }
    };

    _.update = function (deltaTime) {
        let particles = this.particles;
        let subSteps = 200;
        let subStepDeltaTime = deltaTime / subSteps;
        for (let i = 0; i < subSteps; ++i) {
            /*
            // apply gravity to all the particles
            for (let particle of particles) {
                particle.applyGravity (deltaTime);
            }
            GroundConstraint.apply (particles, deltaTime);
            */

            // loop over all the particles to update them
            for (let particle of particles) {
                particle.update(subStepDeltaTime);
            }

            // loop over all the constraints to apply them
            for (let constraint of this.constraints) {
                constraint.apply(this.particles, subStepDeltaTime);
            }
        }

        // do a little update to keep everything normalized (numerical methods drift, this provides
        // a regular reset to counteract the drift).
        this.updateCoordinateFrame ();

        // update the scene graph nodes
        for (let i = 0; i < particles.length;  ++i) {
            let particle = particles[i];
            let node = Node.get ("particle-" + i);
            node.transform = Float4x4.chain (
                Float4x4.scale (0.05),
                Float4x4.translate (particle.base),
                this.transform
                //,Float4x4.translate (particle.position)
            );
        }
    };

    _.runMotor = function (which, speed) {
        let particles = this.particles;
        let particle = particles[which];

        let forceDirection = Float3.normalize (Float3.subtract (particles[4].position, particles[5].position));
        // speed is a [0..1] value
        particle.applyForce (Float3.scale (forceDirection, this.mass * 2.0 * speed));

        // XXX also add torque to the connected parts
    };

    _.getTransformationMatrix = function () {
        // compute a basis using points in the drone

        // have position, need to compute the look along...
        // lookFrom = function (from, along, up)
        let viewVector = Float3.new ();
        Float4x4.lookFrom (this.position, along, up);
    };

    _.addToScene = function (parentNode) {
        // put down the platonic we will use for each corner
        for (let i = 0; i < this.particles.length;  ++i) {
            let red = i / this.particles.length;
            let blue = 1.0 - red;
            parentNode.addChild(Node.new({
                transform: Float4x4.identity(),
                state: function (standardUniforms) {
                    Program.get("basic").use();

                    standardUniforms.MODEL_COLOR =  [red, 0.25, blue];
                },
                shape: "cube",
                children: false
            }, "particle-" + i));
        }

    };

    return _;
} ();
