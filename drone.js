"use strict;"

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

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        let transform = this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // the model is assumed to start at rest
        // the drone uses a shape something like a squashed octahedron, with 6 points and 13 edges
        // that define a connected set of 4 tetrahedrons as stable shapes. we assume all of the
        // vertices have a mass of about 125g.
        let particles = this.particles = [
            Particle.new ({ position: [ 1.0,  0.5, -1.0], mass: 125 }), // 0
            Particle.new ({ position: [ 0.0,  0.5, -0.6], mass: 125 }), // 1
            Particle.new ({ position: [-1.0,  0.5, -1.0], mass: 125 }), // 2
            Particle.new ({ position: [-0.6,  0.5,  0.0], mass: 125 }), // 3
            Particle.new ({ position: [-1.0,  0.5,  1.0], mass: 125 }), // 4
            Particle.new ({ position: [ 0.0,  0.5,  0.6], mass: 125 }), // 5
            Particle.new ({ position: [ 1.0,  0.5,  1.0], mass: 125 }), // 6
            Particle.new ({ position: [ 0.6,  0.5,  0.0], mass: 125 }), // 7
            Particle.new ({ position: [ 0.0, -0.2,  0.0], mass: 125 })  // 8
        ];

        // compute the mass, and the motor forces, such that all 4 motors at half speed are on a
        // balance with gravity
        let mass = 0;
        for (let particle of particles) {
            mass += particle.mass;
        }
        this.motorForce = (2 * mass * 9.8) / 4;


        // establish the distance constraints that hold the drone together
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

            DistanceConstraint.new ({particles: particles, a: 3, b: 7}),

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
            particle.position = Float4x4.preMultiply (particle.base, transform);
        }

        // start the model off with no velocity...
        this.position = computePosition (this.particles);
        this.velocity = Float3.create ().fill (0);

        this.motors = [0, 0, 0, 0];
        this.run (0.5, 0);// = [0.5, -0.5, 0.6, -0.5];
    };

    _.updateCoordinateFrame = function () {
        let particles = this.particles;

        // compute the centroid
        let position = drone.position = computePosition (particles);
        this.velocity = Float3.subtract (position, this.position);
        //console.log ("Velocity: " + Float3.str (this.velocity));

        // the Y frame will be the average of pts 0, 2, 4, and 6 minus point 8
        let Ymid = Float3.scale (Float3.add (Float3.add (particles[0].position, particles[2].position), Float3.add (particles[4].position, particles[6].position)), 1.0 / 4.0);
        let Y = Float3.normalize (Float3.subtract (Ymid, particles[8].position));
        let X = Float3.normalize (Float3.subtract (particles[0].position, particles[2].position));
        let Z = Float3.normalize (Float3.subtract (particles[6].position, particles[0].position));

        let Z2 = Float3.cross (X, Y);
        let Z3 = Float3.normalize (Float3.add (Z, Z2));

        let X2 = Float3.cross (Y, Z3);

        let transform = this.transform = Float4x4.inverse (Float4x4.viewMatrix (X2, Y, Z3, position));

        // reset all of the points to their base, transformed by the transform
        for (let particle of particles) {
            particle.position = Float4x4.preMultiply (particle.base, transform);
        }
    };

    let stun = false;
    _.update = function (deltaTime) {
        let particles = this.particles;
        let subSteps = 200;
        let subStepDeltaTime = deltaTime / subSteps;
        for (let i = 0; i < subSteps; ++i) {
            // apply gravity to all the particles
            for (let particle of particles) {
                particle.applyGravity (subStepDeltaTime);
            }
            stun = GroundConstraint.apply (particles, subStepDeltaTime) && stun;
            if (! stun) {
                for (let i = 0, end = this.motors.length; i < end; ++i) {
                    this.runMotor (i, this.motors[i]);
                }
            }

            // loop over all the constraints to apply them
            for (let constraint of this.constraints) {
                constraint.apply(subStepDeltaTime);
            }

            // loop over all the particles to update them
            for (let particle of particles) {
                particle.update(subStepDeltaTime);
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
                //Float4x4.translate (particle.position)
            );
        }
    };

    let boundaryParticleIndexGroups = [
        [0, 1, 7],
        [2, 3, 1],
        [4, 5, 3],
        [6, 7, 5]
    ];

    _.runMotor = function (which, speed) {
        // speed is positive for clockwise, negative for counter-clockwise [-1..1]

        // the motor is mounted in a triangular bracket at three points, and to apply the motor
        // forces we compute the torque application vectors and the force application vectors
        let particles = this.particles;
        let boundaryParticleIndexes = boundaryParticleIndexGroups[which];
        let a = particles[boundaryParticleIndexes[0]];
        let b = particles[boundaryParticleIndexes[1]];
        let c = particles[boundaryParticleIndexes[2]];

        // compute the origin, o - not a mass-based centroid, just the average of the three boundary
        // particles
        let o = Float3.create ().fill (0);
        o = Float3.add (o, a.position);
        o = Float3.add (o, b.position);
        o = Float3.add (o, c.position);
        o = Float3.scale (o, 1.0 / 3.0);

        // compute the plane defined by the three boundary particles
        let ab = Float3.subtract (b.position, a.position);
        let ac = Float3.subtract (c.position, a.position);
        let n = Float3.normalize (Float3.cross (ab, ac));

        // compute the delta vectors
        let oa = Float3.subtract (a.position, o);
        let ob = Float3.subtract (b.position, o);
        let oc = Float3.subtract (c.position, o);

        // compute the lengths of the delta vectors
        let oaLength = Float3.norm (oa);
        let obLength = Float3.norm (ob);
        let ocLength = Float3.norm (oc);

        // now compute the application vectors as the perpendicular vector to the plane defined by
        // the delta vector and the plane normal, we need to scale the computed vectors to ensure
        // that a torque is delivered to each one of the particles correctly - because the center is
        // probably not equidistant to all three of the boundary particles. this bakes the
        // distribution of torque and force into the computed vectors, making application of those
        // elements simple
        let pa = Float3.scale (Float3.normalize (Float3.cross (n, oa)), 1.0 / oaLength);
        let pb = Float3.scale (Float3.normalize (Float3.cross (n, ob)), 1.0 / obLength);
        let pc = Float3.scale (Float3.normalize (Float3.cross (n, oc)), 1.0 / ocLength);

        // compute the torque and thrust forces for the motor at speed
        let torque = (1e4 * speed) / 3.0;
        let force = (this.motorForce * Math.abs (speed)) / 3.0;

        // apply the motor forces
        a.applyForce (Float3.scale (pa, torque)).applyForce (Float3.scale (n, force));
        b.applyForce (Float3.scale (pb, torque)).applyForce (Float3.scale (n, force));
        c.applyForce (Float3.scale (pc, torque)).applyForce (Float3.scale (n, force));
    };

    _.run = function (speed, turn) {
        // speed is a value 0..1
        // turn is a -1..1 value that gets turned into a ratio
        let motors02 = speed + (turn * speed);
        let motors13 = -((2 * speed) - motors02);
        console.log ("Run (" + motors02 + ", " + motors13 + ")");
        this.motors[0] = this.motors[2] = motors02;
        this.motors[1] = this.motors[3] = motors13;
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
