"use strict;"

let Drone = function () {
    let _ = Object.create (Thing);

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

        // define a group of particles that we will use to build our triangulated, stable structure
        let particles = this.particles = [
            Particle.new ({ position: [ 1.0,  0.5, -1.0], mass: 100 }), // 0
            Particle.new ({ position: [ 0.0,  0.5, -0.6], mass: 125 }), // 1
            Particle.new ({ position: [-1.0,  0.5, -1.0], mass: 100 }), // 2
            Particle.new ({ position: [-0.6,  0.5,  0.0], mass: 125 }), // 3
            Particle.new ({ position: [-1.0,  0.5,  1.0], mass: 100 }), // 4
            Particle.new ({ position: [ 0.0,  0.5,  0.6], mass: 125 }), // 5
            Particle.new ({ position: [ 1.0,  0.5,  1.0], mass: 100 }), // 6
            Particle.new ({ position: [ 0.6,  0.5,  0.0], mass: 125 }), // 7
            Particle.new ({ position: [ 0.0, -0.2,  0.0], mass: 150 })  // 8
        ];

        // compute the mass, and the motor forces, such that all 4 motors at half speed are on a
        // balance with gravity
        let mass = 0;
        for (let particle of particles) {
            mass += particle.mass;
        }
        this.motorForce = (2 * mass * Math.GRAVITY) / 4;

        // establish the triangulated distance constraints that hold the particles together as a
        // stable structure
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
        // the origin; we'll compute the centroid and relocate the points - so the position is at
        // the origin
        let position = computePosition (this.particles);
        for (let particle of particles) {
            particle.base = Float4.point (Float3.subtract (particle.position, position));
            particle.position = Float4x4.preMultiply (particle.base, transform);
        }

        // position *SHOULD* be at the origin, and start the model off with no velocity...
        this.position = computePosition (this.particles);
        this.velocity = Float3.create ().fill (0);

        this.motors = [0, 0, 0, 0];

        // set up the controller PIDs - we scale the x-z location PIDs to limit the allowed output
        // prevent the drone from
        // flipping itself over
        this.controller = {
            locationX: PID.new (  { gains: { p: 0.5, i: 0.0, d: 1.0 }, outputScale: 0.333 }),
            locationY: PID.new (  { gains: { p: 0.5, i: 0.0, d: 0.65 }}),
            locationZ: PID.new (  { gains: { p: 0.5, i: 0.0, d: 1.0 }, outputScale: 0.333 }),
            orientation: PID.new ({ gains: { p: 0.5, i: 0.0, d: 0.65 }, deltaFunction: function (x, y) {
                    return Math.conditionAngle (y - x) / Math.PI;
                }}),
            tiltX: PID.new (      { gains: { p: 0.5, i: 0.0, d: 0.55 }}),
            tiltZ: PID.new (      { gains: { p: 0.5, i: 0.0, d: 0.55 }})
        };

        // drones are only alive if they haven't crashed, we call that "stunned", and start out
        // *NOT* stunned
        this.stun = false;

        // default start goal, we create this here *NOT* to set the goal, but to create the object
        // that will hold the goal values as we continue
        this.goal = { x: 0, y: 1, z: 0 };

        // some of the particle nodes will flash brightly to resemble running lights. these control
        // the flash timing
        let flashers = this.flashers = [];
        for (let i = 0; i < 5; ++i) {
            flashers.push (Flasher.new ());
        }

        // copy if the drone is created in a debug state
        this.debug = Utility.defaultValue (parameters.debug, false);
    };

    _.updateCoordinateFrame = function (deltaTime) {
        let particles = this.particles;

        // compute the centroid
        let position = computePosition (particles);
        this.velocity = Float3.scale (Float3.subtract (position, this.position), 1.0 / deltaTime);
        //console.log ("Velocity: " + Float3.str (this.velocity));
        this.position = position;

        // the Y frame will be the average of pts 0, 2, 4, and 6; minus point 8
        let Ymid = Float3.scale (Float3.add (Float3.add (particles[0].position, particles[2].position), Float3.add (particles[4].position, particles[6].position)), 1.0 / 4.0);
        let Y = Float3.normalize (Float3.subtract (Ymid, particles[8].position));
        let X = Float3.normalize (Float3.subtract (particles[0].position, particles[2].position));
        let Z = Float3.normalize (Float3.subtract (particles[6].position, particles[0].position));
        let Z2 = Float3.cross (X, Y);
        Z = Float3.normalize (Float3.add (Z, Z2));
        X = Float3.cross (Y, Z);

        this.transform = Float4x4.inverse (Float4x4.viewMatrix (X, Y, Z, position));
    };

    _.subUpdateParticles = function (subStepDeltaTime) {
        let particles = this.particles;

        // apply the ground constraint, then loop over all the distance constraints to apply them
        this.stun = GroundConstraint.apply (particles, subStepDeltaTime) || this.stun;
        for (let constraint of this.constraints) {
            constraint.apply (subStepDeltaTime);
        }

        // apply gravity and air resistance to all the particles, and update them
        for (let particle of particles) {
            particle
                .applyGravity (subStepDeltaTime)
                .applyDrag ()
                .update (subStepDeltaTime);
        }

        this.updateCoordinateFrame (subStepDeltaTime);
    };

    _.subUpdate = function (subStepDeltaTime) {
        if (this.stun === false) {
            this.runController (subStepDeltaTime);
            for (let i = 0, end = this.motors.length; i < end; ++i) {
                this.runMotor (i, this.motors[i]);
            }
        }
    };

    _.update = function (deltaTime) {
        let particles = this.particles;

        // we want to update the physics simulation faster than we display it, the goal is a total
        // of *about* 1,000Hz.
        let subSteps = Math.floor(1000 / targetFrameRate);
        let subStepDeltaTime = deltaTime / subSteps;
        for (let i = 0; i < subSteps; ++i) {
            this.subUpdateParticles(subStepDeltaTime);
            this.subUpdate(subStepDeltaTime);
        }

        // update the scene graph nodes - this is primarily a debugging tool, as the particle nodes
        // exist as static transforms relative to the base... do this BEFORE we re-normalize
        if (this.debug === true) {
            for (let i = 0; i < particles.length; ++i) {
                let particle = particles[i];
                Node.get(this.name + " (particle-" + i + ")").transform = Float4x4.chain(Float4x4.scale(0.05), Float4x4.translate(particles[i].position));
            }
        }

        // numerical methods drift, a regular re-normalization counteracts the drift. all of the
        // points are reset to their base multiplied by the transform (as computed in
        // updateCoordinateFrame).
        let transform = this.transform;
        for (let particle of particles) {
            particle.position = Float4x4.preMultiply(particle.base, transform);
        }

        // update the flashers
        for (let flasher of this.flashers) {
            flasher.update(deltaTime);
        }

        // and finally - set the transform on the node so we can draw the drone
        Node.get (this.name + " (drone-model)").transform = transform;
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
        // forces we compute the torque application vectors and the force application vectors at
        // those three points
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

        // compute the torque and thrust forces for the motor at the requested speed, remember that
        // this model assumes the motors produce positive thrust, regardless of the sign of the
        // speed. the reason is that drones run motors in opposite directions to use the torque to
        // control orientation - we don't run the prop backwards to create reverse thrust.
        let torque = (1e4 * speed) / 3.0;
        let force = (this.motorForce * Math.abs (speed)) / 3.0;

        // apply the motor forces
        a.applyForce (Float3.scale (pa, torque)).applyForce (Float3.scale (n, force));
        b.applyForce (Float3.scale (pb, torque)).applyForce (Float3.scale (n, force));
        c.applyForce (Float3.scale (pc, torque)).applyForce (Float3.scale (n, force));
    };

    _.run = function (speed, turn, tilt) {
        // motors are configured like this:
        // 1 0
        // 2 3
        // 0 and 2 are run clockwise, 1 and 3 are run counter-clockwise

        // turn is a -1..1 value that gets turned into a ratio of 0-2 to 1-3 (input of 1 maps to
        // all 1-3, and input of -1 maps to all 0-2)
        let turnRatio02 = turn + 1.0;
        let turnRatio13 = 2.0 - turnRatio02;

        // now tilt is an x-y vector, where each axis is a ratio of pairs of motors to the opposing
        // pair. the z-axis ratio is motors 0-1 to 2-3, and the x-axis ratio is motors 1-2 to 0-3
        let xTiltRatio12 = tilt.x + 1.0;
        let xTiltRatio03 = 2.0 - xTiltRatio12;
        let zTiltRatio01 = tilt.z + 1.0;
        let zTiltRatio23 = 2.0 - zTiltRatio01;

        // the final run speeds are a linear combination of all the ratios with speed, which is a
        // value 0..1, assuming motors are always running in a direction that produces lift
        this.motors[0] = Math.clamp (speed * turnRatio02 * xTiltRatio03 * zTiltRatio01, 0, 1);
        this.motors[1] = -Math.clamp (speed * turnRatio13 * xTiltRatio12 * zTiltRatio01, 0, 1);
        this.motors[2] = Math.clamp (speed * turnRatio02 * xTiltRatio12 * zTiltRatio23, 0, 1);
        this.motors[3] = -Math.clamp (speed * turnRatio13 * xTiltRatio03 * zTiltRatio23, 0, 1);
    };

    _.runController = function (deltaTime) {
        // because I hate having to type "this." all the time...
        let controller = this.controller;
        let transform = this.transform;
        let goal = this.goal;

        // compute the altitude of the drone using the y component of the translation
        let speed = (controller.locationY.update (transform[13], goal.y, deltaTime) + 1.0) / 2.0;

        // compute the orientation of the drone using the x and z components of the x-axis - our
        // goal is to always orient the drone with the x and z axes (the other assumptions are
        // violated if the drone turns, and it will become unstable if it turns too much. we rely on
        // a rapid update cycle to keep the orientation close to the stable configuration).
        let orientationAngle = Math.atan2(transform[2], transform[0]);
        let turn = -controller.orientation.update (orientationAngle, 0.0, deltaTime);

        // compute the required velocity input to reach the target location in each axis
        let xVel = controller.locationX.update (transform[12], goal.x, deltaTime);
        let zVel = controller.locationZ.update (transform[14], goal.z, deltaTime);

        // use the velocity input to compute the tilt input, we measure these values by looking at
        // x and z components of the y axis.
        let tilt = {
            x: controller.tiltX.update (transform[4], xVel, deltaTime),
            z: controller.tiltZ.update (transform[6], zVel, deltaTime)
        };

        // give the drone the inputs
        this.run (speed, turn, tilt);
    };

    _.setGoal = function (xyz) {
        this.goal.x = xyz[0];
        this.goal.y = xyz[1];
        this.goal.z = xyz[2];
    };

    _.addToScene = function (parentNode) {
        console.log ("Drone named: " + this.name);
        let that = this;

        let particles = this.particles;

        // put down the dots we will use for each of the drone particles - this is a debugging tool
        if (this.debug === true) {
            for (let i = 0; i < particles.length; ++i) {
                parentNode.addChild(Node.new({
                    transform: Float4x4.identity(),
                    state: function (standardUniforms) {
                        standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                        Program.get("basic").use();
                    },
                    shape: "sphere2",
                    children: false
                }, this.name + " (particle-" + i + ")"));
            }
        }

        // put down a representation of the drone geometry for visual purposes
        let modelNode = Node.new ({
                transform: Float4x4.identity (),
                state: function (standardUniforms) {
                    standardUniforms.OUTPUT_ALPHA_PARAMETER = 1.0;
                },
            }, this.name + " (drone-model)")
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([-0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([-0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([-0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([-0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.rotateX (Math.PI / -2), Float4x4.scale ([1.1, 1, 1.1])),
                state: function (standardUniforms) {
                    Program.get ("basic-texture").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                    standardUniforms.TEXTURE_SAMPLER = "drone-deck";
                },
                shape: "square",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.rotateX (Math.PI / -2), Float4x4.translate ([0, 0.05, 0]), Float4x4.scale ([1.1, 1, 1.1])),
                state: function (standardUniforms) {
                    Program.get ("basic-texture").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                    standardUniforms.TEXTURE_SAMPLER = "drone-props";
                },
                shape: "square",
                children: false
            }));

        // add the actual particles below the model node, so they transform with the drone
        for (let i = 0; i < particles.length;  ++i) {
            let particle = particles[i];
            modelNode.addChild(Node.new({
                transform: Float4x4.chain ( Float4x4.scale (0.0002 * particle.mass), Float4x4.translate (particle.base)),
                state: function (standardUniforms) {
                    switch (i) {
                        case 0: standardUniforms.MODEL_COLOR = [0.0, 0.25 + (that.flashers[0].getIsOn() * 0.75), 0.0]; Program.get("color").use(); break;
                        case 6: standardUniforms.MODEL_COLOR = [0.0, 0.25 + (that.flashers[1].getIsOn () * 0.75), 0.0]; Program.get("color").use(); break;
                        case 2: standardUniforms.MODEL_COLOR = [0.25 + (that.flashers[2].getIsOn () * 0.75), 0.0, 0.0]; Program.get("color").use();break;
                        case 4: standardUniforms.MODEL_COLOR = [0.25 + (that.flashers[3].getIsOn () * 0.75), 0.0, 0.0]; Program.get("color").use();break;
                        case 5: {
                            let isOn = that.flashers[4].getIsOn ();
                            standardUniforms.MODEL_COLOR = [0.25 + (isOn * 0.75), 0.25 + (isOn * 0.75), 0.2 + (isOn * 0.6)];
                            Program.get ("color").use ();
                            break;
                        }
                        default: standardUniforms.MODEL_COLOR = [0.5, 0.25, 0.125]; Program.get("basic").use();break;
                    }
                },
                shape: "sphere2",
                children: false
            }));
        }

        parentNode.addChild (modelNode);
        return this;
    };

    return _;
} ();
