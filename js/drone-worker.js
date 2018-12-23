"use strict;"

let DroneWorker = function () {
    let _ = Object.create (PhysicsWorker);

    _.construct = function (parameters) {
        // call teh super class constructor
        Object.getPrototypeOf(_).construct.call(this, parameters);

        // compute the motor forces, such that all 4 motors at half speed balance gravity exactly
        let model = parameters.model;
        this.motorForce = (2 * model.totalMass * Math.GRAVITY) / 4;
        this.motors = [0, 0, 0, 0];

        this.boundaryParticleIndexGroups = model.motors;

        // set up the controller PIDs - we scale the x-z location PIDs to limit the allowed output
        // prevent the drone from
        // flipping itself over
        this.controller = {
            locationX:      PID.new ({ gains: { p: 0.5, i: 0.0, d: 1.35 }, outputScale: 0.333 }),
            locationY:      PID.new ({ gains: { p: 0.5, i: 0.0, d: 1.0 }}),
            locationZ:      PID.new ({ gains: { p: 0.5, i: 0.0, d: 1.35 }, outputScale: 0.333 }),
            orientation:    PID.new ({ gains: { p: 0.65, i: 0.0, d: 1.0 }, deltaFunction: function (x, y) { return Math.conditionAngle (y - x) / Math.PI; }}),
            tiltX:          PID.new ({ gains: { p: 0.5, i: 0.0, d: 0.55 }}),
            tiltZ:          PID.new ({ gains: { p: 0.5, i: 0.0, d: 0.55 }})
        };

        // default start goal, we create this here *NOT* to set the goal, but to create the object
        // that will hold the goal values as we continue
        this.goal = { x: 0, y: 0, z: 0 };
    };

    _.subUpdate = function (subStepDeltaTime) {
        if (this.stun === false) {
            this.runController (subStepDeltaTime);
            for (let i = 0, end = this.motors.length; i < end; ++i) {
                this.runMotor (i, this.motors[i]);
            }
        }
    };

    _.updateCoordinateFrame = function (deltaTime) {
        let particles = this.particles;

        //  extract the basis using the differences in the particle positions. the Y-axis will be
        //  the average of pts 0, 2, 4, and 6; minus point 8. X-axis and Z-axis get a couple of
        //  iterations to average out the relative error
        let Ymid = Float3.scale (Float3.add (Float3.add (particles[0].position, particles[2].position), Float3.add (particles[4].position, particles[6].position)), 1.0 / 4.0);
        let X = Float3.normalize (Float3.subtract (particles[0].position, particles[2].position));
        let Y = Float3.normalize (Float3.subtract (Ymid, particles[8].position));
        let Z = Float3.normalize (Float3.subtract (particles[6].position, particles[0].position));
        let Z2 = Float3.cross (X, Y);
        Z = Float3.normalize (Float3.add (Z, Z2));
        X = Float3.cross (Y, Z);

        // build the transform...
        this.transform = Float4x4.inverse (Float4x4.viewMatrix (X, Y, Z, this.position));
    };

    _.runMotor = function (which, speed) {
        // speed is positive for clockwise, negative for counter-clockwise [-1..1]

        // the motor is mounted in a triangular bracket at three points, and to apply the motor
        // forces we compute the torque application vectors and the force application vectors at
        // those three points
        let particles = this.particles;
        let boundaryParticleIndexes = this.boundaryParticleIndexGroups[which];
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

        // compute the deltaScales, default to 1, but if there's a long way to go, try to balance
        // the PID responses to go in a straight line
        let deltaScales = [1, 1, 1];
        /*
        let deltaToGoal = Float3.subtract ([goal.x, goal.y, goal.z], this.position).map (component => Math.abs (component));
        const near = 1.0;
        let deltaToGoalLength = Float3.norm (deltaToGoal);
        if (deltaToGoalLength > near) {
            // figure how much of the distance to go is outside of the "near" region, to compute an
            // interpolant
            let nearWeight = near / deltaToGoalLength;
            let farWeight = 1.0 - nearWeight;

            deltaScales = deltaToGoal.map (component => (nearWeight * 1.0) + (farWeight * (component / deltaToGoalLength)));
        }
        */

        // compute the altitude of the drone using the y component of the translation
        let speed = (controller.locationY.update (transform[13], goal.y, deltaTime, deltaScales[1]) + 1.0) / 2.0;

        // compute the orientation of the drone using the x and z components of the x-axis - our
        // goal is to always orient the drone with the x and z axes (the other assumptions are
        // violated if the drone turns, and it will become unstable if it turns too much. we rely on
        // a rapid update cycle to keep the orientation close to the stable configuration).
        let orientationAngle = Math.atan2(transform[2], transform[0]);
        let turn = -controller.orientation.update (orientationAngle, 0.0, deltaTime);

        // compute the required velocity input to reach the target location in each axis
        let xVel = controller.locationX.update (this.position[0], goal.x, deltaTime, deltaScales[0]);
        let zVel = controller.locationZ.update (this.position[2], goal.z, deltaTime, deltaScales[2]);

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

    return _;
} ();
