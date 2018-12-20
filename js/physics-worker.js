"use strict;"

let PhysicsWorker = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        let transform = this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // copy the particles out of the model
        let model = parameters.model;
        let particles = this.particles = [];
        for (let particle of model.particles) {
            particle.transform = transform;
            particles.push (Particle.new (particle));
        }
        this.totalMass = model.totalMass;

        // establish the triangulated distance constraints that hold the particles together as a
        // stable structure
        let constraints = this.constraints = [];
        for (let strut of model.struts) {
            constraints.push (DistanceConstraint.new ({particles: particles, a: strut.a, b: strut.b}));
        }
    };

    _.updateCoordinateFrame = function (deltaTime) {
        let particles = this.particles;

        // compute the centroid of the particles as the translation for the updated transformation
        // matrix
        let centerOfMass = [0, 0, 0];
        for (let particle of particles) {
            centerOfMass = Float3.add (centerOfMass, Float3.scale (particle.position, particle.mass));
        }
        centerOfMass = Float3.scale (centerOfMass, 1 / this.totalMass);

        // XXX - this needs a way to specify the basis from the model
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
        this.transform = Float4x4.inverse (Float4x4.viewMatrix (X, Y, Z, centerOfMass));
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
        // place holder for sub-classes to do actual work
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

        // numerical methods drift, a regular re-normalization counteracts the drift. all of the
        // points are reset to their base multiplied by the transform (as computed in
        // updateCoordinateFrame).
        let transform = this.transform;
        for (let particle of particles) {
            particle.position = Float4x4.preMultiply(particle.base, transform);
        }
    };

    return _;
} ();
