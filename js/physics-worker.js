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
            constraints.push (DistanceConstraint.new ({particles: particles, a: strut.a, b: strut.b, springConstant: strut.k}));
        }

        // copy the basis
        let basis = model.basis;
        this.basis = {
            x: { a: particles[basis.x.a], b: particles[basis.x.b]},
            y: { a: particles[basis.y.a], b: particles[basis.y.b]},
            z: { a: particles[basis.z.a], b: particles[basis.z.b]}
        };

        // objects are only alive if they haven't hit the ground, we call that "stunned", and start
        // out *NOT* stunned
        this.stun = false;
    };

    _.updateCoordinateFrame = function (deltaTime) {
        let particles = this.particles;

        // the model defines the basis vectors as the vectors between three given pairs of
        // particles, and we iterate over the solution to create a rigid, perpendicular basis
        let basis = this.basis;
        let X = Float3.normalize (Float3.subtract (basis.x.a.position, basis.x.b.position));
        let Y = Float3.normalize (Float3.subtract (basis.y.a.position, basis.y.b.position));
        let Z = Float3.normalize (Float3.subtract (basis.z.a.position, basis.z.b.position));

        /*
        let Z2 = Float3.cross (X, Y);
        Z = Float3.normalize (Float3.add (Z, Z2));
        X = Float3.cross (Y, Z);
        */

        // build the transform...
        this.transform = Float4x4.inverse (Float4x4.viewMatrix (X, Y, Z, this.position));
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

        // compute the centroid of the particles as the translation for the updated transformation
        // matrix
        let position = [0, 0, 0];
        for (let particle of particles) {
            position = Float3.add (position, Float3.scale (particle.position, particle.mass));
        }
        this.position = Float3.scale (position, 1 / this.totalMass);

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
