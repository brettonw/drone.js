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
        let force = Float3.scale (acceleration, this.mass);
        this.applyForce (force);
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

let Constraint = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.a = parameters.a;
        this.b = parameters.b;
        this.length = Float3.norm (Float3.subtract (parameters.particles[this.a].position, parameters.particles[this.b].position))

        // good defaults, damping: 0.5, springConstant (aka: k): 2.0
        this.damping = Utility.defaultValue (parameters.damping, 0.475);
        this.springConstant = Utility.defaultValue (parameters.springConstant, 2.0);
        console.log ("Constraint: (" + this.a + " -> " + this.b + "), length: " + this.length.toFixed(3));
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

let Drone = function () {
    let _ = Object.create (ClassBase);

    _.reset = function () {
        // the drone uses a shape something like a squashed octahedron, with 6 points and 13 edges
        // that define a connected set of 4 tetrahedrons as stable shapes. we assume all of the
        // vertices have a mass of about 125g.
        let particles = this.particles = [
            Particle.new ({ position: [ 1.00,  0.00,  0.00], mass: 125 }), // 0
            Particle.new ({ position: [ 0.00,  0.00,  1.00], mass: 125 }), // 1
            Particle.new ({ position: [-1.00,  0.00,  0.00], mass: 125 }), // 2
            Particle.new ({ position: [ 0.00,  0.00, -1.00], mass: 125 }), // 3
            Particle.new ({ position: [ 0.00,  0.25,  0.00], mass: 125 }), // 4
            Particle.new ({ position: [ 0.00, -0.50,  0.00], mass: 125 })  // 5
        ];

        let constraints = this.constraints = [
            Constraint.new ({particles: particles, a: 4, b: 0}),
            Constraint.new ({particles: particles, a: 4, b: 1}),
            Constraint.new ({particles: particles, a: 4, b: 2}),
            Constraint.new ({particles: particles, a: 4, b: 3}),

            Constraint.new ({particles: particles, a: 4, b: 5}),

            Constraint.new ({particles: particles, a: 5, b: 0}),
            Constraint.new ({particles: particles, a: 5, b: 1}),
            Constraint.new ({particles: particles, a: 5, b: 2}),
            Constraint.new ({particles: particles, a: 5, b: 3}),

            Constraint.new ({particles: particles, a: 0, b: 1}),
            Constraint.new ({particles: particles, a: 1, b: 2}),
            Constraint.new ({particles: particles, a: 2, b: 3}),
            Constraint.new ({particles: particles, a: 3, b: 0})
        ];
    };

    _.construct = function (parameters) {
        // the model is assumed to start at rest
        this.reset ();

        // copy a transformation matrix if one was provided
        this.transformation = Utility.defaultValue (parameters.transformation, Float4x4.identity ());

        this.particles[0].applyForce ([0, 1250, 0]);
    };

    _.update = function (deltaTime) {
        let particles = this.particles;
        let subDeltaTime = deltaTime * 0.1;
        for (let i = 0; i < 10; ++i) {
            // loop over all the particles to update them
            for (let particle of particles) {
                particle.update(subDeltaTime);
            }

            // loop over all the constraints to apply them
            for (let constraint of this.constraints) {
                constraint.apply(this.particles, subDeltaTime);
            }
        }

        // update the scene graph nodes
        for (let i = 0; i < particles.length;  ++i) {
            let particle = particles[i];
            let node = Node.get ("particle-" + i);
            node.transform = Float4x4.chain (
                Float4x4.scale (0.05),
                Float4x4.translate (particle.position)
            );
        }
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
            parentNode.addChild(Node.new({
                transform: Float4x4.identity(),
                state: function (standardUniforms) {
                    Program.get("basic").use();
                    standardUniforms.MODEL_COLOR = [1.0, 0.25, 0.25];
                },
                shape: "cube",
                children: false
            }, "particle-" + i));
        }

    };

    return _;
} ();