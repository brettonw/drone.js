"use strict;"

let Model = function () {
    let _ = ClassNamed (CLASS_NAME_GENERATED);

    _.construct = function (parameters) {
        // get the drone geometry from the file
        let inputModel = JSON.parse(TextFile.get (parameters.model).text);
        this.particles = inputModel.particles;
        this.struts = inputModel.struts;
        this.basis = inputModel.basis;
        this.motors = inputModel.motors;

        // the points of the model might have been defined in a "comfortable" way, where the
        // centroid is not at the origin; first, we compute the center of mass
        let centerOfMass = [0, 0, 0];
        this.totalMass = 0;
        for (let particle of this.particles) {
            centerOfMass = Float3.add (centerOfMass, Float3.scale (particle.position, particle.mass));
            this.totalMass += particle.mass;
        }
        centerOfMass = Float3.scale (centerOfMass, 1 / this.totalMass);

        // and then and relocate the points so the model's center of mass is at the origin.
        for (let particle of this.particles) {
            particle.position = Float3.subtract (particle.position, centerOfMass);
        }
    };

    return _;
} ();
