"use strict;"

// this is just a utility subclass of DroneThing that lets me tune the PIDS without having to
// constantly
let DroneTuner = function () {
    let _ = Object.create (DroneThing);

    _.construct = function (parameters) {
        // call the super class constructor
        parameters.useThread = false;
        Object.getPrototypeOf(_).construct.call(this, parameters);

        // set up the error tracking history for the error terms
        this.history = [];

        // set the drone controller gains to my own PID gains
        let controller = this.controller = this.drone.worker.object.controller;
        controller.locationX.gains =   { p: 0.6, i: 0.0, d: 1.0 };
        controller.locationY.gains =   { p: 0.6, i: 0.0, d: 0.5 };
        controller.locationZ.gains =   { p: 0.6, i: 0.0, d: 1.0 };
        controller.orientation.gains = { p: 0.6, i: 0.0, d: 0.65 };
        controller.tiltX.gains =       { p: 0.6, i: 0.0, d: 0.55 };
        controller.tiltZ.gains =       { p: 0.6, i: 0.0, d: 0.55 };

        // update the drone controller with a function that doesn't apply any external forces
        this.drone.subUpdateParticles = function (subStepDeltaTime) {
            let particles = this.particles;

            // loop over all the distance constraints to apply them
            for (let constraint of this.constraints) {
                constraint.apply (subStepDeltaTime);
            }

            // apply gravity to all the particles, and update them
            for (let particle of particles) {
                particle.applyGravity (subStepDeltaTime);
                particle.update (subStepDeltaTime);
            }

            this.updateCoordinateFrame (subStepDeltaTime);
        };

        this.drone.runController = function (deltaTime) {
            let controller = this.controller;
            let transform = this.transform;
            let goal = this.goal;

            // compute the altitude of the drone using the y component of the translation
            let speed = (controller.locationY.update (transform[13], goal.y, deltaTime) + 1.0) / 2.0;

            // compute the orientation of the drone using the x/z components of the x axis - our goal is
            // to always orient the drone with the x/z axes
            let orientationAngle = Math.atan2(transform[2], transform[0]);
            let turn = -controller.orientation.update (orientationAngle, 0.0, deltaTime);

            // compute the target tilt using as a proxy for target velocity to the target location
            let xVel = controller.locationX.update (transform[12], goal.x, deltaTime);
            let zVel = controller.locationZ.update (transform[14], goal.z, deltaTime);

            // compute the tilt of the drone using the x/z components of the y axis
            let tilt = {
                x: controller.tiltX.update (transform[4], xVel, deltaTime),
                z: controller.tiltZ.update (transform[6], zVel, deltaTime)
            };

            // give the drone the inputs
            this.run (speed, turn, tilt);
        };
    };

    _.update = function (deltaTime) {
        this.goal = [15, 2, 0];
        this.goto (this.goal);
        this.history.push({
            time: worldState.query(WorldState.TIME),
            p: {
                locationX: this.controller.locationX.p,
                locationY: this.controller.locationY.p,
                locationZ: this.controller.locationZ.p,
                orientation: this.controller.orientation.p,
                tiltX: this.controller.tiltX.p,
                tiltZ: this.controller.tiltZ.p
            }
        });

        // check to see if the display needs to be updated
        // now evaluate the function
        let plotDataName = "locationX";

        let plotErrorData = [];
        let timeStart = this.history[0].time;
        for (let dataPoint of this.history) {
            let time = (dataPoint.time - timeStart) * 1.0e-3;
            let value = dataPoint.p[plotDataName];
            if (Math.abs (value) > 0.5) {
                value = Math.sign(value) * 0.5;
            }
            plotErrorData.push({x: time, y: value});
        }

        document.getElementById("plot-error").innerHTML = PlotSvg.setPlotPoints (false).setLegendPosition(440, 340).multipleLine("PID Error (P) for " + plotDataName, "Time", "Error", [plotErrorData]);
    };

    return _;
} ();

