"use strict;"

let DroneTester = function () {
    let _ = Object.create (DroneThing);

    _.construct = function (parameters) {
        this.countdownTime = this.countdownDuration = 3 + (Math.random () * 3);

        // call the super class constructor
        Object.getPrototypeOf(_).construct.call(this, parameters);
    };

    _.update = function (deltaTime) {
        // check if the drone is at the goal
        let distanceToTarget = Float3.norm (Float3.subtract (this.drone.position, this.goal));
        if (distanceToTarget < 0.25) {
            // it is, so start counting down until time to leave
            this.countdownTime -= deltaTime;

            // check if it's time to leave
            if (this.countdownTime < 0) {
                // it is, so reset the countdown
                this.countdownTime = this.countdownDuration;
                /*
                let controller = navigator.getGamepads()[0];
                let axes = controller ? controller.axes : [0, 0, 0, 0];
                */

                // establish a new goal
                const radius = 15.0;
                let newGoal = Float3.copy (this.goal);

                // force the thing to make big moves
                while (Float3.norm (Float3.subtract (newGoal, this.goal)) < (radius * 1.25)) {
                    newGoal[0] = (Math.floor (Math.random () * (radius / 3) * 2) * 3) - radius;
                    newGoal[1] = 1.5 + (Math.floor (Math.random () * (radius / 3)) * 3);
                    newGoal[2] = (Math.floor (Math.random () * (radius / 3) * 2) * 3) - radius;
                }
                this.goto (newGoal);
            }
        }
    };

    return _;
} ();

