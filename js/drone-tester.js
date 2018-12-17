"use strict;"

let DroneTester = function () {
    let _ = Object.create (DroneThing);

    _.construct = function (parameters) {
        this.countdownDuration = 8 + (Math.random () * 8);
        this.countdownTime = 5 + (Math.random () * 5);

        // call the super class constructor
        Object.getPrototypeOf(_).construct.call(this, parameters);
    };

    _.update = function (deltaTime) {
        this.countdownTime -= deltaTime;
        if (this.countdownTime < 0) {
            this.countdownTime = this.countdownDuration;
            /*
            let controller = navigator.getGamepads()[0];
            let axes = controller ? controller.axes : [0, 0, 0, 0];
            */
            let newGoal = Float3.copy (this.goal);
            let radius = 15.0;

            // force the thing to make big moves
            while (Float3.norm (Float3.subtract (newGoal, this.goal)) < radius) {
                newGoal[0] = Math.floor (Math.random () * radius * 2) - radius;
                newGoal[1] = 1.5 + Math.floor (Math.random () * radius);
                newGoal[2] = Math.floor (Math.random () * radius * 2) - radius;
            }
            this.goto (newGoal);
        }
    };

    return _;
} ();

