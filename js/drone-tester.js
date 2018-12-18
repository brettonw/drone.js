"use strict;"

let DroneTester = function () {
    let _ = Object.create (DroneThing);

    _.construct = function (parameters) {
        this.countdownTime = this.countdownDuration = 3 + (Math.random () * 3);

        // call the super class constructor
        Object.getPrototypeOf(_).construct.call(this, parameters);
    };

    const span = 15.0;
    const goalSpacingSq = 40;
    const goalSpacing = Math.sqrt (goalSpacingSq);
    const yOffset = 1.5;

    _.checkGoal = function (goal) {
        let result = false;

        let from = this.goal;
        let deltaToGoal = Float3.subtract (goal, from);
        let distanceToGoal = Float3.norm (deltaToGoal);

        // check that the distance to the goal is sufficient
        if (distanceToGoal > (span * 1.25)) {
            result = true;

            let that = this;
            DroneThing.forEach (function (thing) {
                if ((thing != that) && (result === true) && ("goal" in thing)) {
                    // check if thing.goal is too close to our goal
                    let deltaGoals = Float3.subtract (thing.goal, goal);
                    if (Float3.normSq (deltaGoals) < goalSpacingSq) {
                        result = false;
                    }
                }
            });

            /*
            // normalize the delta
            deltaToGoal = Float3.scale (deltaToGoal, 1 / distanceToGoal);

            // go over all the drones... (except this one)
            let that = this;
            DroneThing.forEach (function (thing) {
                if ((thing != that) && (result === true) && ("goal" in thing)) {
                    // check if thing.goal is too close to a line between from-to - it's not a
                    // guarantee, but it is a start... start by projecting the goal onto our line
                    let deltaToThingGoal = Float3.subtract  (thing.goal, from);

                    // the projection is expressed in terms of lengths of our delta vector, which is
                    // 1, so we can use this to reason about distance directly - if it's negative,
                    // the thing's goal is behind us and we can ignore it - it should have met these
                    // requirements itself... if it's more than "far" away, we can ignore it too...
                    let dot = Float3.dot (deltaToThingGoal, deltaToGoal);
                    if ((dot > -goalSpacing) && (dot < (distanceToGoal + goalSpacing))) {
                        // ok, so it's between us and our goal at *some* distance, but how far?
                        let projectedPoint = Float3.add (from, Float3.scale (deltaToGoal, dot));
                        let projectedDelta = Float3.subtract (thing.goal, projectedPoint);
                        let projectedDeltaLenSq = Float3.normSq (projectedDelta);
                        if (projectedDeltaLenSq < goalSpacingSq) {
                            // this goal is too close to our path - say no
                            //console.log ("REJECT PATH");
                            result = false;
                        }
                    }
                }
            });
            */
        }
        return result;
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
                let newGoal = Float3.copy (this.goal);

                // force the thing to make big moves
                while (! this.checkGoal (newGoal)) {
                    newGoal[0] = Math.floor (Math.random () * span * 2) - span;
                    newGoal[1] = yOffset + Math.floor (Math.random () * span);
                    newGoal[2] = Math.floor (Math.random () * span * 2) - span;
                }
                this.goto (newGoal);
            }
        }
    };

    return _;
} ();

