"use strict;"

let DroneThing = function () {
    let _ = Object.create (Thing);

    _.construct = function (parameters) {
        this.countdownDuration = 8 + (Math.random () * 8);
        this.countdownTime = 5 + (Math.random () * 5);
        this.goal = Utility.defaultValue (parameters.goal, [0, 2, 0]);
        this.drone = Drone.new ({ transform: Float4x4.translate (this.goal) });
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

    _.goto = function (newGoal) {
        Node.get (this.name + " (target)").transform = Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate (newGoal));
        let delta = Float3.norm (Float3.subtract (newGoal, this.goal));
        //console.log (this.name + ", go from: " + Float3.str (this.goal) + ", to: " + Float3.str (newGoal) + ", distance: " + delta.toFixed (3));
        this.goal = newGoal;
        this.drone.setGoal (newGoal);
        return this;
    };

    _.addToScene = function (parentNode) {
        // add the target node before the drone so the props don't clip it
        parentNode.addChild (Node.new ({
            transform: Float4x4.identity (),
            state: function (standardUniforms) {
                Program.get ("basic").use ();
                standardUniforms.MODEL_COLOR = [0.3, 0.7, 1.0];
                //standardUniforms.OUTPUT_ALPHA_PARAMETER = 0.5;
            },
            shape: "sphere2",
            children: false
        }, this.name + " (target)"));

        this.drone.addToScene (parentNode);

        return this.goto (this.goal);
    };

    return _;
} ();

