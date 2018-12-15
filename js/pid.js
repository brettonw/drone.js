"use strict;"

let PID = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.last = { p: 0.0, i: 0.0 };
        this.gain = Utility.defaultValue (parameters.gains, { p: 2.0, i: 0.0, d: 20.0 });
    };

    // update will return an output that is the effort to exert on the system input
    _.update = function (pNew, pGoal, deltaFunction) {
        deltaFunction = (typeof deltaFunction !== "undefined") ? deltaFunction : function (x, y) {
            return y - x;
        };
        // compute PID, and update the history
        let last = this.last;
        let p = deltaFunction (pNew, pGoal);
        let i = last.i + p;
        let d = p - last.p;
        last.p = p;
        last.i = i;
        //console.log ("p = " + p.toFixed(4) + ", i = " + i.toFixed(4) + ", d = " + d.toFixed(4));

        // compute the PID weighted response
        let response = (p * this.gain.p) + (i * this.gain.i) + (d * this.gain.d);
        return Math.clamp(response, -1.0, 1.0);
    };

    return _;
} ();

