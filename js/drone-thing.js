"use strict;"

let DroneThing = function () {
    let _ = Object.create (Thing);

    _.construct = function (parameters) {
        this.last = { p: 0.0, i: 0.0 };
        this.deltaFunction = Utility.defaultValue (parameters.deltaFunction, function (x, y) {
            return y - x;
        });
        this.outputScale = Utility.defaultValue (parameters.outputScale, 1.0);
        this.gain = Utility.defaultValue (parameters.gains, { p: 2.0, i: 0.0, d: 20.0 });
    };

    // update will return an output that is the effort to exert on the system input
    _.update = function (pNew, pGoal, deltaTime, deltaFunction) {
        // compute PID, and update the history
        let last = this.last;
        let p = this.deltaFunction (pNew, pGoal);
        let i = last.i + p;
        let d = p - last.p;
        last.p = p;
        last.i = i;
        //console.log ("p = " + p.toFixed(4) + ", i = " + i.toFixed(4) + ", d = " + d.toFixed(4));

        // the PID gain values may be sensitive to update rate - so we include the deltaTime value
        // the D value is definitely sensitive to update rate

        // compute the PID weighted response
        let response = (p * this.gain.p) + (i * this.gain.i) + ((d / deltaTime) * this.gain.d);
        return this.outputScale * Math.clamp(response, -1.0, 1.0);
    };

    return _;
} ();

