"use strict;"

// this class implements a generic Proportional, Integral, Derivative (PID) controller - the p-term
// is often considered the "error", the delta between the current state and the desired state.
let PID = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.p = 0.0;
        this.i = 0.0;
        this.last = { p: 0.0, i: 0.0 };
        this.deltaFunction = Utility.defaultValue (parameters.deltaFunction, function (x, y) {
            return y - x;
        });
        this.outputScale = Utility.defaultValue (parameters.outputScale, 1.0);
        this.gains = Utility.defaultValue (parameters.gains, { p: 2.0, i: 0.0, d: 20.0 });
    };

    // update will return an output that is the effort to exert on the system input
    _.update = function (measuredValue, setValue, deltaTime, deltaScale) {
        // compute PID, and update the history (see https://en.wikipedia.org/wiki/PID_controller)
        // the i and d components are senstive to the deltaTime

        deltaScale = Utility.defaultValue (deltaScale, 1.0);

        let last = this.last;
        this.v = measuredValue;
        let p = this.deltaFunction (measuredValue, setValue) * deltaScale;
        let i = this.i = this.i + (p * deltaTime);
        let d = (p - this.p) / deltaTime;
        this.p = p;
        //console.log ("p = " + p.toFixed(4) + ", i = " + i.toFixed(4) + ", d = " + d.toFixed(4));

        // compute the PID weighted response
        let response = (p * this.gains.p) + (i * this.gains.i) + (d * this.gains.d);
        return this.outputScale * Math.clamp(response, -1.0, 1.0);
    };

    return _;
} ();

