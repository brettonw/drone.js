"use strict;"

let RollingAverage = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.count = Utility.defaultValue (parameters.count, 10);
        this.history = [];
        this.current = 0;
        this.sum = 0;
    };

    _.update = function (value) {
        this.sum += value;
        if (this.history.length < this.count) {
            this.history.push(value);
            return this.sum / this.history.length;
        } else {
            this.sum -= this.history[this.current];
            this.history[this.current] = value;
            this.current = (this.current + 1) % this.count;
            return this.sum / this.count;
        }
    };

    return _;
} ();

