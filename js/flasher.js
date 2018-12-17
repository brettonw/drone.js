"use strict;"

let Flasher = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        this.on = (Math.random () > 0.5);
        this.time = Math.random ();
        this.onDuration = 0.1 + ((Math.random () * 0.1) - 0.05);
        this.offDuration = 1.0 + ((Math.random () * 0.3) - 0.15);
    };

    _.update = function (deltaTime) {
        this.time += deltaTime;
        if (this.on) {
            if (this.time > this.onDuration) {
                this.time = 0;
                this.on = false;
            }
        } else {
            if (this.time > this.offDuration) {
                this.time = 0;
                this.on = true;
            }
        }
    };

    _.getIsOn = function () {
        return this.on ? 1.0 : 0.0;
    };

    return _;
} ();

