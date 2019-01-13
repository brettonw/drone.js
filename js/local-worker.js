"use strict;"

let LocalWorker = function () {
    let _ = Object.create (ClassBase);

    _.construct = function (parameters) {
        let objectType = eval (parameters.workerObjectType);
        this.object = objectType.new (parameters);
    };

    _.postMessage = function (message) {
        if (message.command in this.object) {
            // only one parameter is allowed
            this.object[message.command].call (this.object, Object.values (message.parameters)[0]);
        }
    };

    return _;
} ();

