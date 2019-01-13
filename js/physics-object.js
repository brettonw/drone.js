"use strict;"

let PhysicsObject = function () {
    let _ = Object.create (Thing);

    _.startWorker = function (parameters) {
        // include a few details in the parameters
        parameters.model = this.model;
        parameters.workerObjectType = Utility.defaultValue (parameters.workerObjectType, "PhysicsWorker");

        // check to see if the worker should be threaded
        let useThread = Utility.defaultValue (parameters.useThread, true);

        // if we want to use threads - let's try...
        if (useThread === true) {
            if (typeof(Worker) !== "undefined") {
                this.worker = new Worker ("js/threaded-worker.js");
                let that = this;
                this.worker.addEventListener("message", function (event) {
                    that.handleWorkerResponse(event.data);
                });

                this.postMessage ("start", parameters);
            } else {
                // web workers aren't supported?
                useThread = false;
            }
        }

        // if we didn't want threads, or we failed to get a thread...
        if (! useThread) {
            this.worker = LocalWorker.new (parameters);
        }
    };

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // get the drone geometry from the file
        this.model = Model.new ({ model: parameters.model });

        // position *SHOULD* be at the origin
        this.position = [0, 0, 0];

        // copy if the object is created in a debug state
        this.debug = Utility.defaultValue (parameters.debug, false);

        // start a worker - threaded if we want to
        this.startWorker (parameters);
    };

    _.postMessage = function (command, parameters) {
        this.worker.postMessage ({command: command, parameters: parameters});
    };

    _.handleWorkerResponse = function (response) {
        //console.log ("got it (" + response + ")");
        switch (response.command) {
            case "update":
                let transform = response.parameters.transform;
                Node.get (this.name + " (drone-model)").transform = transform;
                this.position = [transform[12], transform[13], transform[14]];
                break;
            case "error":
                console.log (response.parameters.description);
                break;
        }
    };

    _.update = function (deltaTime) {
        this.postMessage ("update", { deltaTime: deltaTime });
        if ("object" in this.worker) {
            let transform = this.worker.object.transform;
            Node.get (this.name + " (drone-model)").transform = transform;
            this.position = [transform[12], transform[13], transform[14]];
        }
    };

    _.addToScene = function (parentNode) {
    };

    return _;
} ();
