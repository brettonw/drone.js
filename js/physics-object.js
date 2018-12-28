"use strict;"

let PhysicsObject = function () {
    let _ = Object.create (Thing);

    _.startWorker = function (parameters) {
        // check to see if the worker should be threaded
        let useThread = Utility.defaultValue (parameters.useThread, true);

        // if we want to use threads - let's try...
        if (useThread === true) {
            if (typeof(Worker) !== "undefined") {
                let workerName = Utility.defaultValue (parameters.workerName, "worker.js");
                let worker = this.worker = new Worker ("js/" + workerName);
                let that = this;
                worker.addEventListener("message", function (event) {
                    that.handleWorkerResponse(event.data);
                });

                // pass the model to the worker in the parameters
                parameters.model = model;
                this.postMessage ("start", parameters);
            } else {
                // web workers aren't supported?
                useThread = false;
            }
        }

        // if we didn't want threads, or we failed to get a thread...
        if (! useThread) {

        }
    };

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        let transform = this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // get the drone geometry from the file
        let model = this.model = Model.new ({ model: parameters.model });

        // position *SHOULD* be at the origin
        this.position = [0, 0, 0];

        // copy if the object is created in a debug state
        this.debug = Utility.defaultValue (parameters.debug, false);

        // start a worker - threaded if we want to
        startWorker (parameters);
    };

    _.postMessage = function (command, parameters) {
        this.worker.postMessage ({command: command, parameters: parameters});
    };

    _.handleWorkerResponse = function (response) {
        //console.log ("got it (" + response + ")");
        switch (response.command) {
            case "update":
                let transform = response.parameters.transform;;
                Node.get (this.name + " (drone-model)").transform = transform;
                this.position = [transform[12], transform[13], transform[14]];
                break;
            case "error":
                console.log (response.parameters.description);
                break;
        }
    };

    _.update = function (deltaTime) {
        // update the flashers
        for (let flasher of this.flashers) {
            flasher.update(deltaTime);
        }

        this.postMessage ("update", { deltaTime: deltaTime });
    };

    _.setGoal = function (xyz) {
        this.postMessage ("set-goal", { xyz: xyz });
    };

    _.addToScene = function (parentNode) {
        console.log ("Drone named: " + this.name);
        let that = this;

        // put down a representation of the drone geometry for visual purposes
        let modelNode = Node.new ({
            transform: this.transform,
            state: function (standardUniforms) {
                standardUniforms.OUTPUT_ALPHA_PARAMETER = 1.0;
            },
        }, this.name + " (drone-model)")
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([-0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([-0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.translate ([0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 0.6, 0.125];
                },
                shape: "ring",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([-0.62, 0.05, 0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([-0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.scale (0.05), Float4x4.translate ([0.62, 0.05, -0.62])),
                state: function (standardUniforms) {
                    Program.get ("basic").use ();
                    standardUniforms.MODEL_COLOR = [0.1, 0.1, 0.1];
                },
                shape: "sphere2",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.rotateX (Math.PI / -2), Float4x4.scale ([1.1, 1, 1.1])),
                state: function (standardUniforms) {
                    Program.get ("basic-texture").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                    standardUniforms.TEXTURE_SAMPLER = "drone-deck";
                },
                shape: "square",
                children: false
            }))
            .addChild (Node.new ({
                transform: Float4x4.chain (Float4x4.rotateX (Math.PI / -2), Float4x4.translate ([0, 0.05, 0]), Float4x4.scale ([1.1, 1, 1.1])),
                state: function (standardUniforms) {
                    Program.get ("basic-texture").use ();
                    standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                    standardUniforms.TEXTURE_SAMPLER = "drone-props";
                },
                shape: "square",
                children: false
            }));

        // add the actual particles below the model node, so they transform with the drone
        for (let i = 0; i < this.model.particles.length;  ++i) {
            let particle = this.model.particles[i];
            modelNode.addChild(Node.new({
                transform: Float4x4.chain ( Float4x4.scale (0.0002 * particle.mass), Float4x4.translate (particle.position)),
                state: function (standardUniforms) {
                    switch (i) {
                        case 0: standardUniforms.MODEL_COLOR = [0.0, 0.25 + (that.flashers[0].getIsOn() * 0.75), 0.0]; Program.get("color").use(); break;
                        case 6: standardUniforms.MODEL_COLOR = [0.0, 0.25 + (that.flashers[1].getIsOn () * 0.75), 0.0]; Program.get("color").use(); break;
                        case 2: standardUniforms.MODEL_COLOR = [0.25 + (that.flashers[2].getIsOn () * 0.75), 0.0, 0.0]; Program.get("color").use();break;
                        case 4: standardUniforms.MODEL_COLOR = [0.25 + (that.flashers[3].getIsOn () * 0.75), 0.0, 0.0]; Program.get("color").use();break;
                        case 5: {
                            let isOn = that.flashers[4].getIsOn ();
                            standardUniforms.MODEL_COLOR = [0.25 + (isOn * 0.75), 0.25 + (isOn * 0.75), 0.2 + (isOn * 0.6)];
                            Program.get ("color").use ();
                            break;
                        }
                        default: standardUniforms.MODEL_COLOR = [0.5, 0.25, 0.125]; Program.get("basic").use();break;
                    }
                },
                shape: "sphere2",
                children: false
            }));
        }

        parentNode.addChild (modelNode);
        return this;
    };

    return _;
} ();
