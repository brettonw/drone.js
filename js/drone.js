"use strict;"

let Drone = function () {
    let _ = Object.create (Thing);

    let computePosition = function (particles) {
        let position = Float3.create().fill (0);
        let totalMass = 0;
        for (let particle of particles) {
            position = Float3.add (position, Float3.scale (particle.position, particle.mass));
            totalMass += particle.mass;
        }
        position = Float3.scale (position, 1 / totalMass);
        return position;
    };

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        let transform = this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // define a group of particles that we will use to build our triangulated, stable structure
        let particles = this.particles = [
            { position: [ 1.0,  0.5, -1.0], mass: 100 }, // 0
            { position: [ 0.0,  0.5, -0.6], mass: 125 }, // 1
            { position: [-1.0,  0.5, -1.0], mass: 100 }, // 2
            { position: [-0.6,  0.5,  0.0], mass: 125 }, // 3
            { position: [-1.0,  0.5,  1.0], mass: 100 }, // 4
            { position: [ 0.0,  0.5,  0.6], mass: 125 }, // 5
            { position: [ 1.0,  0.5,  1.0], mass: 100 }, // 6
            { position: [ 0.6,  0.5,  0.0], mass: 125 }, // 7
            { position: [ 0.0, -0.2,  0.0], mass: 150 }  // 8
        ];

        // the points might have been defined in a "comfortable" way, where the centroid is not at
        // the origin; we'll compute the centroid and relocate the points - so the position is at
        // the origin
        let position = computePosition (this.particles);
        for (let particle of particles) {
            particle.position = Float3.subtract (particle.position, position);
        }

        // position *SHOULD* be at the origin
        //this.position = computePosition (this.particles);
        this.position = [0, 0, 0];

        // some of the particle nodes will flash brightly to resemble running lights. these control
        // the flash timing
        let flashers = this.flashers = [];
        for (let i = 0; i < 5; ++i) {
            flashers.push (Flasher.new ());
        }

        // copy if the drone is created in a debug state
        this.debug = Utility.defaultValue (parameters.debug, false);

        // create a web worker to run the drone simulation
        if (typeof(Worker) !== "undefined") {
            let worker = this.worker = new Worker ("js/worker.js");
            let that = this;
            worker.addEventListener("message", function (event) {
                that.handleWorkerResponse(event.data);
            });
            // pass the particle to the
            parameters.particles = particles;
            this.postMessage ("start", parameters);
        } else {
            // web workers aren't supported?
            console.log ("ERROR - no web workers");
        }
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

        let particles = this.particles;

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
        for (let i = 0; i < particles.length;  ++i) {
            let particle = particles[i];
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
