"use strict;"

let DroneObject = function () {
    let _ = Object.create (PhysicsObject);

    _.construct = function (parameters) {
        // copy a transformation matrix if one was provided
        this.transform = Utility.defaultValue (parameters.transform, Float4x4.identity ());

        // get the drone geometry from the file
        parameters.model = Utility.defaultValue (parameters.model, "drone");
        parameters.workerObjectType = Utility.defaultValue (parameters.workerObjectType, "DroneWorker");
        Object.getPrototypeOf(_).construct.call(this, parameters);

        // some of the particle nodes will flash brightly to resemble running lights. these control
        // the flash timing
        let flashers = this.flashers = [];
        for (let i = 0; i < 5; ++i) {
            flashers.push (Flasher.new ());
        }
    };

    _.update = function (deltaTime) {
        // update the flashers
        for (let flasher of this.flashers) {
            flasher.update(deltaTime);
        }

        // do the parental update
        Object.getPrototypeOf(_).update.call(this, deltaTime);
    };

    _.setGoal = function (xyz) {
        this.postMessage ("setGoal", { xyz: xyz });
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
                    //context.disable (context.DEPTH_WRITEMASK);
                    //context.depthMask (false);
                    standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                    standardUniforms.TEXTURE_SAMPLER = "drone-props";
                },
                shape: "square",
                children: false
            }))
            .addChild (Node.new ({
                state: function (standardUniforms) {
                    //context.enable (context.DEPTH_WRITEMASK);
                    //context.depthMask (true);
                },
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
