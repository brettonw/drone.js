"use strict;"


let scene;
let standardUniforms = Object.create (null);

// the default deltaTime
deltaTime = 1.0 / targetFrameRate;
const targetFrameDeltaTimeMs = 1000.0 * deltaTime * 0.9;

let fpsHistory = RollingAverage.new ({ count: targetFrameRate });

let droneOne;

let animateCheckbox;
let displayFpsSpan;

let visibilityState = document.visibilityState;
document.addEventListener("visibilitychange", function (event) {
    //console.log ("Visbility State changed to '" + document.visibilityState + "'");
    visibilityState = document.visibilityState;
    updateRunFocus ();
});

let windowFocusState = "focus";
window.addEventListener("focus", function (event) {
    windowFocusState = "focus";
    //console.log ("Window Focus");
    updateRunFocus ();
});
window.addEventListener("blur", function (event) {
    windowFocusState = "blur";
    //console.log ("Window Blur");
    updateRunFocus ();
});

let runFocus = true;
let updateRunFocus = function () {
    if ((visibilityState === "visible") && (windowFocusState === "focus")) {
        runFocus = true;
        document.getElementById("render-canvas").focus ();
        drawFrame ();
    }
    else {
        runFocus = false;
    }
};

let lastTime = new Date ().getTime ();
let drawFrame = function () {
    if ((animateCheckbox.checked) && (runFocus === true)) {
        // draw again as fast as possible
        window.requestAnimationFrame (drawFrame);

        // compute the updated time (in ms)
        let nowTime = new Date ().getTime ();
        let frameDeltaTime = nowTime - lastTime;
        if (frameDeltaTime < targetFrameDeltaTimeMs) {
            // if not enough time has past, skip this frame...
            return;
        }
        lastTime = nowTime;

        // update the fps
        let averageDeltaTimeMs = fpsHistory.update (frameDeltaTime);
        let fps = 1000.0 / averageDeltaTimeMs;
        displayFpsSpan.innerHTML = Utility.padNum(fps.toFixed(1), 3) + " fps";

        // NOTE - might need to control the order of thing updates, especially the world state first
        Thing.updateAll (deltaTime);
    }

    standardUniforms.PROJECTION_MATRIX_PARAMETER = Float4x4.perspective (25, context.viewportWidth / context.viewportHeight, 0.1, 64);
    let cameraDeltaVectorLength = Float3.norm (droneOne.drone.position);
    let cameraDeltaVector = Float3.add (Float3.scale (droneOne.drone.position, (1 / cameraDeltaVectorLength)  * (cameraDeltaVectorLength + 5)), [-0.75, 2.75, 4.25]);
    standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (cameraDeltaVector, droneOne.drone.position, [0, 1, 0]);
    //standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (Float3.add ([0, 2, 4], droneOne.drone.position), droneOne.drone.position, [0, 1, 0]);
    standardUniforms.MODEL_MATRIX_PARAMETER = Float4x4.identity ();

    // compute the camera position and set it in the standard uniforms
    let vmi = Float4x4.inverse (standardUniforms.VIEW_MATRIX_PARAMETER);
    standardUniforms.CAMERA_POSITION = [vmi[12], vmi[13], vmi[14]];
    //console.log ("CAMERA AT: " + Float3.str (standardUniforms.CAMERA_POSITION));

    // draw the scene
    scene.traverse (standardUniforms);
};

let buildScene = function () {
    worldState = WorldState.new ();

    // create a surface of revolution for the prop rings
    let rp = [[0.4375, 0.0500], [0.4250, 0.0500], [0.4375, 0.0000], [0.4250, -0.0500], [0.4375, -0.0500], [0.4500, -0.0167], [0.4500, 0.0167]];
    let rpIndex = [0, 6, 5, 4, 4, 3,  3,  2,  1, 1, 0];
    let rpSign =  [1, 1, 1, 1, 0, 0, -1, -1, -1, 0, 0];
    let rpi = function (i) { return rp[rpIndex[i]]; };
    let rpn = function (i) { return Float2.scale (Float2.normalize (rp[rpIndex[i]]), rpSign[i]); };
    makeRevolve ("ring",
        [rpi(0), rpi (1), rpi (2), rpi (3), rpi (4), rpi (5), rpi (6), rpi (7), rpi (8), rpi (9), rpi (10)],
        [rpn (0), rpn (1), rpn (2), rpn (3), [0.0, -1.0], [0.0, -1.0], rpn (6), rpn (7), rpn (8), [0.0, 1.0], [0.0, 1.0],],
        48);


    //gl = canvas.getContext ("webgl", { alpha: false });
    scene = Node.new ({
        transform: Float4x4.scale ([1, 1, 1]),
        state: function (standardUniforms) {
            // ordinarily, webGl will automatically present and clear when we return control to the
            // event loop from the draw function, but we overrode that to have explicit control.
            // webGl still presents the buffer automatically, but the back buffer is not cleared
            // until we do it...
            context.clearColor (0.0, 0.0, 0.0, 1.0);
            context.clear (context.COLOR_BUFFER_BIT | context.DEPTH_BUFFER_BIT);

            // back face culling enabled, and full z-buffer utilization
            context.enable (context.CULL_FACE);
            context.cullFace (context.BACK);
            context.enable (context.DEPTH_TEST);
            context.depthMask (true);

            // extensions I want for getting gradient information inside the fragment shaders
            //context.getExtension ("OES_standard_derivatives");
            //context.getExtension ("EXT_shader_texture_lod");

            // oh for &#^%'s sake, alpha blending should be standard
            context.blendFunc (context.SRC_ALPHA, context.ONE_MINUS_SRC_ALPHA);
            context.enable (context.BLEND);

            // a little bit of setup for lighting
            standardUniforms.OUTPUT_ALPHA_PARAMETER = 1.0;
            standardUniforms.AMBIENT_LIGHT_COLOR = [0.8, 0.8, 1.0];
            standardUniforms.LIGHT_COLOR = [1.0, 1.0, 0.8];
            standardUniforms.LIGHT_DIRECTION = Float3.normalize ([1.55, 1.75, 1.45]);
            standardUniforms.AMBIENT_CONTRIBUTION = 0.25;
            standardUniforms.DIFFUSE_CONTRIBUTION = 0.75;
            standardUniforms.SPECULAR_CONTRIBUTION = 0.25;
            standardUniforms.SPECULAR_EXPONENT = 8.0;
            standardUniforms.TEXTURE_SAMPLER = "xxx";

        }
    }, "root")
        .addChild (Node.new ({
            transform: Float4x4.chain (Float4x4.scale (4), Float4x4.rotateX (Math.PI / -2)),
            state: function (standardUniforms) {
                Program.get ("basic-texture").use ();
                standardUniforms.TEXTURE_SAMPLER = "grid-16x16";
                standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
                standardUniforms.OUTPUT_ALPHA_PARAMETER = 1.0;
            },
            shape: "square",
            children: false
        }))
    ;

    // add some "space" nodes to give a sense of motion, spaced at 1x1, from -12 .. 12
    let spaceHigh = 16;
    let spaceLow = -spaceHigh;
    let spaceSpacing = 2;
    for (let i = spaceLow; i <= spaceHigh; i+= spaceSpacing) {
        for (let k = spaceLow; k <= spaceHigh; k+= spaceSpacing) {
            if ((Math.abs (i) >= 4) || (Math.abs (k) >= 4)) {
                scene.addChild (Node.new ({
                    transform: Float4x4.chain (Float4x4.scale (0.02), Float4x4.translate ([i, 0, k])),
                    state: function (standardUniforms) {
                        Program.get ("basic").use ();
                        //standardUniforms.OUTPUT_ALPHA_PARAMETER = 0.5;
                        let color = 1.0 - (Math.sqrt ((i * i) + (k * k)) / Math.sqrt (2 * (spaceHigh * spaceHigh)));
                        standardUniforms.MODEL_COLOR = [color, color, color];
                    },
                    shape: "cube",
                    children: false
                }));
            }
        }
    }

    // lay down a pack of drones
    let droneHigh = 3;
    let droneLow = -droneHigh;
    let droneSpacing = 3;
    for (let i = droneLow; i <= droneHigh; i += droneSpacing) {
        for (let j = droneLow; j <= droneHigh; j += droneSpacing) {
            if (! ((i === 0) && (j === 0))) {
                DroneTester.new ({ goal: [i, 1.5, j] }).addToScene (scene);
            }
        }
    }

    // create the drone, the transform applies to the first configuration to give the initial
    // flight configuration of the drone
    droneOne = DroneTester.new ({ goal: [0, 1.5, 0], debug: false }, "one").addToScene (scene);

    drawFrame ();
};

let clickAnimateCheckbox = function () {
    if (animateCheckbox.checked) {
        lastTime = new Date ().getTime ();
        drawFrame ();
    }
};

let main = function () {
    animateCheckbox = document.getElementById("animateCheckbox");
    displayFpsSpan = document.getElementById("displayFpsSpan");

    // create the render object
    Render.new ({
        canvasId: "render-canvas",
        loaders: [
            LoaderPath.new ({ type: Texture, path: "textures/@.png" }).addItems (["grid-16x16", "drone-deck", "drone-props"], { generateMipMap: true }),
        ],
        onReady: OnReady.new (null, buildScene)
    });
};
