"use strict;"


let scene;
let standardUniforms = Object.create (null);

// the default deltaTime
deltaTime = 1.0 / targetFrameRate;
const targetFrameDeltaTimeMs = 1000.0 * deltaTime * 0.9;

let fpsHistory = RollingAverage.new ({ count: targetFrameRate });

let droneOne;
let droneOneHistory = [];
let droneOneHistoryIndex = 0;
let trail;

let animateCheckbox;
let showTrailCheckbox;
let cameraSelect;
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
let lastGoal = [0, 0, 0];
let currentGoal = [0, 0, 0];
let newMidPoint = [0, 0, 0];
let cameraOffset = [0, 2, 0];
let newCameraPosition = [-1, 10, 10];
let lastCameraPosition = Float3.copy (newCameraPosition);
let lastMidPoint = [0, 0, 0];
let upVector = [0, 1, 0];
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

        droneOneHistory[droneOneHistoryIndex].transform = Float4x4.chain (Float4x4.scale (0.025), Float4x4.translate (droneOne.drone.position));
        droneOneHistoryIndex = [droneOneHistoryIndex + 1] % droneOneHistory.length;
    }

    let fovDegrees = 30;
    standardUniforms.PROJECTION_MATRIX_PARAMETER = Float4x4.perspective (fovDegrees, context.viewportWidth / context.viewportHeight, 0.1, 128);

    /*
    let oldestHistoryIndex = (droneOneHistoryIndex + 1) % droneOneHistory.length;
    let oldestTransform = droneOneHistory[oldestHistoryIndex].transform;
    let oldestPosition = [oldestTransform[12], oldestTransform[13], oldestTransform[14]];
    let cameraDeltaVector = Float3.subtract (droneOne.drone.position, oldestPosition);
    let cameraDeltaVectorLength = Float3.norm (cameraDeltaVector);
    cameraDeltaVector = Float3.add (Float3.add (Float3.scale (cameraDeltaVector, (1 / cameraDeltaVectorLength) * 5), droneOne.drone.position), [-0.5, 1, 1]);
    standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (cameraDeltaVector, oldestPosition, [0, 1, 0]);
    */

    let showTrail = true;
    switch (cameraSelect.value) {
        case "Wide Sweep": {
            if (!Float3.equals (droneOne.goal, currentGoal)) {
                lastGoal = currentGoal;
                newMidPoint = Float3.scale (Float3.add (currentGoal, droneOne.goal), 0.5);
                currentGoal = droneOne.goal;

                // compute the newCameraPosition, at the midpoint, combine the up-vector with the mid->goal
                // vector, and then move it out far enough to ensure the goal is in view
                let midPointToGoalVector = Float3.subtract (currentGoal, newMidPoint);
                let normalizedMidPointToGoalVector = Float3.normalize (midPointToGoalVector);
                //let offsetVector = Float3.normalize (Float3.add (upVector, normalizedMidPointToGoalVector));

                let perp = Float3.cross (upVector, normalizedMidPointToGoalVector);
                let offsetVector = Float3.normalize (Float3.add (perp, upVector));
                let ptU = Float3.add (newMidPoint, Float3.scale (offsetVector, Float3.dot (offsetVector, midPointToGoalVector)));

                let opp = Float3.norm (Float3.subtract (ptU, currentGoal)) + 3;
                let adj = (opp / Utility.tan ((16 / 9) * (fovDegrees / 2)));
                newCameraPosition = Float3.add (Float3.add (ptU, Float3.scale (offsetVector, adj)), cameraOffset);

                // add in a bit of perpendicular offset
            }
            // linear combination of the frame positions so the camera takes a full second to make the transition
            lastCameraPosition = Float3.scale (Float3.add (newCameraPosition, Float3.scale (lastCameraPosition, targetFrameRate - 1)), 1 / targetFrameRate);
            let midPointToGoalVector = Float3.subtract (currentGoal, newMidPoint);
            let normalizedMidPointToGoalVector = Float3.normalize (midPointToGoalVector);
            let extendedGoalPosition = Float3.add (Float3.add (currentGoal, Float3.scale (normalizedMidPointToGoalVector, 10)), Float3.scale (upVector, 0));
            newCameraPosition = Float3.scale (Float3.add (newCameraPosition, Float3.scale (extendedGoalPosition, 0.01)), 1 / 1.01);

            lastMidPoint = Float3.scale (Float3.add (newMidPoint, Float3.scale (lastMidPoint, targetFrameRate - 1)), 1 / targetFrameRate);

            standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (lastCameraPosition, lastMidPoint, upVector);

        } break;

        case "OTS": {
            let cameraDeltaVectorLength = Float3.norm (droneOne.drone.position);
            let cameraDeltaVector = Float3.add (Float3.scale (droneOne.drone.position, (1 / cameraDeltaVectorLength) * (cameraDeltaVectorLength + 5)), [-0.75, 2.75, 4.25]);
            standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (cameraDeltaVector, droneOne.drone.position, [0, 1, 0]);
        } break;

        case "Z-Axis Locked": {
            standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (Float3.add ([0, 2, 4], droneOne.drone.position), droneOne.drone.position, [0, 1, 0]);
            showTrail = false;
        } break;
    }

    standardUniforms.MODEL_MATRIX_PARAMETER = Float4x4.identity ();

    // compute the camera position and set it in the standard uniforms
    let vmi = Float4x4.inverse (standardUniforms.VIEW_MATRIX_PARAMETER);
    standardUniforms.CAMERA_POSITION = [vmi[12], vmi[13], vmi[14]];
    //console.log ("CAMERA AT: " + Float3.str (standardUniforms.CAMERA_POSITION));

    // draw the scene
    trail.enabled = showTrailCheckbox.checked && (showTrail === true);
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

    // lay down a few seconds of history
    trail = Node.new ({
        transform: Float4x4.identity (),
        state: function (standardUniforms) {
            Program.get ("color").use ();
            standardUniforms.OUTPUT_ALPHA_PARAMETER = 0.25;
            standardUniforms.MODEL_COLOR = [1.0, 1.0, 0.0];
        }
    }, "trail");
    scene.addChild (trail);
    for (let i = 0; i < (targetFrameRate * 8); ++i) {
        let node = Node.new ({
            transform: Float4x4.chain (Float4x4.scale (0.01), Float4x4.translate ([0, 1.49, 0])),
            shape: "sphere2",
            children: false
        });
        trail.addChild (node);
        droneOneHistory.push (node);
    }

    // lay down a pack of drones
    /*
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
    */

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
    showTrailCheckbox = document.getElementById ("showTrailCheckbox");
    cameraSelect = document.getElementById ("cameraSelect");
    displayFpsSpan = document.getElementById("displayFpsSpan");

    // create the render object
    Render.new ({
        canvasId: "render-canvas",
        loaders: [
            LoaderPath.new ({ type: Texture, path: "texture/@.png" }).addItems (["grid-16x16", "drone-deck", "drone-props"], { generateMipMap: true }),
            LoaderPath.new ({ type: TextFile, path: "model/@.json" }).addItems (["drone", "star"]),
        ],
        onReady: OnReady.new (null, buildScene)
    });
};
