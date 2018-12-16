"use strict;"

let scene;
let standardUniforms = Object.create (null);

let lastTime = new Date ().getTime ();

const targetFrameRate = 30;
const targetFrameDeltaTimeMs = 1000.0 * (1.0 / targetFrameRate) * 0.9;

const fpsHistoryCount = 32;
let fpsHistory = Array (fpsHistoryCount).fill (0);
let fpsHistoryIndex = 0;
let fpsHistoryAverage = 0;

let droneOne;
let droneTwo;

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

let countdownDuration = 15;
let countdownTime = 8;
let locX = 0;
let locY = 4;
let locZ = 0;

let drawFrame = function () {
    let nowTime = new Date ().getTime ();

    if ((animateCheckbox.checked) && (runFocus === true)) {

        // draw again as fast as possible
        window.requestAnimationFrame (drawFrame);

        // compute the updated time (in ms)
        let deltaTime = nowTime - lastTime;
        if (deltaTime < targetFrameDeltaTimeMs) {
            // if not enough time has past, skip this frame...
            return;
        }

        // update the fps
        fpsHistoryAverage -= fpsHistory[fpsHistoryIndex];
        fpsHistoryAverage += (fpsHistory[fpsHistoryIndex] = deltaTime);
        fpsHistoryIndex = (fpsHistoryIndex + 1) % fpsHistoryCount;

        let averageDeltaTime = fpsHistoryAverage / fpsHistoryCount;
        let fps = 1000.0 / averageDeltaTime;
        displayFpsSpan.innerHTML = Utility.padNum(fps.toFixed(1), 3) + " fps";

        deltaTime = 1.0 / targetFrameRate;
        Thing.updateAll (deltaTime);

        countdownTime -= deltaTime;
        if (countdownTime < 0) {
            countdownTime = countdownDuration;
            /*
            let controller = navigator.getGamepads()[0];
            let axes = controller ? controller.axes : [0, 0, 0, 0];
            */
            let oldLoc = [locX, locY, locZ];
            let newLoc = [locX, locY, locZ];
            let radius = 10.0;
            // force the thing to make big moves
            while (Float3.norm (Float3.subtract (newLoc, oldLoc)) < radius) {
                newLoc[0] = Math.floor (Math.random () * radius * 2) - radius;
                newLoc[1] = 1.5 + Math.floor (Math.random () * radius);
                newLoc[2] = Math.floor (Math.random () * radius * 2) - radius;
            }

            locX = newLoc[0];
            locY = newLoc[1];
            locZ = newLoc[2];
            Node.get ("target").transform = Float4x4.chain (
                Float4x4.scale (0.15),
                Float4x4.translate (newLoc)
            );
            console.log ("goto: (" + locX + ", " + locY + ", " + locZ + ")");
            //locX = 2;
            //locY = 2;
        }
        droneOne.setGoal (locX, locY, locZ);
    }
    lastTime = nowTime;

    standardUniforms.PROJECTION_MATRIX_PARAMETER = Float4x4.perspective (30, context.viewportWidth / context.viewportHeight, 0.1, 32);
    let cameraDeltaVectorLength = Float3.norm (droneOne.position);
    let cameraDeltaVector = Float3.add (Float3.scale (droneOne.position, (1 / cameraDeltaVectorLength)  * (cameraDeltaVectorLength + 5)), [-0.5, 2, 4]);
    standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (cameraDeltaVector, droneOne.position, [0, 1, 0]);
    //standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (Float3.add ([0, 2, 8], droneOne.position), droneOne.position, [0, 1, 0]);
    standardUniforms.MODEL_MATRIX_PARAMETER = Float4x4.identity ();

    // compute the camera position and set it in the standard uniforms
    let vmi = Float4x4.inverse (standardUniforms.VIEW_MATRIX_PARAMETER);
    standardUniforms.CAMERA_POSITION = [vmi[12], vmi[13], vmi[14]];
    //console.log ("CAMERA AT: " + Float3.str (standardUniforms.CAMERA_POSITION));

    // draw the scene
    scene.traverse (standardUniforms);
};

let buildScene = function () {
    // create a surface of revolution for the prop rings
    let rp = [[0.4375, 0.0500], [0.4250, 0.0500], [0.4375, 0.0000], [0.4250, -0.0500], [0.4375, -0.0500], [0.4500, -0.0167], [0.4500, 0.0167]];
    /*
    let rpIndex = [0, 1, 1, 2, 3, 3, 4, 4, 5, 6, 0];
    let rpSign =  [0, 0, -1, -1, -1, 0, 0, 1, 1, 1, 1];
    */
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
        .addChild(Node.new({
            transform:  Float4x4.chain (Float4x4.scale (0.15), Float4x4.translate ([locX, locY, locZ])),
            state: function (standardUniforms) {
                Program.get("basic").use();
                standardUniforms.MODEL_COLOR =  [1.0, 0.25, 0.125];
                standardUniforms.OUTPUT_ALPHA_PARAMETER = 1.0;
            },
            shape: "sphere2",
            children: false
        }, "target"))
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

    // add some "space" nodes to give a sense of motion, spaced at 4x4, from -12 .. 12
    let spaceLow = -12;
    let spaceHigh = 12;
    let spaceSpacing = 2;
    for (let i = spaceLow; i <= spaceHigh; i+= spaceSpacing) {
        for (let j = 0; j <= spaceHigh; j+= spaceSpacing) {
            for (let k = spaceLow; k <= spaceHigh; k+= spaceSpacing) {
                scene.addChild (Node.new({
                    transform:  Float4x4.chain (Float4x4.scale (0.0125), Float4x4.translate ([i, j, k])),
                    state: function (standardUniforms) {
                        Program.get("basic").use();
                        standardUniforms.OUTPUT_ALPHA_PARAMETER = 0.5;
                        standardUniforms.MODEL_COLOR =  [0.5, 0.5, 0.5];
                    },
                    shape: "sphere2",
                    children: false
                }));
            }
        }
    }


    // create the drone, the transform applies to the first configuration to give the initial
    // flight configuration of the drone
    droneOne = Drone.new ({transform: Float4x4.translate ([0, 1, 0])}, "one").addToScene (scene);
    droneTwo = Drone.new ({transform: Float4x4.translate ([3, 1, -3])}, "two").addToScene (scene);
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
