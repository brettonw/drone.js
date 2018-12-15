"use strict;"

let render;
let scene;
let standardUniforms = Object.create (null);

let globalTime = 0;
let lastTime = new Date ().getTime ();

const targetFrameRate = 30;
const targetFrameDeltaTimeMs = 1000.0 * (1.0 / targetFrameRate)* 0.9;

const fpsHistoryCount = 32;
let frameCounter = 0;
let fpsHistory = Array (fpsHistoryCount).fill (0);
let fpsHistoryIndex = 0;
let fpsHistoryAverage = 0;

let drone;

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
        // compute the updated time (in ms)
        let deltaTime = nowTime - lastTime;
        if (deltaTime < targetFrameDeltaTimeMs) {
            // if not enough time has past, skip this frame...
            window.requestAnimationFrame (drawFrame);
            return;
        }

        // update the fps
        fpsHistoryAverage -= fpsHistory[fpsHistoryIndex];
        fpsHistoryAverage += (fpsHistory[fpsHistoryIndex] = deltaTime);
        fpsHistoryIndex = (fpsHistoryIndex + 1) % fpsHistoryCount;
        if (++frameCounter > fpsHistoryCount) {
            let averageDeltaTime = fpsHistoryAverage / fpsHistoryCount;
            let fps = 1000.0 / averageDeltaTime;
            displayFpsSpan.innerHTML = Utility.padNum(fps.toFixed(1), 3) + " fps";

            deltaTime = 1.0 / targetFrameRate;
            drone.update (deltaTime);

            globalTime += deltaTime;
            Thing.updateAll (globalTime);

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
                    newLoc[1] = 2.0 + Math.floor (Math.random () * radius);
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
            drone.setGoal (locX, locY, locZ);
        }

        // draw again as fast as possible
        window.requestAnimationFrame (drawFrame);
    }
    lastTime = nowTime;

    standardUniforms.PROJECTION_MATRIX_PARAMETER = Float4x4.perspective (35, context.viewportWidth / context.viewportHeight, 0.1, 32);
    let cameraDeltaVectorLength = Float3.norm (drone.position);
    let cameraDeltaVector = Float3.add (Float3.scale (drone.position, (1 / cameraDeltaVectorLength)  * (cameraDeltaVectorLength + 5)), [3, 1, 2]);
    standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (cameraDeltaVector, drone.position, [0, 1, 0]);
    //standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt (Float3.add ([0, 2, 8], drone.position), drone.position, [0, 1, 0]);
    standardUniforms.MODEL_MATRIX_PARAMETER = Float4x4.identity ();

    // compute the camera position and set it in the standard uniforms
    let vmi = Float4x4.inverse (standardUniforms.VIEW_MATRIX_PARAMETER);
    standardUniforms.CAMERA_POSITION = [vmi[12], vmi[13], vmi[14]];
    //console.log ("CAMERA AT: " + Float3.str (standardUniforms.CAMERA_POSITION));

    // draw the scene
    scene.traverse (standardUniforms);
};

let buildScene = function () {

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
            standardUniforms.SPECULAR_CONTRIBUTION = 0.05;
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


    drone = Drone.new ({transform: Float4x4.translate ([0, 1, 0])});
    drone.addToScene (scene);
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
    /*
    MouseTracker.new ("render-canvas", OnReady.new (null, function (deltaPosition) {
        if (!animateCheckbox.checked) {
            window.requestAnimationFrame (drawFrame);
        }
        drone.run (0.5 - (10.0 * deltaPosition[1]), 100.0 * deltaPosition[0]);
    }), 0.01);
    */

    // create the render object
    render = Render.new ({
        canvasId: "render-canvas",
        loaders: [
            LoaderPath.new ({ type: Texture, path: "textures/@.png" }).addItems (["grid-16x16", "drone-deck", "drone-props"], { generateMipMap: true }),
        ],
        onReady: OnReady.new (null, buildScene)
    });
};
