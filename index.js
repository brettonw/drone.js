"use strict;"

let render;
let scene;
let standardUniforms = Object.create (null);

let globalTime = 0;
let lastTime = new Date ().getTime ();

const fpsHistoryCount = 64;
let frameCounter = 0;
let fpsHistory = Array (fpsHistoryCount).fill (0);
let fpsHistoryIndex = 0;
let fpsHistoryAverage = 0;

let drone;

let animateCheckbox;
let displayFpsSpan;

let drawFrame = function () {
    if (document.hidden) {
        animateCheckbox.checked = false;
        return;
    }
    let nowTime = new Date ().getTime ();
    if (animateCheckbox.checked) {

        // compute the updated time
        let deltaTime = nowTime - lastTime;

        // update the fps
        fpsHistoryAverage -= fpsHistory[fpsHistoryIndex];
        fpsHistoryAverage += (fpsHistory[fpsHistoryIndex] = deltaTime);
        fpsHistoryIndex = (fpsHistoryIndex + 1) % fpsHistoryCount;
        let averageDeltaTime = fpsHistoryAverage / fpsHistoryCount;
        let fps = 1000.0 / averageDeltaTime;
        displayFpsSpan.innerHTML = Utility.padNum(fps.toFixed(1), 3) + " fps";
        if (++frameCounter > fpsHistoryCount) {
            drone.update (averageDeltaTime / 1000);

            globalTime += deltaTime;
            Thing.updateAll (globalTime);
        }

        // draw again as fast as possible
        window.requestAnimationFrame (drawFrame);
    }
    lastTime = nowTime;

    standardUniforms.PROJECTION_MATRIX_PARAMETER = Float4x4.perspective (30, context.viewportWidth / context.viewportHeight, 0.1, 1000);
    standardUniforms.VIEW_MATRIX_PARAMETER = Float4x4.lookFromAt ([-2, 5, 10], [0, 1, 0], [0, 1, 0]);
    standardUniforms.MODEL_MATRIX_PARAMETER = Float4x4.identity ();

    // compute the camera position and set it in the standard uniforms
    let vmi = Float4x4.inverse (standardUniforms.VIEW_MATRIX_PARAMETER);
    standardUniforms.CAMERA_POSITION = [vmi[12], vmi[13], vmi[14]];
    //console.log ("CAMERA AT: " + Float3.str (standardUniforms.CAMERA_POSITION));

    // draw the scene
    scene.traverse (standardUniforms);
};

let buildScene = function () {

    scene = Node.new ({
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
        .addChild (Node.new ({
            transform: Float4x4.scale (0.05),
            state: function (standardUniforms) {
                Program.get ("basic").use ();
                standardUniforms.MODEL_COLOR = [1.0, 1.0, 0.25];
            },
            shape: "sphere2",
            children: false
        }))
        .addChild (Node.new ({
            transform: Float4x4.chain (Float4x4.scale (4), Float4x4.rotateX (Math.PI / -2)),
            state: function (standardUniforms) {
                Program.get ("basic-texture").use ();
                standardUniforms.TEXTURE_SAMPLER = "grid-16x16";
                standardUniforms.MODEL_COLOR = [1.0, 1.0, 1.0];
            },
            shape: "square",
            children: false
        }))
    ;

    drone = Drone.new ({transformation: Float4x4.translate ([0, 0.5, 0])});
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
    MouseTracker.new ("render-canvas", OnReady.new (null, function (deltaPosition) {
        if (!animateCheckbox.checked) {
            window.requestAnimationFrame (drawFrame);
        }
    }), 0.01);

    // create the render object
    render = Render.new ({
        canvasId: "render-canvas",
        loaders: [
            LoaderPath.new ({ type: Texture, path: "textures/@.png" }).addItems ("grid-16x16", { generateMipMap: true }),
        ],
        onReady: OnReady.new (null, buildScene)
    });
};
