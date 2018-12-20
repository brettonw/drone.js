"use strict;"

importScripts("https://brettonw.github.io/webgl.js/src/main/webapp/webgl-debug.js");
importScripts(
    "constants.js",
    "particle.js",
    "ground-constraint.js",
    "distance-constraint.js",
    "pid.js",
    "physics-worker.js",
    "drone-worker.js"
);

let drone;

addEventListener("message", function(event) {
    let message = event.data;
    switch (message.command) {
        case "start":
            // get the transform...
            drone = DroneWorker.new (message.parameters);
            break;
        case "update":
            drone.update(message.parameters.deltaTime);
            postMessage({command: "update", parameters: { transform: drone.transform }});
            break;
        case "set-goal":
            drone.setGoal(message.parameters.xyz);
            break;
        case "stop":
            close();
            break;
        default:
            postMessage({command: "error", parameters: { description: "Unknown command (" + message.command + ")" }});
    };
}, false);
