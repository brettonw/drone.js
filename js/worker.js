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

let object;

addEventListener("message", function(event) {
    let message = event.data;
    switch (message.command) {
        case "start":
            // get the object type
            let objectType = eval (message.parameters.workerObjectType);
            object = objectType.new (message.parameters);
            break;
        case "update":
            object.update(message.parameters.deltaTime);
            postMessage({command: "update", parameters: { transform: object.transform }});
            break;
        case "set-goal":
            object.setGoal(message.parameters.xyz);
            break;
        case "stop":
            close();
            break;
        default:
            postMessage({command: "error", parameters: { description: "Unknown command (" + message.command + ")" }});
    }
}, false);
