"use strict;"

importScripts("https://brettonw.github.io/webgl.js/src/main/webapp/webgl-debug.js");
importScripts(
    "constants.js",
    "particle.js",
    "ground-constraint.js",
    "distance-constraint.js",
    "pid.js",
    "local-worker.js",
    "physics-worker.js",
    "drone-worker.js"
);

let localWorker;

addEventListener("message", function(event) {
    let message = event.data;
    switch (message.command) {
        case "start":
            localWorker = LocalWorker.new (message.parameters);
            break;
        case "update":
            localWorker.postMessage(message);
            postMessage({command: "update", parameters: { transform: localWorker.object.transform }});
            break;
        case "stop":
            close();
            break;
        default:
            localWorker.postMessage (message);
            break;
    }
}, false);
