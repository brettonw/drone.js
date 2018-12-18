addEventListener("message", function(event) {
    let message = event.data;
    switch (message.command) {
        case "start":
            postMessage("WORKER STARTED");
            break;
        case "update":
            postMessage("GOT IT (" + message.command + ")");
            break;
        case "stop":
            postMessage("WORKER STARTED");
            close();
            break;
        default:
            postMessage("Unknown command: " + message.command);
    };
}, false);