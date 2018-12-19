"use strict;"

let WorldState = function () {
    let _ = Object.create (Thing);

    let TIME = _.TIME = "time";
    let TEMPERATURE = _.TEMPERATURE = "temperature";
    let WIND = _.WIND = "wind";
    let SOLAR_DIRECTION = _.SOLAR_DIRECTION = "solar-direction";

    _.construct = function (parameters) {
        this.time = Utility.defaultValue (parameters.time, new Date ().getTime ());
        this.wind = Utility.defaultValue (parameters.wind, [0, 0, 0]);

        // the average daily temperature on earth, degrees Celsius
        this.temperature = 15;
    };

    _.update = function (deltaTime) {
        // update the time, which is in seconds
        this.time += (deltaTime * 1000.0);

        // generate a new perturbation on the wind direction
        this.wind = Float3.add (this.wind, Float3.scale ([(Math.random () * 2.0) - 1.0, 0.0, (Math.random () * 2.0) - 1.0], 0.1));

        // compute the ECS solar position so we can query it based on earth location
        // XXX TODO

    };

    _.WGS84 = function (location) {
        // Baltimore Washington Monument is the default location
        location = Utility.defaultValue (location, { longitude: -76.615586, latitude: 39.297275, altitude: 0 });
        // XXX TODO
        return [0, 0, 0];
    };

    _.query = function (name, location) {
        // location is assumed to be a structure (longitude degrees, latitude degrees, altitude
        // meters above ellipsoid). we convert that to an (x, y, z) in meters assuming the x-axis is longitude, the
        // z-axis is -latitude, and the y-axis is "up"
        let position = this.WGS84 (location);
        switch (name) {
            // global time in milliseconds
            case TIME: return this.time;

            // the average daily temperature on earth
            case TEMPERATURE: return this.temperature;

            // return the perturbed wind direction (assuming we've been updating regularly)
            case WIND: return this.wind;

            // compute the solar direction vector for the given location
            case SOLAR_DIRECTION:
                // XXX TODO
                Float3.normalize ([1.55, 1.75, -1.45]);
                break;

            default: console.log ("WorldState.query: Unknown name (" + name + ")"); break;
        }
    };

    _.set = function (name, value) {
        let position = this.WGS84 (location);
        switch (name) {
            // global time in milliseconds
            case TIME: this.time = value; break;

            // the average daily temperature on earth
            case TEMPERATURE: this.temperature = value; break;

            // return the perturbed wind direction (assuming we've been updating regularly)
            case WIND: this.wind = value; break;

            default: console.log ("WorldState.set: Unknown name (" + name + ")"); break;
        }
    };

    return _;
} ();

// deltaTime is a global value we set to control simulation rates
let worldState;
let deltaTime = 1.0 / targetFrameRate;

