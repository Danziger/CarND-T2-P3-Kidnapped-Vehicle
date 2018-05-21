#include "ParticleFilter.h"

#include "json.hpp"

#include <uWS/uWS.h>
#include <math.h>
#include <iostream>


#define MAP_DATA_SRC "../input/data/map_data.txt"


// For convenience:
using json = nlohmann::json;
using namespace std;


// HELPER FUNCTIONS:

/*
* Checks if the SocketIO event has JSON data.
* If there is data the JSON object in string format will be returned,
* else the empty string "" will be returned.
*/
string hasData(const string s) {
    const auto found_null = s.find("null");
    const auto b1 = s.find_first_of("[");
    const auto b2 = s.rfind("}]");

    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";
}


// MAIN:

int main() {
    // Set up parameters here:

    double delta_t = 0.1; // Time elapsed between measurements [sec]
    double sensor_range = 50; // Sensor range [m]

    double sigma_pos[3] = { 0.3, 0.3, 0.01 }; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark[2] = { 0.3, 0.3 }; // Landmark measurement uncertainty [x [m], y [m]]

    // Read map data:

    Map map;

    if (!read_map_data(MAP_DATA_SRC, map)) {
        cout << "Error: Could not open map file at " << MAP_DATA_SRC << endl;

        return -1;
    }

    // Create particle filter
    ParticleFilter pf;

    // MESSAGE PROCESSING:

    uWS::Hub h; // Initialize WebSocket.

    h.onMessage([
        &pf,
        &map,
        &delta_t,
        &sensor_range,
        &sigma_pos,
        &sigma_landmark
    ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event
        const string sdata = string(data).substr(0, length);

        if (sdata.size() <= 2 || sdata[0] != '4' || sdata[1] != '2') {
            return;
        }

        const string s = hasData(sdata);

        if (s == "") {
            // Manual driving:

            string msg = "42[\"manual\",{}]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            return;
        }

        const auto j = json::parse(s);
        const string event = j[0].get<string>();

        if (event != "telemetry") {
            return;
        }

        // j[1] is the data JSON object:

        if (!pf.initialized()) {

            // Sense noisy position data from the simulator
            double sense_x = stod(j[1]["sense_x"].get<string>());
            double sense_y = stod(j[1]["sense_y"].get<string>());
            double sense_theta = stod(j[1]["sense_theta"].get<string>());

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
        } else {
            // Predict the vehicle's next state from previous (noiseless control) data.
            double previous_velocity = stod(j[1]["previous_velocity"].get<string>());
            double previous_yawrate = stod(j[1]["previous_yawrate"].get<string>());

            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
        }

        // receive noisy observation data from the simulator
        // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
        vector<LandmarkObs> noisy_observations;
        string sense_observations_x = j[1]["sense_observations_x"];
        string sense_observations_y = j[1]["sense_observations_y"];

        vector<float> x_sense;
        istringstream iss_x(sense_observations_x);

        copy(
            istream_iterator<float>(iss_x),
            istream_iterator<float>(),
            back_inserter(x_sense)
        );

        vector<float> y_sense;
        istringstream iss_y(sense_observations_y);

        copy(
            istream_iterator<float>(iss_y),
            istream_iterator<float>(),
            back_inserter(y_sense)
        );

        unsigned int num_observations = x_sense.size();

        for (unsigned int i = 0; i < num_observations; ++i) {
            LandmarkObs obs;
            obs.x = x_sense[i];
            obs.y = y_sense[i];

            noisy_observations.push_back(obs);
        }

        // Update the weights and resample:
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        // Calculate and output the average weighted error of the particle filter over all time steps so far:
        vector<Particle> particles = pf.particles_;

        unsigned int num_particles = particles.size();

        double highest_weight = -1.0;
        Particle best_particle;
        double weight_sum = 0.0;

        for (unsigned int i = 0; i < num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
                highest_weight = particles[i].weight;
                best_particle = particles[i];
            }

            weight_sum += particles[i].weight;
        }

        // cout << "highest w " << highest_weight << endl;
        // cout << "average w " << weight_sum/num_particles << endl;

        json msgJson;
        msgJson["best_particle_x"] = best_particle.x;
        msgJson["best_particle_y"] = best_particle.y;
        msgJson["best_particle_theta"] = best_particle.theta;

        // Optional message data used for debugging particle's sensing and associations
        msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
        msgJson["best_particle_sense_x"] = pf.getSenseX(best_particle);
        msgJson["best_particle_sense_y"] = pf.getSenseY(best_particle);

        auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";

        // Send it:
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);                    
    });

    // ON HTTP REQUEST:
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(

    h.onHttpRequest([](
        uWS::HttpResponse *res,
        uWS::HttpRequest req,
        char *data,
        size_t,
        size_t
    ) {
        const string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    // ON CONNECTION:

    h.onConnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        uWS::HttpRequest req
    ) {
        cout
            << endl
            << " Connected!" << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl
            << endl;
    });

    // ON DISCONNECTION:

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        int code,
        char *message,
        size_t length
     ) {
        ws.close();

        cout << "Disconnected!" << endl << endl << endl;
    });

    // START LISTENING:

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl;

    } else {
        cerr << endl << "Failed to listen on port" << port << "!" << endl << endl;

        return -1;
    }

    h.run();
}
