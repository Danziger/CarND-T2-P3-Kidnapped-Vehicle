/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <limits>

#include "ParticleFilter.h"


#define EPS 0.0001
#define NEPS -EPS
#define KEEP_IN_RANGE(n) (n < NEPS ? NEPS : (n < EPS ? EPS : n))
#define INITIAL_WEIGHT  1.0


using namespace std;


void ParticleFilter::init(
    const double x,
    const double y,
    const double theta,
    const double std[]
) {
    // Set the number of particles:
    num_particles_ = 1000;

    // Initialize all particles to first position (based on estimates of x, y, theta
    // and their uncertainties from GPS and accounting for random Gaussian noise):
    
    default_random_engine gen;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Initialize all the particles:

    for (int i = 0; i < num_particles_; ++i) {
        Particle p;

        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen); // Normalized.
        p.weight = INITIAL_WEIGHT;

        particles_.push_back(p);
    }

    // ...and all weights:
    weights_ = vector<double> (num_particles_, INITIAL_WEIGHT);

    // We are done here:
    is_initialized_ = true;
}


void ParticleFilter::prediction(
    const double dt,
    const double std_pos[],
    const double velocity,
    const double yaw_rate
) {
    // We also need to account for the uncertainty in the control input here:
    default_random_engine gen;

    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // Calculate fixed terms:
    const double v_over_yaw_rate = velocity / yaw_rate;
    const double yaw_rate_times_dt = yaw_rate * dt;
    const double vdt = velocity * dt;

    // Calculate prediction, avoiding division by 0:

    for (int i = 0; i < num_particles_; ++i) {
        const double theta = particles_[i].theta;

        // Update x, y and theta while accounting for noise too:

        if (fabs(yaw_rate) < EPS) {
            particles_[i].x += vdt * cos(theta) + dist_x(gen);
            particles_[i].y += vdt * sin(theta) + dist_y(gen);
            particles_[i].theta += dist_theta(gen);
        } else {
            const double theta_next = theta + yaw_rate_times_dt; // Not normalized yet.

            particles_[i].x += v_over_yaw_rate * (sin(theta_next) - sin(theta)) + dist_x(gen);
            particles_[i].y += v_over_yaw_rate * (cos(theta) - cos(theta_next)) + dist_y(gen);
            particles_[i].theta += yaw_rate_times_dt + dist_theta(gen);
        }
    }
}


void ParticleFilter::updateWeights(
    const double sensor_range,
    const double std_landmark[], 
	vector<LandmarkObs> &observations,
    const Map &map_landmarks
) {
    // We do this once here instead of doing a sqrt on the inner-most loop:
    const double sensor_range_2 = pow(sensor_range, 2);
    const double std_x = std_landmark[0];
    const double std_y = std_landmark[1];
    const double dx = 2.0 * pow(std_x, 2);
    const double dy = 2.0 * pow(std_y, 2);
    const double d = 2.0 * M_PI * std_x * std_y;
    const int total_observations = observations.size();

    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < num_particles_; ++i) {
        Particle particle = particles_[i];

        // TODO: Improve this and/or make it optional:
        particles_[i].associations = vector<int>();
        particles_[i].sense_x = vector<double>();
        particles_[i].sense_y = vector<double>();

        // Calculate landmarks in range:

        vector<Map::single_landmark_s> landmarks_in_range;

        for (const auto landmark : map_landmarks.landmark_list) {
            const double distance_2 = pow(landmark.x_f - particle.x, 2) + pow(landmark.y_f - particle.y, 2);

            if (distance_2 <= sensor_range_2) {
                landmarks_in_range.push_back(landmark);
            }
        }

        // Calculate fixed terms that depend only on particle data:
        const double theta_cos = cos(particle.theta);
        const double theta_sin = sin(particle.theta);

        // Init particle weight (mult-variate Gaussian distribution):
        // See https://en.wikipedia.org/wiki/Multivariate_normal_distribution).
        double weight = 1.0;

        for (int j = 0; j < total_observations; ++j) {
            // Transform observation from vehicle's coordinate system to map's coordinate system:
            const LandmarkObs observation = observations[j];

            const double obs_x_rel = observation.x;
            const double obs_y_rel = observation.y;

            // Equations from figure 3.33 at http://planning.cs.uiuc.edu/node99.html:
            const double obs_x_glob = obs_x_rel * theta_cos - obs_y_rel * theta_sin + particle.x;
            const double obs_y_glob = obs_x_rel * theta_sin + obs_y_rel * theta_cos + particle.y;

            // Calculate distances from landmarks to observations and keep track of the closest one:
            // Note dataAssociation is not used and has been removed, as it would involve additional
            // loops and calculations.

            double closest_landmark_x = 0;
            double closest_landmark_y = 0;
            double closest_landmark_distance_2 = numeric_limits<double>::max();
            int closest_landmark_id = -1;

            for (const auto landmark : landmarks_in_range) {
                const double landmark_x = landmark.x_f;
                const double landmark_y = landmark.y_f;
                const double distance_2 = pow(landmark_x - obs_x_glob, 2) + pow(landmark_y - obs_y_glob, 2);

                if (distance_2 < closest_landmark_distance_2) {
                    // Update the smallest distance found and the ID of the closest landmark:3
                    closest_landmark_x = landmark_x;
                    closest_landmark_y = landmark_y;
                    closest_landmark_distance_2 = distance_2;
                    closest_landmark_id = landmark.id_i;
                }
            }

            // Associate an observation to the closest landmark:
            observations[j].id = closest_landmark_id;

            // Associate an observation and its position in map's coordinate system to a particle (for debugging):

            // TODO: Improve this and/or make it optional
            particles_[i].associations.push_back(closest_landmark_id);
            particles_[i].sense_x.push_back(obs_x_glob);
            particles_[i].sense_y.push_back(obs_y_glob);

            // Update weight:

            const double term_x = pow(obs_x_glob - closest_landmark_x, 2) / dx;
            const double term_y = pow(obs_y_glob - closest_landmark_y, 2) / dy;

            weight *= exp(- term_x - term_y);
        }

        weight /= d;
        weight = KEEP_IN_RANGE(weight);

        particles_[i].weight = weight;
        weights_[i] = weight;
    }
}

void ParticleFilter::resample() {

    // TODO: Try resampling wheel method.

    default_random_engine gen;

    // Resample particles with replacement with probability proportional to their weight. 
    discrete_distribution<int> dist_index(weights_.begin(), weights_.end());

    vector<Particle> particles(num_particles_);

    for (int i = 0; i < num_particles_; ++i) {
        const Particle chosen_one = particles_[dist_index(gen)];

        // The same particle could be chosen more than once, so we can't just push it as the
        // ids should not be repeated.

        particles[i] = Particle {
            i,
            chosen_one.x,
            chosen_one.y,
            chosen_one.theta,
            chosen_one.weight,
            // TODO: Improve this: associations, sense_x and sense_y don't need to be on every single Particle.
            chosen_one.associations,
            chosen_one.sense_x,
            chosen_one.sense_y,
        };
    }

    particles_ = particles;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
