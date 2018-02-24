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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;

    static default_random_engine gen;

    // Create a normal Gaussian distribution for x, y and theta
    normal_distribution <double> dist_x(x, std[0]);
    normal_distribution <double> dist_y(y, std[1]);
    normal_distribution <double> dist_theta(theta, std[2]);

    // Particle initialization
    particles.clear();
    for(int i=0; i<num_particles; i++){
        Particle p = {};
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        // Initialize weight to each particle
        p.weight = 1;
        // Add to list of particles
        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    normal_distribution <double> dist_x(0, std_pos[0]);
    normal_distribution <double> dist_y(0, std_pos[1]);
    normal_distribution <double> dist_theta(0, std_pos[2]);

    for(Particle& p: particles){
        // Check if yaw_rate is equal to zero
        if(fabs(yaw_rate) > 0.001){
            p.x += velocity/yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += velocity/yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        } else{
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
            p.theta = p.theta;
        }
        // Add random Gaussian noise
        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    for(LandmarkObs& obs : observations){
        double min = numeric_limits<double>::max();
        for(LandmarkObs& pred : predicted){
            // Calculate the distance of predicted measurement and observation measurement
            double d = dist(obs.x, obs.y, pred.x, pred.y);
            // Find the nearest landmark and assign id;
            if(d < min){
                min = d;
                obs.id = pred.id;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> observations, const Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    vector<LandmarkObs> predictions;
    for(Particle& p : particles){
        // Check landmarks in the sensor range of particles
        for(Map::single_landmark_s landmark : map_landmarks.landmark_list){
            // Calculate the distance between landmarks and particles
            double  d = dist(landmark.x_f, landmark.y_f, p.x, p.y);
            // Check if distance in sensor range
            if(d <= sensor_range){
                LandmarkObs obs = {};
                obs.id = landmark.id_i;
                obs.x = landmark.x_f;
                obs.y = landmark.y_f;
                predictions.push_back(obs);
            }
        }

        // Transform observation from VEHICLE coordinates to MAP coordinates
        vector<LandmarkObs> transformed_observations;
        for(LandmarkObs obs : observations){
            LandmarkObs new_obs = {};
            // Homogenous transform
            new_obs.id = obs.id;
            new_obs.x = p.x + cos(p.theta) * obs.x - sin(p.theta) * obs.y;
            new_obs.y = p.y + sin(p.theta) * obs.x + cos(p.theta) * obs.y;
            transformed_observations.push_back(new_obs);
        }

        // Association of observation to its nearest landmark
        dataAssociation(predictions, transformed_observations);
        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;

        // Update particle weight according to observation in Map from nearest landmarks
        double weight = 1;
        double std_x_2 = std_landmark[0] * std_landmark[0];
        double std_y_2 = std_landmark[1] * std_landmark[1];
        double std_x_y = std_landmark[0] * std_landmark[1];

        // Calculate weights
        for(LandmarkObs& new_obs : transformed_observations){
            // Get associated landmark
            Map::single_landmark_s landmark = map_landmarks.landmark_list.at(new_obs.id - 1);
            // Calculate the Multivariate Gaussian probability
            double e_x = new_obs.x - landmark.x_f;
            double e_y = new_obs.y - landmark.y_f;
            double exponent = e_x * e_x / (2.0 * std_x_2) + e_y * e_y / (2.0 * std_y_2);
            double prob = exp(-exponent) / (2.0 * M_PI * std_x_y);
            // Multiply all the calculated measurement probabilities
            weight *= prob;
            // Record particle association
            associations.push_back(new_obs.id);
            sense_x.push_back(new_obs.x);
            sense_y.push_back(new_obs.y);
        }

        // Update particle final weight
        p.weight = weight;
        // Record in weight vector
        weights.push_back(weight);
        // Update particle associations
        SetAssociations(p, associations, sense_x, sense_y);
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> d(weights.begin(), weights.end());

    // New vector of the resample particles
    vector<Particle> new_particles;

    for(Particle& p :particles){
        const int id = d(gen);
        Particle new_p = particles[id];
        new_particles.push_back(new_p);
    }

    particles.clear();
    weights.clear();
    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int> associations,
                                     const std::vector<double> sense_x, const std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    // Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
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
