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
	
	num_particles = 10;
	default_random_engine gen;
	//particles = new vector<Particle>[num_particles];
	
	// These lines create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	for (int i = 0; i < num_particles; i++) {
		double sample_x, sample_y, sample_theta;
	
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);
		
		// Print your samples to the terminal.
		//cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
		Particle particle;
		particle.id = i;
		particle.x = sample_x;
		particle.y = sample_y;
		particle.theta = sample_theta;
		// Set all weights to 1
		particle.weight = 1;
		weights.push_back(1);
		
		particles.push_back(particle);
		
	}
	
	is_initialized = true; // Finally initialize the particle filter
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// Don't forget noise
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	
	double delta_theta = yaw_rate * delta_t;
	
	// 14.7 but with zero yaw_rate factored in - avoid divide by zero
	for (int i = 0; i < num_particles; i++) {
		Particle p = particles[i];
		
		if (yaw_rate == 0)
		{
			p.x = p.x + velocity * delta_t * cos(p.theta) + dist_x(gen);
			p.y = p.y + velocity * delta_t * sin(p.theta) + dist_y(gen);
			p.theta = p.theta + dist_theta(gen);
		}
		else
		{	double vel_yaw = velocity / yaw_rate;
			p.x = p.x + vel_yaw * (sin(p.theta + delta_theta) - sin(p.theta)) + dist_x(gen);
			p.y = p.y + vel_yaw * (cos(p.theta) - cos(p.theta + delta_theta)) + dist_y(gen);
			p.theta = p.theta + delta_theta + dist_theta(gen);
		}
		particles[i] = p;
	}
	
	// 
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (auto& observation : observations) {
		
		double lowest_distance = 999999.;
		// Iterate landmarks, find closest one for observation
		for(auto landmark : predicted) {
			double distance = dist(landmark.x,landmark.y, observation.x, observation.y);
			if (distance < lowest_distance) {
				lowest_distance = distance;
				observation.id = landmark.id;
			}
		}	
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
	// x * cos(theta) - y * sin(theta) + x(t)
	// x * sin(theta) + y * cos(theta) + y(t)
	// in a matrix
	// cos(theta) -sin(theta) x(t)
	// sin(theta)  cos(theta) y(t)
	// 0           0          1
	// multiplying 
	// x
	// y
	// 1
	// Gives the first matrix with an extra element of 1.
	weights.clear();
	
	
	for (auto particle: particles) {
		
		particle.associations.clear();
		particle.sense_x.clear();
		particle.sense_y.clear();
		
		vector<LandmarkObs> mapped_observations;
		for (auto& observation : observations) {
		//	observation.x = new value;
		// Convert to particle coordinates
			LandmarkObs obsv;
			obsv.x = observation.x * cos(particle.theta) - observation.y * sin(particle.theta) + particle.x;
			obsv.y = observation.x * sin(particle.theta) + observation.y * cos(particle.theta) + particle.y;
			
			obsv.id = 0; // Start out with no observed landmark
			double lowest_distance = sensor_range;
			// Iterate landmarks, find closest one for observation
			for(auto landmark : map_landmarks.landmark_list) {
				double distance = dist(landmark.x_f,landmark.y_f, obsv.x, obsv.y);
				if (distance < lowest_distance) {
					lowest_distance = distance;
					obsv.id = landmark.id_i;
				}
			}
			
			mapped_observations.push_back(obsv);
			
			// At this point our observation should know which landmark it is associated with.
			// Calculate the weight.
			 
			
		}
		// At this point p_observations should be a list of landmark ids for where we think we are 
		// We can pair those up with the landmarks and work out the new weights
		particle.weight = 1.0;
		for(int i = 0; i < mapped_observations.size(); i++){
			int association = mapped_observations[i].id;
			double mapped_x = mapped_observations[i].x;
			double mapped_y = mapped_observations[i].y;
			
			if (association != 0)
			{
				
				double mu_x = map_landmarks.landmark_list[association].x_f;
				double mu_y = map_landmarks.landmark_list[association].y_f;
				double sig_x = std_landmark[0];
				double sig_y = std_landmark[1];
				double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));
				double exponent = (pow((mapped_x - mu_x), 2.0) / (2 * sig_x * sig_x)) + (pow((mapped_y - mu_y), 2.0) / (2 * sig_y * sig_y));
				long double new_weight = gauss_norm * exp(-exponent);
				//long double multiplier = 1/(2 * M_PI * std_landmark[0]*std_landmark[1])*exp(-(pow(meas_x-0)));
				if (new_weight > 0) {
					particle.weight *= new_weight;
				}
					
			}
			particle.associations.push_back(association+1);
			particle.sense_x.push_back(mapped_x);
			particle.sense_y.push_back(mapped_y);
		}
		//p.x = p.x + velocity_dt * (sin(p.theta + delta_theta) - sin(p.theta)) + dist_x(gen);
		//p.y = p.y + velocity_dt * (cos(p.theta) - cos(p.theta + delta_theta)) + dist_y(gen);
		//p.theta = p.theta + delta_theta + dist_theta(gen);
		
		weights.push_back(particle.weight);
		// 
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	
	vector<Particle> resample_particles;
	
	for (int i = 0; i < num_particles; i++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
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
