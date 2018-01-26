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
#include <chrono> // Required for random seed

#include "particle_filter.h"

using namespace std;

#define EPSILON 0.0001 // For checking if a number is zero
#define NUM_PARTICLES 100

// Global Random Number Generator
default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());

bool isZero(double value) {
  return fabs(value) < EPSILON;
}

// Returns the distance between two coordinate points
double distance(double x0, double y0, double x1, double y1) {
  double x_diff = x0 - x1;
  double y_diff = y0 - y1;

  return sqrt(x_diff*x_diff + y_diff*y_diff);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  cout << "Initializing particle filter" << endl;

  // Get random number generator
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Generate random particles
  num_particles = NUM_PARTICLES;
  particles.resize(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    Particle &p = particles[i];

    p.id = i;
    p.weight = 1.0;

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }

  // Set weights to 1
  weights.resize(num_particles, 1.0);

  // Set is_initialized flag
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Check if initialized
  if(!is_initialized) {
    cout << "ERROR: Cannot run prediction step if the particle filter isn't initialized." << endl;
    return;
  }

  // cout << "Predicting next particle states" << endl;

  // Get Gaussian distribution generators
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  // For each particle, predict its next state
  for (int i = 0; i < num_particles; ++i) {
    Particle &p = particles[i];

    // Add movement terms based on whether yaw_rate is zero or nonzero
    if (isZero(yaw_rate)) {
      p.x += velocity * cos(p.theta) * delta_t;
      p.y += velocity * sin(p.theta) * delta_t;
    } else {
      p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;
    }

    // Store updated states with gaussians into the particle properties
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);

    // // Normalize p.theta
    // // NOTE: Apparently this causes issues with the algorithm when making sharp turns
    // while (p.theta >  M_PI) p.theta -= 2.0 * M_PI;
    // while (p.theta < -M_PI) p.theta += 2.0 * M_PI;
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

  // Check if initialized
  if(!is_initialized) {
    cout << "ERROR: Cannot run prediction step if the particle filter isn't initialized." << endl;
    return;
  }

  // cout << "Associating observed landmarks with expected landmarks" << endl;

  int num_observations  = observations.size();

  // For each observation, find which landmark is closest
  for (int i = 0; i < num_observations; ++i) {
    LandmarkObs &obs = observations[i];

    double min_distance;
    int closest_landmark_id = -1;

    // Update closest_landmark_id if lmk is closer
    for (auto lmk : predicted) {
      double dist = distance(obs.x, obs.y, lmk.x, lmk.y);
      if (min_distance > dist || closest_landmark_id == -1) {
        min_distance = dist;
        closest_landmark_id = lmk.id;
      }
    }

    // Associate closest landmark to observed landmark
    obs.id = closest_landmark_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

  // Check if initialized
  if(!is_initialized) {
    cout << "ERROR: Cannot run prediction step if the particle filter isn't initialized." << endl;
    return;
  }

  // cout << "Updating particle weights" << endl;

  // For each particle, update weights by observation error distances
  int num_landmarks = map_landmarks.landmark_list.size();
  for (int i = 0; i < num_particles; ++i) {
    Particle &p = particles[i];

    // Get viewable landmarks within sensor range from the particle's position
    vector<LandmarkObs> viewable_landmarks;
    for (auto l : map_landmarks.landmark_list) {
      if (distance(p.x, p.y, l.x_f, l.y_f) < sensor_range) {
        LandmarkObs new_landmark;
        new_landmark.id = l.id_i;
        new_landmark.x = l.x_f;
        new_landmark.y = l.y_f;

        viewable_landmarks.push_back(new_landmark);
      }
    }

    // Map observations from vehicle space to map space
    vector<LandmarkObs> mapped_observations;
    for (auto o : observations) {
      LandmarkObs o_m;
      o_m.x = o.x * cos(p.theta) - o.y * sin(p.theta) + p.x;
      o_m.y = o.x * sin(p.theta) + o.y * cos(p.theta) + p.y;

      mapped_observations.push_back(o_m);
    }

    // Associate observations to expected landmarks
    dataAssociation(viewable_landmarks, mapped_observations);

    // Update particle weights using Multivariate Gaussian Probability
    double new_weight = 1.0;
    for (auto m : mapped_observations) {
      if (m.id > num_landmarks) cout << "m has an invalid id of " << m.id << endl;

      Map::single_landmark_s l = map_landmarks.landmark_list[m.id-1];

      // Multivariate Gaussian Calculation
      double x_diff = m.x - l.x_f;
      double y_diff = m.y - l.y_f;
      double sx = std_landmark[0];
      double sy = std_landmark[1];

      double exp_term = -1 * (x_diff*x_diff/(2*sx*sx) + y_diff*y_diff/(2*sy*sy));
      double gauss_norm = 2 * M_PI * sx * sy;

      // double prob = exp(exp_term) / gauss_norm;

      new_weight *= exp(exp_term) / gauss_norm;
    }

    p.weight = new_weight;
    weights[i] = new_weight;

  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Check if initialized
  if(!is_initialized) {
    cout << "ERROR: Cannot run prediction step if the particle filter isn't initialized." << endl;
    return;
  }

  // cout << "Resampling particles" << endl;

  // Create discrete generator
  discrete_distribution<> d(weights.begin(), weights.end());

  // Resample particles using the discrete random number generator
  vector<Particle> p2(num_particles);

  for (int i = 0; i < num_particles; ++i) {
    // Resample with random particle and renumber resampled particle's id
    int chosen_particle_id = d(gen);
    p2[i] = particles[chosen_particle_id];
    p2[i].id = i;
  }

  particles = p2;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
