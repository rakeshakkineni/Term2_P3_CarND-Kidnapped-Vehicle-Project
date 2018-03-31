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
	//std::vector<Particle> tmp_par;
	//cout<<"Initializing...\n";
	default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);


	num_particles = 30;

	// Initialize particles with default values i.e GPS position before started and 1 for weights.

	for(int cnt =0; cnt<num_particles;cnt++)
	{
		Particle particles_t;
		particles_t.id = cnt;
		particles_t.x = dist_x(gen);
		particles_t.y = dist_y(gen);
		particles_t.theta = dist_theta(gen);
		particles_t.weight = 1.0;

		particles.push_back(particles_t);
		weights.push_back(1.0);
		//cout<<"Init\t"<<"ID\t"<<particles_t.id<<"\tx\t"<<particles_t.x<<"\ty\t"<<particles_t.y<<endl;
	}

	is_initialized =  true;
	//cout<<"Initializing...End\n";

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//cout<<"Predicting...\n";
	default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	/*Run through all the particles and predict their new position*/
	for(int cnt =0; cnt<num_particles;cnt++)
	{
		//cout<< cnt<< "particle updated\n";
		double old_theta= particles[cnt].theta;
		if(fabs(yaw_rate)<0.0001)
		{
			/*If yaw rate is very small use equation x0+a*t*/
			particles[cnt].x += velocity*delta_t*cos(old_theta)+dist_x(gen);
			particles[cnt].y += velocity*delta_t*sin(old_theta)+dist_y(gen);

		}
		else
		{
			particles[cnt].theta += yaw_rate*delta_t;
			particles[cnt].x += ((velocity*(sin(particles[cnt].theta)-sin(old_theta)))/yaw_rate)+dist_x(gen);
			particles[cnt].y += ((velocity*(cos(old_theta)-cos(particles[cnt].theta)))/yaw_rate)+dist_y(gen);

		//cout<<"ID\t"<<particles[cnt].id<<"\tx\t"<<particles[cnt].x<<"\ty\t"<<particles[cnt].y<<endl;
		}
		/*Add Noise to theta*/
		particles[cnt].theta += dist_theta(gen);


	}


	//cout<<"Prediction...End\n";
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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
	//cout<<"Updating Weights...\n";

	/*Normalization Term for probablity calculation*/
	double gauss_norm = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]);

	for(int cnt = 0; cnt < particles.size(); cnt++)
	{
		// Transform the coordinate system
		particles[cnt].weight = 1.0;

		double cos_theta = cos(particles[cnt].theta);
		double sin_theta = sin(particles[cnt].theta);

		/* Look for landmark near to the current particle*/
		Map map_landmarks_temp;
		for(int map_cnt=0;map_cnt<map_landmarks.landmark_list.size();map_cnt++)
		{
			double landmark_dist =sqrt((pow((particles[cnt].x-map_landmarks.landmark_list[map_cnt].x_f),2))+(pow((particles[cnt].y-map_landmarks.landmark_list[map_cnt].y_f),2)));

			if(landmark_dist<sensor_range)
			{
				map_landmarks_temp.landmark_list.push_back(map_landmarks.landmark_list[map_cnt]);
			}
		}


		for(int i = 0; i < observations.size(); i++)
		{
			LandmarkObs obser_temp;
			/* Transform the observations to map coordinates*/
			obser_temp.x = particles[cnt].x + cos_theta*observations[i].x-sin_theta*observations[i].y;

			obser_temp.y = particles[cnt].y + sin_theta*observations[i].x+
										cos_theta*observations[i].y;

			/*Find nearest land mark to observation*/
			double dist = sqrt((pow((obser_temp.x-map_landmarks_temp.landmark_list[0].x_f),2))+
					(pow((obser_temp.y-map_landmarks_temp.landmark_list[0].y_f),2)));
			double dist_min = dist;
			double landmark_id = map_landmarks_temp.landmark_list[0].id_i;

			for(int j = 1; j < map_landmarks_temp.landmark_list.size(); j++)
			{
				dist = sqrt((pow((obser_temp.x-map_landmarks_temp.landmark_list[j].x_f),2))+
						(pow((obser_temp.y-map_landmarks_temp.landmark_list[j].y_f),2)));

				if(dist<dist_min)
				{
					dist_min = dist;
					landmark_id = map_landmarks_temp.landmark_list[j].id_i;

				}
			}

			obser_temp.id = landmark_id;


			/*Weights Calculation*/
			double x_land = obser_temp.x;
			double y_land = obser_temp.y;

			double x_mu = map_landmarks.landmark_list[landmark_id-1].x_f;
			double y_mu = map_landmarks.landmark_list[landmark_id-1].y_f;

			/*Exponent*/
			double exponent = -(((pow((x_land-x_mu),2))/(2*std_landmark[0]*std_landmark[0]))+
								((pow((y_land-y_mu),2))/(2*std_landmark[1]*std_landmark[1])));


			//cout<<"min_dis\t"<<dist_min<<"x\t"<<(x_land-x_mu)<<"y\t"<<(y_land-y_mu)<<endl;
			/*Calculate the probablity*/
			particles[cnt].weight *=gauss_norm*pow(exp(1.0),exponent);

		}


		weights[cnt] = particles[cnt].weight;
		//cout<<"Updating Weights...end\n";

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//cout<<"Resamping...\n";
	std::vector<Particle> resampled_particles(num_particles);
	std::discrete_distribution<int> ddist(weights.begin(),weights.end());
	std::default_random_engine gen;

	/*re-populate the particles with the once with highest weight*/
	for(int cnt =0; cnt<num_particles;cnt++)
	{
		resampled_particles[cnt]=particles[ddist(gen)];
	}

	particles = resampled_particles;

	//cout<<"Resamping...end\n";
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
	//cout<<"Associating...\n";
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    //cout<<"Associating...End\n";
}

string ParticleFilter::getAssociations(Particle best)
{
	//cout<<"GetAssociations...\n";
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    //cout<<"GetAssociations...End\n";
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	//cout<<"GetSenseX...\n";
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    //cout<<"GetSenseX...End\n";
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	//cout<<"GetSenseY...\n";
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    //cout<<"GetSenseY...End\n";
    return s;
}
