/**
 * Localization Challenge
 * 
 * main.cpp 
 *
 * Initiates program main loop, contains function templates for 
 * localization procedures to be implemented.
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <time.h>
#include <algorithm>
#include "RobotDefinitions.h"
#include "LocalizationController.h"
#include "main.h"


/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
    // the position would be the particle that has maximum weight
    std::vector<double>::iterator it = std::max_element(g_weights.begin(),g_weights.end()); 
    size_t ind = std::distance(g_weights.begin(), it);
    
    estimatePosn.x = g_particles[ind].x;
    estimatePosn.y = g_particles[ind].y;
    estimatePosn.theta = g_particles[ind].theta;

    // calculate the number of efficient particles
    double sum=0, sq_sum = 0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
        sum += g_weights[i];
        sq_sum += g_weights[i]*g_weights[i];
    }
    double n_eff = sum * sum / sq_sum;
    //if number of efficient particles are less than an threshold, we need to resample the particles
    if (n_eff < 0.4 * NUM_PARTICLES) {
        g_particles = resample(g_particles, g_weights);
        std::cout << "resample!!" << std::endl;
    }
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // implement the Probablistic Motion Models
    // update the state for each particle according to the odometry
    // data and add noise accordingly

    double dtrans = sqrt(delta.x * delta.x + delta.y * delta.y);
    double drot = delta.theta;
    // get the noise rotational and translational variance
    double rot_var =  (g_robotParams.odom_noise_rotation_from_rotation * fabs(drot)
                    +g_robotParams.odom_noise_rotation_from_translation * fabs(dtrans));
    double tran_var = (g_robotParams.odom_noise_translation_from_rotation * fabs(drot)
                    +g_robotParams.odom_noise_translation_from_translation * fabs(dtrans));
    
    for (int i = 0; i < NUM_PARTICLES; i++) {
        // for each particle, add noice to its odometry;
        double dx_r = generateGaussianNoise(delta.x, tran_var);
        double dy_r = generateGaussianNoise(delta.y, tran_var);
        double dtheta_r = generateGaussianNoise(delta.theta, rot_var);
        // transform it from robot coordinate to world coordinate
        double theta = g_particles[i].theta;
        double dx_w = cos(theta) * dx_r - sin(theta) * dy_r;
        double dy_w = sin(theta) * dx_r + cos(theta) * dy_r;
        double dtheta_w = dtheta_r;
        // update the particle
        g_particles[i].x += dx_w;
        g_particles[i].y += dy_w;
        g_particles[i].theta += dtheta_w;
        if (g_particles[i].theta > PI)  g_particles[i].theta -= 2*PI;
        if (g_particles[i].theta < -PI)  g_particles[i].theta += 2*PI;

    }
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    // each particle will have an assumption, and the probability
    // of that assumption is calculated to update the weight

    // correlation
    double corr[NUM_PARTICLES] = {1}; 

    // loop through marker
    for (int i = 0; i < observations.size(); i++) {
        // get the marker position
        double m_x = g_markerLocations[observations[i].markerIndex].x;
        double m_y = g_markerLocations[observations[i].markerIndex].y;
        // get the distance and orientation
        double m_dist = observations[i].distance, m_ori = observations[i].orientation;
        // get the variance of sensor noise;
        double sen_var = (g_robotParams.sensor_noise_distance * fabs(m_dist) +
                        g_robotParams.sensor_noise_orientation * fabs(m_ori));

        //loop through particles
        for (int j = 0; j < NUM_PARTICLES; j++) {
            // get the position of particle
            double p_x = g_particles[j].x;
            double p_y = g_particles[j].y;
            double p_theta = g_particles[j].theta;
            // get the relative landmark position in robot coordinate
            double dx_r = m_dist * cos(m_ori);
            double dy_r = m_dist * sin(m_ori);
            // transfer it to the world coordinate
            double dx_w = cos(p_theta) * dx_r - sin(p_theta) * dy_r;
            double dy_w = sin(p_theta) * dx_r + cos(p_theta) * dy_r;
            // get the assumption of marker position
            double m_x_as = p_x + dx_w;
            double m_y_as = p_y + dy_w;
            // calculate the probability p(o|s);
            double dis = ((m_x - m_x_as) * (m_x - m_x_as) + (m_y - m_y_as) * (m_y - m_y_as));
            double p = exp(-dis / (2.0 * sen_var));
            g_weights[j] *= p;
        } 
    }
    normalizeWeight(g_weights);
}

/**
 * normalizeWeight()
 * This function is used to normalize weight
 */
void normalizeWeight(std::vector<double>& weight) {
    double min_val = *std::min_element(weight.begin(),weight.end());
    //shift and calculate sum
    double sum = 0;
    for (int i = 0; i < weight.size(); i++) {
        if (min_val <= 0) weight[i] = weight[i] - min_val + pow(10,-6);
        sum += weight[i];
    }

    for (int i = 0; i < weight.size(); i++) {
        weight[i] /= sum;
        //printf("%.3f\n", weight[i]);
    }
}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // initialize particles and 
    //initialize importance weights to be uniformly distributed

    for (int i = 0; i < NUM_PARTICLES; i++) {
        g_particles.push_back(robotState);
        g_weights.push_back(1.0 / NUM_PARTICLES);
    }
    // intialize robot parameters and landmarks location
    g_robotParams = robotParams;
    g_markerLocations = markerLocations;
}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
   // draw all particles
   int pixelX, pixelY;

   double globalX, globalY;
   
   // Draw cyan colored points at specified global locations on field
   glBegin(GL_POINTS);
   glColor3f(0.0, 1.0, 1.0);
   
   for (int i = 0; i < NUM_PARTICLES; i++) {
        globalX = g_particles[i].x;
        globalY = g_particles[i].y;
        global2pixel(globalX, globalY, pixelX, pixelY);
        glVertex2i(pixelX, pixelY);
   }
   glEnd();
}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here
	
	return 0;
}


/**
*generateGaussianNoise()
*This function generate gaussian noise given mean and vaiance
*It uses Box-Muller Transform
*code is from wikipedia
*https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
*/
double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

/**
*resample()
*use Low Variance Sampling technique to resample the particles
*/
std::vector<RobotState> resample(std::vector<RobotState>& particles, std::vector<double>& weights) {

    printf("\n");
    std::vector<RobotState> new_particles;

    double increment = 1.0/NUM_PARTICLES;
    double val = rand() * (1.0 / RAND_MAX);
    int ind = 0;
    for (int i=0; i<NUM_PARTICLES;i++) {
        val += increment;
        while (weights[ind] < val) {
            val -= weights[ind++];
            if (ind >= NUM_PARTICLES) ind = 0;
        }
        
        new_particles.push_back(particles[ind]);
    }

    // set weights to uniform distributed
    std::fill(weights.begin(), weights.end(), 1.0 / NUM_PARTICLES);
    return new_particles;
}


/**
 * Main entrypoint for the program.
 */
int main (int argc, char ** argv)
{   
    srand(time(0));
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    //runMainLoop(argc, argv);
    
    runMainLoop(argc, argv);

    return 0;
}

