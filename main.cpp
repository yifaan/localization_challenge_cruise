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
    // TODO: Write your procedures to set the current robot position estimate here
    
//    estimatePosn.x = 0.0;
//    estimatePosn.y = 0.0;
//    estimatePosn.theta = 0.0;
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
    double rotVar =  g_robotParams.odom_noise_rotation_from_rotation * drot
                    +g_robotParams.odom_noise_rotation_from_translation * dtrans;
    double tranVar = g_robotParams.odom_noise_translation_from_rotation * drot
                    +g_robotParams.odom_noise_translation_from_translation * dtrans;
    for (int i = 0; i < NUM_PARTICLES; i++) {
        // for each particle, add noice to its odometry;
        double dx_r = generateGaussianNoise(delta.x, tranVar);
        double dy_r = generateGaussianNoise(delta.y, tranVar);
        double dtheta_r = generateGaussianNoise(delta.theta, rotVar);
        // transform it from robot coordinate to world coordinate
        double theta = g_particles[i].theta;
        double dx_w = cos(theta) * dx_r - sin(theta) * dy_r;
        double dy_w = sin(theta) * dx_r + cos(theta) * dy_r;
        double dtheta_w = dtheta_r;
        // update the particle
        g_particles[i].x += dx_w;
        g_particles[i].y += dy_w;
        g_particles[i].theta += dtheta_w;
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
    // TODO: Write your sensor update procedures here

    
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
        g_weight.push_back(1.0 / NUM_PARTICLES);
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
    // TODO: Write your drawing procedures here 
    //       (e.g., robot position uncertainty representation)
    
    
   // Example drawing procedure
   int pixelX, pixelY;
   double globalX = g_particles[0].x, globalY = g_particles[0].y;
   const double POINT_SPREAD = 0.2;
   
   // Draw cyan colored points at specified global locations on field
   glBegin(GL_POINTS);
   glColor3f(0.0, 1.0, 1.0);
   
   global2pixel(globalX, globalY, pixelX, pixelY);
   glVertex2i(pixelX, pixelY);
   
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

