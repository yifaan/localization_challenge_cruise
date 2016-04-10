/**
 * Localization Challenge
 * 
 * main.h
 *
 * Function prototypes for user functions.
 */
#ifndef _MAIN_H_
#define _MAIN_H_

#include <vector>
#include "RobotDefinitions.h"

// Sets current robot position estimate
void getRobotPositionEstimate(RobotState& estimatePosn);

// Robot motion update
void motionUpdate(RobotState delta);

// Landmark sensor update
void sensorUpdate(std::vector<MarkerObservation> observations);

// Initialization and UI methods
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS]);
void mydisplay();
int  mykeyboard(unsigned char key);

//generateGaussianNoise
double generateGaussianNoise(double, double);

// Set the global variables
const int NUM_PARTICLES = 10;
// particles
std::vector<RobotState> g_particles;
// importance weight
std::vector<double> g_weight;
RobotParams g_robotParams;
FieldLocation* g_markerLocations;

#endif
