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

//normalize weight
void normalizeWeight(std::vector<double>&);

//resemple particles
std::vector<RobotState> resample(std::vector<RobotState>&, std::vector<double>&);

//generate random particles to overcome kidnaped problem
void reInitializeParticles(std::vector<RobotState>&, std::vector<double>&);


// Set the global variables
const int NUM_PARTICLES = 2000;
const double PI = 3.14159265358979323846;

// set the value for resample 
const double RESAMPLE_THRESH = 0.8;

// The value for kidnapp, if it is larger than some threshold it is kidnapped;
int kidnapp;
const int KIDNAPP_THRESH = 10;

// particles
std::vector<RobotState> g_particles;

// best position
RobotState g_estimate_pos;

// importance weight
std::vector<double> g_weights;
RobotParams g_robotParams;
FieldLocation* g_markerLocations;

#endif
