# localization_challenge_Yifan_Yang

In this challenge, I implemented particle filter based Monte Carlo localization.

##There are several reason that I choose this algorithm:
- Easy to implement and visualize given the template provided
- Particle filter is multimodal so that kidnapped problem can easily be solved
  compare to other unimodal method like kalman filter

##Some Implementation detail:
To keep track all the particles, it's weight, whether the robot is kidnapped and 
number of particles, I set up some global variables.

###Some method to help the implementation.
- double generateGaussianNoise(double mu, double sigma);
  This methold can generate gaussian distributed noise given the mean and variance, it is used
   for the motion update.

- std::vector<RobotState> resample(std::vector<RobotState>& particles, std::vector<double>& weights);
  This method is used to resemple the particles when the number of efficient particles is lower than 
  a threshold

- void reInitializeParticles(std::vector<RobotState>& particles, std::vector<double>& weights);
  This method generated uniformly distributed even weighted particles in the play ground. It is use in 
  the initialization and every time a kidnapped is detected. 

### A simple pipeline
Initialize particles and weights
loop
	motionUpdate() 
	if Detect Landmark
		sensorUpdate()
	getRobotPositionEstimate()
	if Neff < threshold resample()
end
	
explanation:
- In motionUpdate(), the position of particles is updated given it's state (x,y,theta) and the odom info 
  with gaussian noise is added.
- In senseUpdate(), p(o_t|s_t) is calculate for each particle. I calculate the bearing and distance for each
  particle, and calculate the probability compare to the observation. And weight is updated accoding to p(o|s)
  and normalized
- In getRobotPositionEstimate(), the particle with largest weight is regarded as the best case.

## Key decisions and some trick
- For the resample of particle, I used a technique called low variance sampling technique, which is quite robust.
- In the sensor update, if the largest probability is less than a threshold, the kinapped value will increase, 
  when the value is large enough, the robot will think itself being kidnapped, and reinitialze particles
- For the calculation of p(o|s) in sensing step, I assume the observation of distance and bearing to be independent,
  so it would simply be p(o_dist, o_angle|s_t) = p(o_dist|s_t) * p(o_dist|s_t)
- I generally don't wan't any possibility to be 0, so I set a lower bound of weight to be 10^-6
- I implemented a low pass filter for the final estimated position in order to make it move smoothly.

### some value I'm using 
const int NUM_PARTICLES = 2000;
const double RESAMPLE_THRESH = 0.8;
const int KIDNAPP_THRESH = 10;
all the gaussian noise variance I used are from the given value


## Improvement can be done
There are so many improvements can be done, I'll just name a few:
- There might be some criticle bug in the program that I overlooked and correctd by the power of particle filter,
  so I might wanna do step by step check.
- clean up the code to make it easier to read.
- play around all the parameters like number of particles, resample threshold, etc, to get better performance.
- make the system more robust to the kidnapped problem.
- Some other resampling technique can be tested.