# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

The purpose of this project is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The project requires obtaining RMSE values that are lower than a given tolerance. 

# Theory

## What are Kalman filters?
The Kalman filters is a family of algorithms used to combine measurements from multiple sources to produce estimates of unknown variables

## Why is it called a "filter?"

It's called a filter because it filters out the uncertainty.

Bayesian filter. A Bayes filter is an algorithm used in computer science for calculating the probabilities of multiple beliefs to allow a robot to infer its position and orientation. Essentially, Bayes filters allow robots to continuously update their most likely position within a coordinate system, based on the most recently acquired sensor data. This is a recursive algorithm. It consists of two parts: prediction and innovation. If the variables are normally distributed and the transitions are linear, the Bayes filter becomes equal to the Kalman filter. 

## Intuition
Take the example of a chicken attempiting to cross the road. The objective is to not get hit by oncoming traffic. It has eyes and makes predictions based on a motion model within its brain.

## Extended Kalman Filter (EKF)
the extended Kalman filter (EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance. In the case of well defined transition models, the EKF has been considered[1] the de facto standard in the theory of nonlinear state estimation, navigation systems and GPS.

Most systems are non-linear, so some attempt was immediately made to apply this filtering method to nonlinear systems. The EKF adapted techniques from calculus, namely multivariate Taylor Series expansions, to linearize a model about a working point. If the system model is not well known or is inaccurate, then Monte Carlo methods, especially particle filters, are employed for estimation. 

Kalman filterss only work with linear functions. Our solution is to make non-linear functions into linear by approximation. The Taylor Series helped get linear approximations of non-linear functions.

## Where did the non-linearity come from?

The motion model for LiDAR sensor is:

This can use the standard Kalman Filter.

RADAR measurements, however, occur in the polar coordinate system. The conversion to cartesian coordinates involves a non-linear transform:

We must linearly approximate the above function using an EKF.

# Implementation

## EKF Flow

1. To kick off the infinite loop, start by taking an initial sensor measurement from either LiDAR or Radar (whichever is available). Use the sensor measurement to generate the state vector.
2. Predict according to the movement model.
 a. Compute the time elapsed between the prvious and current measurements
 b. Update the transition matrix F with the elapsed time
 c. Update the process covariance matrix Q with the second, third, and fourth derivatives of the elapsed time and the process noise.
 d. Execute the prediction
3. Update the prediction with a measurement.
4. Loop back to step 2...rinse and repeat...forever!

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

