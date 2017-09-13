# Unscented Kalman Filter

Project for the Self-Driving Car Engineer Nanodegree Program

---

## Overview

In this project, I implemented an unscented Kalman filter using the CTRV (Constant Turn Rate and Velocity Magnitude) motion model. I used the same bicycle simulation data set from the [extended Kalman filter project](https://github.com/jjaviergalvez/CarND-Extended-Kalman-Filter-Project) (EKF). That way I can compare the results with the EKF project.

All Kalman filters have the same three steps:

1. Initialization
2. Prediction
3. Update

A standard Kalman filter can only handle linear equations. Both the extended Kalman filter and the unscented Kalman filter allow you to use non-linear equations; the difference between EKF and UKF is how they handle non-linear equations. But the basics are the same: initialize, predict, update.



## Dependencies

* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Files in the src Folder

The files you need to work with are in the src folder of the github repository.

* `main.cpp` - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `ukf.cpp` - initializes the filter, calls the predict and update function, defines the predict and update functions
* `tools.cpp` - function to calculate RMSE

The only files I modified are `ukf.cpp` and `tools.cpp` where TODOs comments appear.

## Data

The data file information is provided by the simulator and is the same data files from [EKF](https://github.com/jjaviergalvez/CarND-Extended-Kalman-Filter-Project). Each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.

Although the data set contains values for yaw and yawrate ground truth, there is no need to use these values. `main.cpp` does not use these values, and I only calculate RMSE for x, y vx and vy. You can compare your vx and vy RMSE values from the UKF project and the EKF project. For UKF, vx and vy RMSE should be lower than for EKF; this is because we are using a more detailed motion model and UKF is also known for handling non-linear equations better than EKF.

