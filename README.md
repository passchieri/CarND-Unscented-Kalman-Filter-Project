# Unscented Kalman Filter
**Igor Passchier**

igor.passchier@tassinternational.com

## Introduction
This is the project submission of the project on Unscented Kalman Filters. The remainder of this readme follows the project [rubric](https://review.udacity.com/#!/rubrics/783/view), and finishing with some additions.

## Compiling
The project is based on the starter code provided. The CMakelists.txt has been extended to include the additional source files, but the structure is unchanged. The code should compile on any system, but has been checked on MacOS with command line tools and with XCode.

To build and run:
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

## Accuracy
The project has been developed and tested with the "obj_pose-laser-radar-synthetic-input.txt", and should have RMSE values less than  [.09, .10, .40, .30].
The final RMSE values obtained are [0.062	0.086	0.147	0.164], so meeting the criteria.

## Following the correct algorithm
### Follow the algoritmn process
The overall process flow is handled in [main.cpp](src/main.cpp) line 236. If a command line option is provided, the main routine will read the sensordata and ground truth data directly from the input file (line 122), while if no command line option is provided, than the input from the simulator is expected (line 147). In this way, visual feedback can be provided via the simulator, while parameter tuning can be done much faster with the direct input. Processing the sensor input is done on line 35, and when the input line is processed, the actual filter is called on line 88.

All process state variables are stored in a new class State, implemented in [state.cpp](src/state.cpp).

Relevant output is written to datafiles for further processing, and RMSE values are directly provided on standard out.

The UKF algorithm is implemented in [ukf.cpp](src/ukf.cpp). The full process flow is implemented in the function ProcessMeasurement, on lin e 95. 

### Handle the first measurement appropriately
The UKF is initilized to a large extend statically in the constructor, line 13. The state vector is initialized with zeros, except for v, because they can all have symmetric values around 0. The velocity is expected to be larger than 0, as we expected a driving bicycle. 5 m/s is a realistic value for a bicycle.

The covariance matrix is initialized with realistic values, see line 28. Note, that the first 2 diagonal ellements will be updated on receiving the first measurement.

The lambda and the weights are initialized with the proposed values, see line 41 and further. The process noise parameters std_a and std_yawdd have been optimized, see later. The laser and radar correlation matrices are based on the provided standard deviations, see line 73 and further.

On receiving the first measurmeent, the Initilize function is called to initialize the UKF with the first measurement data, line 119. Depending on the sensor type, state x and y are initialized. Also P(0,0) and P(1,1) are initialized based on the accuracy of the sensor.

### First predict, then update
If the filter is already initialized, then the Prediction function is called on line 107. This function generated the Augmented sigma points and predicts the values based on the timestep. For limiting angles and angle differences to -pi -- pi, a new function has been added to [tools.cpp](src/tools.cpp). This is used throughout the code to prevent incorrect ranges.

Starting at line 109, the update functions are called for Radar otr Lidar data, where it is also checked whether the data is to be ignored. Both are almost identical, except that they implement different sensor models. The sigma points resulting from the prediction step are reused for the update step. The functions follow exactly the process described in the lessons.

As a last step, also the NIS values are calculated, see e.g. line 257.

### Handle laser and radar measurements
See the functions UpdateLidar (line 188) and UpdateRadar (line 264).

## Code efficiency

## Additional work

Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


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
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

