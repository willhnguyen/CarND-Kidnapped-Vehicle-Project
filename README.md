# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree

This project aims to implement a particle filter to provide a car the ability to localize itself with the ability to sense the presence of landmarks nearby.

## Dependencies
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

## Build and Run Instructions
To build the program, run the following sequence of command line instructions.

```bash
$ cd /path/to/cloned/repo
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To see the program in action, make sure to download the [Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases), run it, and choose the second simulation option. Then, execute the program by executing the `particle_filter` program in the `build` folder.

```bash
$ ./particle_filter
```

## Project Information
In this project, a lost car can sense landmarks and needs to use the landmarks to identify where it is located. To do so, a particle filter will be implemented. The vehicle itself knows the general vicinity of where it is located (i.e. it has the local map through GPS). Note that GPS typically can give the general vicinity but not an accurate location which is desired in this project.

Particle filters work by identifying whether a randomly placed state has a high probability of matching the vehicle's current state. The idea is that by collecting data over time, only few positions can correctly correlate to the vehicle's driving history.

To begin, a collection of points (particles so to speak) with random states are initialized and placed in the space. These points include cartesian positions x and y, and a heading orientation theta.

To determine whether a point is likely to match the vehicle's state, each of those points undergo a simulation to update its location after some time step. Once the points are updated, the landmarks in range are identified and checked against the landmarks sensed by the actual vehicle. The sensed and expected landmarks are associated with each other by the nearest neighbor approach. Then, this association is used to determine the weight for each point which represents the probability that the point is likely to be the vehicle's current state.

Once each point's weights have been calculated, the collection of points are resampled based on the points' weight distributions. The most likely point isn't chosen because that point might not be the correct point and some uncertainty is welcomed to further narrow down the belief of the vehicle's actual state.

The algorithm continuously loops back to the prediction step. However, since some of the resampled points may have the exact same state, some noise is added to the state during the prediction step to add variance to the simulation.

## Project Implementation Details
The only modified file is the `src/particle_filter.cpp` file. In this file, 5 methods are updated: `init()`, `prediction()`, `dataAssociation()`, `updateWeights()`, and `resample()`.

In `init()`, the particle filter is initialized with a collection of particles with random state. The amount of particles to initialize is up to the designer—I chose 100.

In `prediction()`, delta time, velocity, and yaw rate (turn rate) are provided to predict each particle's next state. This uses the equations seen in the previous unscented Kalman filter project. Some variance is added from a Gaussian noise generation.

`dataAssociation()` is required for `updateWeights()` to identify the which expected landmark is most likely the observed landmark. A brute force algorithm is used to scan and identify the closest landmarks to each observation.

`updateWeights()` use the associated landmarks to calculate a weight for each particle. This weight represents the particle's probability of being the actual vehicle's location.

In `resample()`, the weights are used to randomly pick a point from the collection of points. The idea is to keep the collection of particles the same size, but keep the most likely ones. A discrete_distribution random number generator is used to properly resample as desired.
