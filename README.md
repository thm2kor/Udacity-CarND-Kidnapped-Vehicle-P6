# Particle Filter Implementation
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Project Introduction
The objective of this project is to find a robot, which has been kidnapped and transported to a new location. The robot has a map of its location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. The project will implement a 2 dimensional particle filter in C++ to detect the actual location of the Robot.

A [simulator](https://github.com/udacity/self-driving-car-sim/) is provided by Udacity which can be used to visualize the motion of the kidnapped car with all of its landmark measurements. The simulator provides the script for the noisy position data, vehicle controls, and noisy observations. The simulator also displays the best particle's sensed positions, along with the corresponding map ID associations. If the green laser sensors from the car nearly overlap the blue laser sensors from the particle, this means that the particle transition calculations are done correctly.

The project communicates with the Simulator using the [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) object. The project calculates the best particles state and transmits the data to the Simulator.

[//]: # (Image References)

[image1]: ./images/particle_filter_process.png "particle filter implementation process"
[image2]: ./images/particle_count.png "particle count"
[image3]: ./images/motion_models.png "motion model"
[image4]: ./images/pseudo_code.png "pseudo code"
## Implementation of a Particle filter
The overall implementation of a Particle filter involves the following steps: <br>
![particle_filter_process][image1]

### Initialization
An empirical [study](https://knowledge.udacity.com/questions/29851), cited by one of the Udacity mentors, on the number of particles against the error in x and y shows that the error rates are at its minimum for a particle count of 100 or more.
![particle_count][image2]<br>
The results from the above study is used as the basis for the setting the particle count to **100**.
The associated positions `x`, `y`, and heading `theta` of each particle is initialized using GPS input. As with all sensor based operations, this step is impacted by noise. The noise components are randomly chosen from a normal distribution with a mean around the respective values of  `x`, `y`, and `theta` and a standard deviations of **0.3, 0.3 and 0.001** respectively.
```c++
// GPS measurement uncertainty [x [m], y [m], theta [rad]]
double sigma_pos [3] = {0.3, 0.3, 0.01};
```

### Prediction Step
In the prediction step, the vehicle location is calculated based on bicycle motion model. Using the control input variables (velocity and yaw-rate measurements), the vehicle's location is predicted by the following equations:<br>
![motion_model][image3]

Since the sensor measurements are noisy, the noise components are derived from a normal distribution with a mean around the predicted values of  `x`, `y`, and `theta` and a standard deviations of **0.3, 0.3 and 0.001** respectively.

The position of the car is described in map coordinates. The sensor measurements are described in vehicle coordinates, which has the x-axis in the direction of the car’s heading, the y-axis pointing orthogonal to the left of the car, and the z-axis pointing upwards.

### Update Step

#### Data association

### Resampling step


---

**NOTE:** All screenshots linked in this README file are taken over from the Udacity CarND program - Lesson - Particle filter.

---
### Running the Code
The Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO. Once the installation for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively the following shell scripts could be used to build and run the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh
