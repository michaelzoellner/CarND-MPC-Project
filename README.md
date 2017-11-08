# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Basic approach

The implementation of the Model-predictive controller is strongly based on the 
code of the previous lesson. A kinematic model is used and it is sufficient to 
project course of the vehicle depending on the two actuators (acceleration and 
steering) with the required precision.

Some of the differences compared to the lesson code will be explained below.

## Tricky things to figure out

### Projection of the vehicles state after latency

The simulation involves some latency between sensoring (positon, orientation 
and speed) and actuation (acceleration and steering), very much comparable to 
any complex mechatronic control system on a real vehicle. This means that at 
time t = t0, there is a timestep t = t0 + t_latency, which is the next time at 
which we can influence the vehicles movement with our actuation. The time 
between t0 and t0 + t_latency will have (already) passed when our actuation is 
calculated and passed on.

In order to calculate the best-fitting actuation, not the current the state of 
the vehicle at t0, but the projected state at t0 + t_latency is considered. 
This is done by applying the simple kinematic model to the sensed position, 
orientation and speed at t0.

### State needed in the vehicles coordinate system

The MPC "sees the world" from the vehicles point of view. That means that the 
waypoints need to be transformed to that position and orientation, as well as 
the current state of the vehicles as well. This "results" in px = py = psi_unity = 0 
for all times!

### Diverse unit, offset and orientation corrections

The kinematic model of the vehicle is working with SI units, whereas the 
simulator telemetry data is using miles per hour and normalized throttle and 
steering. Also, it is using clockwise direction as positive, where as the 
mathematical world is turning counter clockwise for positive values. Therefore, 
diverse unit and orientation transformations had to be applied. 

### Reference speed

I used a fixed reference speed at first. However this is not realistic (unless 
imagine a really low speed limit). When I think about my realworld driving on a 
countryside road, the reference speed is built on two facts:
--* A general speed limit (100 km/h, I used 50 m/s in the project however)
--* The maximum transversal acceleration (I chose 7 m/s^, though great sports cars achieve more than 10 m/s^2)
As the transversal acceleration is v^2/R, I had to estimate the radius based on 
the polynomial coefficients to figure out the reference speed.

### Variables types and how they are initiated by C++

I spent **by far the most time** with a "bug" that results from using C++ with a 
MATLAB-trained mind. Thus I had to learn that the mph to m/s transformation 
would not result in the same numbers if you write either of the two 
```
v = 1610/3600 * v; 
v = v * 1610/3600;
```
Same goes for the steering angle limits
``
vars_lowerbound[i] = -0.4; // returns a float
vars_lowerbound[i] = -25/180 * pi(); // returns an integer (zero!)

## Tweaking of the hyperparameters

Tuning the number and time of the projection steps is a trade-off between 
calculation performance and stability. I figured that 10 steps with a deltaT of 
0.2s, hence a projection/optimization horizon of 2s sufficient.

The other tuning job involves the different weights of the cost function. As 
the most fundamental requirement is to stay on track and move forward, I 
started by using only the two cost weights that penalize cross-track and 
velocity error. Using the other weights (orientation error, actuator use and 
gradient) in addition to this led to a further improved driving behavior. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
