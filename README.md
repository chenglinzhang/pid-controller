## 4. PID Controller Project

### Self-Driving Car Engineer Nanodegree Program - Term 2

In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

One more thing. The speed limit has been increased from 30 mph to 100 mph. Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! 


### Demo

I started off with the default driving speed of 30 mph. The final program achieves the max driving speed of 100 mph. The following demo shows a max driving speed of 65 mph:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/HGbhvGvlKnc/0.jpg)](https://www.youtube.com/watch?v=HGbhvGvlKnc)


### Reflections

1. PID controller

A Proportional–Integral–Derivative controller (PID controller) is a control loop feedback mechanism. It continuously calculates an error value as the difference between a desired setpoint and a process variable and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).

PID controls car steering or speed by minimizing total errors from three component gains: proportional, derivative, and integral, in the following:
```
  total_error = - Kp*p_error - Kd*d_error - Ki*i_error;
``` 
where total_error is the target to minimize in every moment, p_error is for proportional, d_error for derivative, and i_error for inegral, and Kp, Kd, and Ki are respective coefficients. 

Propotional component Kp * p_error is the error we want to correct in every moment. Derivative component Kd * d_error adds resistance to the correction to make the drive more stable and not to oscillate across the target line. Integral component Ki * i_error adds control to minimise accumulated errors and pulls the car out of drifts parallel to the target line and helps to move the car to the target line faster. 

An excellent video that demostrates how the proportional, integral, and derivative gains affect the performance of a vehicle can be found at https://www.youtube.com/watch?v=4Y7zG48uHRo.

2. Parameter Tuning

Two PID controllers have been implemented: a steering PID and a throttle PID. They controll steering angles and throttle speeds respectively. Each PID controller depends on three coefficients: Kp, Ki, and Kd for error calculation. A twiddle algorithm has been impletemented to help finding the right coefficient values. However, I found the algorithm is slow to converge. I did manual tuning for the coefficients and the results are in the following:

```
                Kp   Ki      Kd
  Steering PID: 0.2, 0.004,  3.0
  Throttle PID: 0.1, 0.0006, 0.75
```

3. Prediction

PID controller is dynamic and responsive. In additon to the stability problem illustrated in the class, what will be better if the road conditions ahead (turns, bridges, shadows, etc) can be projected into the target setpoints. This is a prediction problem that requires additional modelling.

### Basic Build Instructions

```
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid [--twiddle]`. 
```

### Dependencies

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)


### Code Style

Follow [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as far as possible.

