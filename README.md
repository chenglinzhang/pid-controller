## 2. PID Controller Project

### Self-Driving Car Engineer Nanodegree Program - Term 2

In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

One more thing. The speed limit has been increased from 30 mph to 100 mph. Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! 


### Demo

I started off with the default driving speed of 30 mph. The final program achieves the max driving speed of 100 mph. The following demo shows a max driving speed of 65 mph:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/xP5Y3oW2rjQ/0.jpg)](https://www.youtube.com/watch?v=xP5Y3oW2rjQ)


### Reflections

1. PID controller

A Proportional–Integral–Derivative controller (PID controller) is a control loop feedback mechanism. It continuously calculates an error value as the difference between a desired setpoint and a process variable and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).

2. Parameter Tuning

PID controller depends on three coefficients: Kp, Ki, and Kd for error calculation:
```
  total_error = -Kp * p_error - Ki * i_error - Kd * d_error;
```  

Two PID controllers have been implemented: a steering PID and a throttle PID. They controll steering angles and throttle speeds respectively. 

A twiddle algorithm has been impletemented to help finding the right coefficient values. However, I found the algorithm is slow to converge. I did manual tuning for the coefficients and the results are in the following:

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
4. Run it: `./pid [twiddle]`. 
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

