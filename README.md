# PID Controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

In this project, I'll implement a PID controller in C++ to control the vehicle, so it can successfully drive around tracks without leaving the road.

Specifically, this project is a revisit of the [Behavioral Cloning Project](https://goo.gl/6njxcy). Instead of using a convolutional neural network for end-to-end control of the vehicle,  I use a [proportional–integral–derivative controller (PID controller)](https://en.wikipedia.org/wiki/PID_controller).


## Project Demo

![demo](demo.gif)

You can also watch a demo video of this project [on my YouTube](https://youtu.be/poxOXTRucyQ). 

Alternatively, if you want to watch the results of my [Behavioral Cloning Project](https://goo.gl/6njxcy), you can watch a simulation video for the [same Lake Track](https://youtu.be/bJPQDfu15sc), or the [more challenging Jungle Track](https://youtu.be/eu2-NLfhzYQ).


## Get the Code

You can download this folder of code [here](https://tugan0329.bitbucket.io/downloads/udacity/car/pid/p9-PID-controller.zip)


## Project Setup

This project involves the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two bash files (`install-mac.sh` and `install-ubuntu.sh`) that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

## Run the Project


Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

```
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid
```

Then, you can open the [Simulator you downloaded](https://github.com/udacity/self-driving-car-sim/releases) and choose "Project 4" to run the project.


## Reflection

### PID Control

A PID controller continuously calculates an error value `e(t)` as the difference between a desired setpoint (SP) and a measured process variable (PV). In this project, it's the cross track error (CTE), which measures the distance from the car to the center of the road line.

Then, based on the CTE, PID controller applies a correction to the **steering value** of the car, based on proportional, integral, and derivative terms (denoted P, I, and D respectively). 

The P (proportional) component corrects the steering value, directly proportional to the CTE. The D (derivative) component, instead, corrects the steering value proportionally to how fast the CTE is changing. So, the faster the CTE is changing, the more correction it applies. The I (integral) component corrects the steering value proportionally to the integral of all the CTE in the past timestamps. This term is helpful to correct for systematic bias in the car, so the longer the CTE is not decreasing, the more correction it applies over time.

All three terms scale proportionally to a small constant, and they all need to be tuned. To do them, we can apply various methods such as grid search, twiddle, SGD, or advanced methods like Ziegler-Nicholas method. However, since this project is evaluated mainly through simulator, I decide to do manual tuning in a similar process like twiddling. Eventually, the parameters I used are 0.35 for the P component, 0.8 for the D term, and 0.0025 for the I term. The effects of these components on the car indeed act as expected, and I used them to guide my manual tuning process as well. 

For example, If a car has a tendency to not turn as much at turning points, I will increase the D component. If a car doesn't try to correct and go to the center of the road much even if its' driving on a straight line, I will try to increase the P component.

### Smoothing

To help the car drive more smoothly, I added some additional controls. 

Specifically, instead of having the integral term keeps sums over all CTE error from past timestamps, which makes the I term has larger and larger influence as time goes (and eventually dominant the control decision), I only use the sum of a sliding window of past 100 timestamps for the past I term.

I also added manually tuned controls to let the car apply brakes when it's driving too fast, so I can cap the car at 30 mph at max. I also do smoothing on the steering values. Instead of steering above and below 1, I smooth them out to have steering angles between `[-0.95, 0.95]`, so it drives more smoothly. 





