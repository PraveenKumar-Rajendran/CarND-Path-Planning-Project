# CarND-Path-Planning-Project

This project is part of the Self-Driving Car Engineer Nanodegree Program.

### Goals
The goal of this project is to safely navigate a virtual highway with other traffic, while adhering to the speed limit of 50 MPH and avoiding collisions. The car receives localization and sensor fusion data, as well as a sparse map of waypoints around the highway. The car should attempt to maintain a speed close to the speed limit, pass slower traffic when possible, and avoid hitting other cars. The car should always stay within the marked road lanes, except when changing lanes. The objective is for the car to complete one loop around the 6946m highway, taking a little over 5 minutes. Additionally, the car should not exceed a total acceleration of 10 m/s^2 or a jerk greater than 10 m/s^3.

## Basic Build Instructions

1. Clone this repository.
2. Create a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run the program: `./path_planning`

## Details

1. The car utilizes a perfect controller and visits every (x,y) point received in the list every 0.02 seconds. The (x,y) points are measured in meters, and the spacing of the points determines the car's speed. The angle of the car is determined by the vector from one point to the next. The planner receives (x,y) point paths that should not result in a total acceleration exceeding 10 m/s^2 or a jerk exceeding 50 m/s^3. It is recommended to average the total acceleration over 1 second and measure the jerk from that value.

2. There may be a slight latency between the simulator running and the path planner returning a path. During this delay, the simulator continues using the last given points. To ensure a smooth transition, it is advisable to store the last points used and either extend the previous path or create a new path that smoothly connects with it. The variables `previous_path_x` and `previous_path_y` can be useful in this transition, as they represent the last points given to the simulator controller with the processed points already removed.

## SPLINE!!!

A highly useful resource for this project, which aids in creating smooth trajectories, is the single-header file spline function available at http://kluge.in-chemnitz.de/opensource/spline/.

<br/><br/>
<h1 align="center"> Addressing Project Rubric </h1>

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
<h2 align="center">Compilation </h2>

<em>Criteria 1: The code compiles without errors using cmake and make.</em>

<h2 align="center">Valid Trajectories </h2>

<em>Criteria 1: The car can drive at least 4.32 miles without any incidents (both in the local environment and in the Udacity workspace).</em>

<em>Criteria 2: The car adheres to the speed limit of 50 mph, as observed in the simulator.</em>

<em>Criteria 3: The car does not exceed the maximum acceleration and jerk limits, as observed in the simulator.</em>

<em>Criteria 4: The car avoids collisions with other vehicles on the road, as observed in the simulator.</em>

<em>Criteria 5: The car stays in its lane, except when changing lanes, and transitions occur within 3 seconds, as observed in the simulator.</em>

<em>Criteria 6: The car can smoothly change lanes when

 it is behind a slowly moving car, and the adjacent lane is clear of traffic, as observed in the simulator.</em>

<h2 align="center">Reflection </h2>

<h3>Path Planning</h3>

The path planning implementation for this project consists of five main sections.

### SECTION 1: Safety Flag Setting

In this section, lane shift flags are set based on sensor fusion data to avoid collisions and maintain the speed as much as possible. The flags include a collision warning flag and lane availability flags.

### SECTION 2: Lane Shift Behavior

After setting the flags, the actual action variables, such as `lane_number` and `ref_velocity`, are modified in this section.

### SECTION 3: Path Decision

This section calculates the reference x, reference y, and reference yaw, and pushes the reference x and y points to `ptsx` and `ptsy`.

### SECTION 4: Evenly Spaced Points

As mentioned in the project Q&A video, waypoints are added at even intervals of 30m ahead of the starting reference in the Frenet coordinate system. These points are pushed to `ptsx` and `ptsy`, and the car's reference angle is shifted to 0 degrees.

### SECTION 5: Usage of Spline for Trajectory

In this section, `ptsx` and `ptsy` are used as setpoints to create a spline. The spline points are calculated to maintain the desired reference velocity. The path planner always outputs 50 points by filling it with previous points. The `next_x_vals` and `next_y_vals` are fed to the simulator.

<h3>Results</h3>

![Udacity Workspace Result](./results/path_planning_Udacity_workspace.PNG)

The GIF file below shows the simulation video recorded in the Udacity workspace:

![Udacity Workspace GIF](./results/path_planning_Udacity_workspace_1.4MB.gif)

You can also find the simulation video recorded in a local environment [here](./results/local_environment_result.webm).