[image1]: ./images/SuccessRun.jpg "Successful Run for more than 4.32 miles"
[image2]: ./images/SuccessLaneChange.jpg "Successful Lane Change"

# Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Overview
In this project, we need to implement a path planning algorithm to drive a car on a highway on a simulator provided by Udacity (the simulator could be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)). The simulator sends car telemetry information (car's position and velocity) and sensor fusion information about the rest of the cars in the highway (Ex. car id, velocity, position). It expects a set of points spaced in time at 0.02 seconds representing the car's trajectory. The communication between the simulator and the path planner is done using [WebSocket](https://en.wikipedia.org/wiki/WebSocket). The path planner uses the [uWebSockets](https://github.com/uNetworking/uWebSockets). WebSocket implementation to handle this communication. Udacity provides a seed project([here](https://github.com/udacity/CarND-Path-Planning-Project)) to start from.
 
For instructions on how to install these components on different operating systems, please, visit [Udacity's seed project](https://github.com/udacity/CarND-Path-Planning-Project).

## Basic Build Instructions

1.  Clone this repo.
2.  Make a build directory: `mkdir build && cd build`
3.  Compile: `cmake .. && make`
4.  Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5 
* make >= 4.1  
* gcc/g++ >= 5.4 
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## Results
 
#### The car drives according to the speed limit.
The car was within the speed limit.

#### Max Acceleration and Jerk are not Exceeded.
There was no Max Acceleration or Jerk.

#### Car does not have collisions.
There was no collision even till 9 milles.

![alt text][image1]

#### The car stays in its lane, except for the time between changing lanes.
The car stays in its lane most of the time except when it changes lane because of traffic or to return to the center lane.

#### The car is able to change lanes
The car change lanes when the there is a slow car in front of it, and it is safe to change lanes (no other cars around) or when it is safe to return the center lane.

![alt text][image2]
