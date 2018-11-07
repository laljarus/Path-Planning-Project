# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program  


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided, there is also a sparse map list of way points around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946 m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Path Planner Overview

The path planner takes the input from other systems such as sensor fusion system, localization to know the state of the car and the surrounding objects. Based on this information the path planner should generate a trajectory and provide it to the motion controller.

The following picture shows the overview of an autonomous car software.

![image](Pictures/PathPlanningArchitecture.png)

The blue box in the above picture shows the tasks of the path planner which are:

	- Behaviour Planning
	- Prediction
	- Trajectory Generation

These modules are explained below. 

## Behaviour Planning

The behaviour planning task takes high level decisions such as optimal route from start point to the goal, should the car keep the current lane or take left/right lane etc based on the current situation. The planning of route is usually done using a search algorithm such as A*. But for this project the car has to drive around a highway track there is no starting point and goal. So this is out of scope. 
The behavior planner implemented in this project takes only one of following high level decisions:

	- Drive with a constant speed in the current lane
	- Accelerate in the current lane
	- Brake in the current lane
	- change lane left at a constant speed
	- change lane right at a constant speed 

The behaviour planner is implemented using a finite state machine.


## Prediction

In this project a very simple prediction is used all the other cars are assumed to travel a constant speed in the current lane which the car is in at the moment. A prediction trajectory of neighboring cars are generated using this simple assumption. Which is used to calculate a cost for possible collision for different behaviour of our car. This is used by the state machine to choose appropriate action.  

![image](Pictures/StateMachine.png)

The above picture shows the state machine of the behavior planner. It has three states at top level Initial Acceleration, changing lane and keep lane. The state initial acceleration is the state that the program starts with. This state generates a jerk minimal trajectory using the function Generatetrajectory() which generates a trajectory for a given set speed, set lane and duration.  The state Changing lane is active when the car changes lane. It generates a trajectory setting a constant speed which is the car speed during initiation of changing lane and this state is active until the car has changed its lane and it stays in the new lane for 1s. Finally the last state is the keep lane state. This state is active during normal operation of the car.During this state the car monitors the behavior of other cars and responds based on the situation. 
