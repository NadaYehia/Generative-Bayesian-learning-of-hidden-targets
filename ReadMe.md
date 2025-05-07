This repository contains a generative model for learning hidden rewarded locations in a square arena using Bayesian inference. 

This codebase is an implementation for the manuscript in preparation (**A Model for rapid spatial learning via efficient exploration and inference**) by **[Nada Abdelrahman,  Wanchen Jiang, Joshua Dudman and Ann Hermundstad]**. 

The model simulates an agent that explores the arena, learns the locations of **uncued** rewarded locations **within tens of trials**, thus outperforming exisiting Reinforcement Learning (RL) algorithms undergoing the same task. 
The target locations are hidden from the animal's point of view

![task_setup2](https://github.com/user-attachments/assets/f8332ff1-e263-46e4-aa6f-0eef51983428)


Rewarding targets will switch locations without any cue to the animal

![task_Setup](https://github.com/user-attachments/assets/3b7d7c6d-a5bb-41c4-afe8-f1b88623ea32)

The code is optimized and parallelized for fast execution.

## Overview
The program starts by running main.m, which orchestrates the simulation and learning process. Below is a detailed breakdown of the key components and their functionalities:

## 1. Environment Class
The Environment class defines the physical properties of the arena and the target locations. It includes the following attributes:

- **Arena Size**: The dimensions of the square arena.

- **Target Locations & Sizes** : The coordinates and sizes of the rewarded targets.

- **Blocks**: The number of trials per target location.

 ### Methods
- setup_targets: A static method that takes the target centers and dimensions as input and outputs their respective corner coordinates. This is useful for visualization and collision detection.

- Future Plans: We plan to extend this class to handle obstacles and their setup in the environment.

## 2. Set Control Actions Space
This module defines the action space for the agent, which consists of possible speeds and angles. The action space is constrained by the physical limits of the arena, ensuring that the agent's movements are realistic and feasible.

## 3. Simulate Run
The simulate_run function simulates the agent's movement along a given trajectory and observes the outcome at the home port. It determines whether the agent successfully intercepts the target and receives a reward.

## 4. Surprise
The surprise function detects changes in the environment, such as shifts in target locations. If the observed outcome is too surprising (i.e., it deviates significantly from the agent's expectations), the agent resets its prior belief to a uniform distribution. This allows the agent to adapt to unexpected changes in the environment.

## 5. Belief in the candidate target locations and Bayesian update
The posterior map represents the agent's belief about the candidate target locations given the history of executed trajectories, their outcomes (reward or no reward), and the agent's internal model for receiving a given outcome given the current executed trajectory and that the target can be at any location within the arena. 

1- This distribution represents the probability of any location within the arena in its polar form, its radial distance and angle relative to the home port, being the target location.

2-The likelihood function reflects the agent's belief to observe the current outcome (reward or no reward) given the executed trajectory and that any location in the arena (is or is not) the target location.

3- We modeled this likelihood function as the sum of 2D Gaussian functions, each 2D function computes the probability of observing the current outcome over all the possible {R*,Theta*} locations in the arena given their Euclidean distances from the t-th point in the executed trajectory, {r_t,theta_t}.

## 6. Sampling Next Actions
It determines which points to visit given the current belief about target locations:
There is a continuum of possible sampling strategies. At one end of this axis, the agent can sample the points to visit in proportion to its belief about the candidate target locations. On the other hand, it can sample the points with peak probabilities in its current belief. 
1- Peak Sampler: Selects actions with the highest local reward probability. This strategy focuses on exploiting the most promising areas.

2- Proportional Sampler: Selects actions proportional to their reward probability in the posterior. This strategy balances exploration and exploitation.

## 7. Trajectory Planner
1- This module is responsible for planning a path connecting any pair of anchors given their distances and angle relative to the home port.

2- It weaves a continuous path between a list of N points, whose order of visiting is presorted to minimize the agent's total path length. 

3- The planner computes the generative functions parameterizations required to generate segments between every anchor pair.

4-The trajectory planner allows for an area of tolerance to hit around the anchors to minimize the total path length without sacrificing the path smoothness.

5- The trajectory planner minimizes the total trajectory path length by minimizing segments' curvatures between every anchor pair and moving the anchors' locations within the area of tolerance.

6- The planner cost function penalizes running longer paths, but not at the cost of running rough paths with sudden angular changes at the anchor points.


**Key Steps**:\
1- Sampler function selects the next anchor locations to run to given the agent's current belief.

2- Trajectory Planner: It generates a smooth trajectory connecting the optimized anchors. This module ensures that the agent's movements are efficient and natural.

3- Bayesian belief Update: this function updates the belief about target locations given the prior belief and the likelihood of observing the current outcome given the executed trajectory.

## How to Run the Code
Clone the repository:

```console
git clone <repository-url>
```
Navigate to the repository directory:
```console
cd <repository-directory>
```
Run the main script in MATLAB:
```console
main.m
```

## Demo
 
 1- A simulation of the model learning a target. Left window shows the executed trajectories and right window shows the agent's posterior belief about target locations
  given the executed trajectory and its outcome (reward/ target hit OR no reward/ target missed). The colormap hue represents the updated belief 
 that a given location is the hidden target location, red scatter dots shows the anchors sampled for the next run using a peak sampler. 

[comment]: <> (<video src="https://raw.githubusercontent.com/NadaYehia/Generative-Bayesian-learning-of-hidden-target/main/Demo/V1.mp4" controls width="640"></video>)

[![Demo Video Thumbnail](https://img.youtube.com/vi/9O7vQaQbwuo/0.jpg)](https://youtu.be/9O7vQaQbwuo)


2- A simulation showing how the agent detect the change in its environment and resets its prior to a flat distribution. 
The agent learnt target 1 pretty well and localized the control parameters that intercepts it, however it will reset its belief to a flat distribution when its surprise in the current outcome given the current posterior exceeds a given surprise threshold which coincides with the experimenter switching the rewarding target location (farthest left target). This reset enables the agent to learn new targets fast and maximize its long term rewards.

[![Demo Video Thumbnail](https://img.youtube.com/vi/raxOUuS0T1o/0.jpg)](https://youtu.be/raxOUuS0T1o)


## Dependencies
MATLAB (tested on version R2021a or later).

Parallel Computing Toolbox (for optimized and parallelized execution).

## Future Work
- Obstacle Handling: Extend the Environment class to include obstacles and their setup.

- Advanced Sampling Strategies: Implement additional sampling algorithms to further improve the agent's exploration and exploitation capabilities.

- Visualization Tools: Add more visualization options to better understand the agent's learning process and trajectory optimization.

## Contributing
We welcome contributions to this project! If you have suggestions, bug reports, or feature requests, please open an issue or submit a pull request.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

This README provides a comprehensive overview of the project, its components, and how to use it. Let me know if you need further refinements or additional sections!
