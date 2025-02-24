Generative Model for Learning Hidden Rewarded Locations in a Square Arena Using Bayesian Inference
This repository contains a generative model for learning hidden rewarded locations in a square arena using Bayesian inference. The model simulates an agent that explores the arena, learns the locations of rewarded targets, and optimizes its trajectory to maximize rewards. The code is optimized and parallelized for fast execution.

## Overview
The program starts by running main.m, which orchestrates the simulation and learning process. Below is a detailed breakdown of the key components and their functionalities:

## 1. Environment Class
The Environment class defines the physical properties of the arena and the target locations. It includes the following attributes:

- **Arena Size**: The dimensions of the square arena.

- **Target Locations & Sizes** : The coordinates and sizes of the rewarded targets.

- **Blocks**: The number of trials per target location.

 ### Methods
- get_target_corners: A static method that takes the target centers and dimensions as input and outputs their respective corner coordinates. This is useful for visualization and collision detection.

- Future Plans: We plan to extend this class to handle obstacles and their setup in the environment.

## 2. Set Control Actions Space
This module defines the action space for the agent, which consists of possible speeds and angles. The action space is constrained by the physical limits of the arena, ensuring that the agent's movements are realistic and feasible.

## 3. Simulate Run
The simulate_run function simulates the agent's movement along a given trajectory and observes the outcome at the home port. It determines whether the agent successfully intercepts the target and receives a reward.

## 4. Surprise
The surprise function detects changes in the environment, such as shifts in target locations. If the observed outcome is too surprising (i.e., it deviates significantly from the agent's expectations), the agent resets its prior belief to a uniform distribution. This allows the agent to adapt to unexpected changes in the environment.

## 5. Bayes' Update for Action Parameters
This function updates the agent's belief about the rewarding locations using Bayesian inference. It works as follows:

1- Likelihood Function: The likelihood of the target location is estimated by convolving the executed trajectory (in polar coordinates) with a 2D Gaussian kernel. The sigma parameter of the Gaussian represents the agent's belief about the size of the rewarding location and the consistency of the outcome around the locations it has explored.

2- Posterior Update: The likelihood function is used to update the agent's posterior belief about the rewarding locations. This posterior is then used to guide the agent's future actions.

## 6. Sampling Next Actions
The Sampling_next_actions function represents the agent's strategy for selecting the next set of actions after updating its posterior belief. Two sampling algorithms are implemented:

1- Peak Sampler: Selects actions with the highest local reward probability. This strategy focuses on exploiting the most promising areas.

2- Proportional Sampler: Selects actions proportional to their reward probability in the posterior. This strategy balances exploration and exploitation.

These two algorithms represent extremes of a continuum, allowing the agent to adapt its sampling strategy based on the task requirements.

## 7. Trajectory Planner
The Trajectory_planner function is the core of the agent's generative model. It takes a list of anchors (sampled by the Sampling_next_actions function) as input and optimizes the path between them. The optimization process minimizes the path length and ensures smoothness.

**Key Steps**:\
1- Anchor Optimization: The function computes the optimal heading and distances for the set of anchors.

2- Path Generation: It generates a smooth trajectory connecting the optimized anchors.

3- Coordinate Transformation: The function samples {x, y} points along the path and calculates their polar coordinates {r, theta}.

This module ensures that the agent's movements are efficient and natural.

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
 
 1- A simulation of the model learning a target. Left window shows the executed trajectories and right window shows the posterior 
 updated given the executed trajectory and the outcome (reward/ target hit OR no reward/ target missed). The color hue represents the probability 
 of a given location being the hidden target location, red scatter dots shows the anchors sampled for the next run using a peak sampler. 

[comment]: <> <video src="https://raw.githubusercontent.com/NadaYehia/Generative-Bayesian-learning-of-hidden-target/main/Demo/V1.mp4" controls width="640"></video>
2- 

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
