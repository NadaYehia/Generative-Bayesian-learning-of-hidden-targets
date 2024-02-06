Generative model for trajectory learning in a Bayesian framework

This repo contains the building blocks of our model mouse

  - building the task environment, simulate an agent run, build the control actions space and bound it by the physical size of the arena.

  - infer underlying control parameters and using it to generate likelihood for the Bayesian update.

  - sample the next run actions from this posterior. The sampling function can extend 2 opposite ends, from a non-linearity that find the maximum of the actions (a peak sampler) to the other end with a linear behavior (a proportional sampler).

  -generate smooth trajectories that traverse these sampled anchors with appropriate noise to compensate for non ideal aspects of the shrinking posterior.

  - Capacity to cache previous successful policy parameterizations and how to sample from these cached posteriors before running a trajectory.

the program starts with running **main.m** which comprises of the following 
  1) **Environment** class that has the attributes of the physical arena size and target locations & sizes, mode of interception to get a reward **intercept mode** and number of trials per target location 
     **blocks**. This class has static method that takes the target centeres and dimensions and outputs their respective corner coordinates. We envision to add further methods in this class that would handle 
     obstacles and its setup in the environment.
  2) **set control actions space** which constraint the action space (possible speeds and angles) given the physical limits of the arena.
  3) **connect actions with a smooth trajectory** this function is the core function for the agent generative model. It does the mapping of anchors from the action space to the x,y space, connect these anchors 
     coordinates using the agent generative model and finally maps back every x,y point on this trajectory to the action space using **convert xy to velocity and angle**; that is the necessary speed and heading 
     scale the agent will need to run from the port location (0,0) to each of these points.
4)  **Baye's update for action parameters** this function uses the action parameters of the generated trajectory in (3) to estimate the likelihood function in case of observing a reward after running this 
     trajectory. 
