Generative model for trajectory learning in a Bayesian framework

This repo contains the building blocks of our model mouse

  - building the task environment, simulate an agent run, build the control actions space and bound it by the physical size of the arena.

  - infer underlying control parameters and using it to generate likelihood for the Bayesian update.

  - sample the next run actions from this posterior. The sampling function can extend 2 opposite ends, from a non-linearity that find the maximum of the actions (a peak sampler) to the other end with a linear behavior (a proportional sampler).

  -generate smooth trajectories that traverse these sampled anchors with appropriate noise to compensate for non ideal aspects of the shrinking posterior.

  - Capacity to cache previous successful policy parameterizations and how to sample from these cached posteriors before running a trajectory.

the program starts with running **main.m** which calls 
  1) **env_settings** to set the physical arena size and target locations & sizes. In addition it initialize some agent hyperparamteres like : the uncertanity in the trajectory parameters **sigma ridge**, mode of interception to get a reward **intercept mode** and number of trials per target location **blocks**
  2) **set control actions space** which constraint the action space (possible speeds and angles) given the physical limits of the arena.
  3) **run random agent trial on a target** which compromises of the core blocks of the agent: **action sampler** (proportional or peak), **the generative model to connect these actions anchors in xy space**, 
  **compute the empirical likelihood** of this trajectory by finding the action parameters that peak at every point along its course, and lastly the **Bayesian update** of the action space posterior given the 
  outcome of this trajectory (reward or no reward) and our model likelihood. 
