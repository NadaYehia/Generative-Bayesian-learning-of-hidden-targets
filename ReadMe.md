Generative model for trajectory learning in a Bayesian framework

This repo contains the building blocks of our model mouse

  1- building the task environment and simulate an agent run.

  2- infer underlying control parameters and using it to generate likelihood for the Bayesian update.

  3- sample the next run actions from this posterior. The sampling function can extend 2 opposite ends from a non-linearity that find the maximum of the actions (a peak sampler) to the other end with a linear behavior (a proportional sampler).

  4-generate smooth trajectories that traverse these sampled anchors with appopriate noise to compenaste for non ideal aspects of the shrinking posterior.

  5- Capacity to cache previous successful policy parameterizations and how to sample from these cached posteriors before running a trajectory.
