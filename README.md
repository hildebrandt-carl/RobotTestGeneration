# Feasible and Stressful Trajectory Generation for Mobile Robots

This reposity contains the code used for the implemenation of our technique for generating physically feasible yet stressful trajectories. This repository is [available annoynomously](https://anonymous.4open.science/r/02a4e2a7-8986-47f7-a237-343535e897b2/).

**Note**: There are a list words that have been removed from the annoysmous reposity. These words will be replaced with XXX in the annoynomous reposity.

## Paper Abstract

While executing nominal tests on mobile robots is required for their validation, such tests may overlook faults that arise under trajectories that accentuate certain aspects of the robot's behavior. Uncovering such stressful trajectories is challenging as the input space for these systems as they move is extremely large, and the relation between a planned trajectory and its potential to induce stress can be subtle. To address this challenge we propose a framework that 1) integrates kinematic and dynamic physical models of the robot into the automated trajectory generation in order to generate valid trajectories, and 2) incorporates a parameterizable scoring model to efficiently generate physically valid yet stressful trajectories for a broad range of mobile robots. We evaluate our approach on four variants of a state-of-the-art quadrotor under a racing simulator. We find that, for non-trivial length trajectories, the incorporation of the kinematic and dynamic model is crucial to generate any valid trajectory, and that the approach with the best hand-crafted scoring model and with a trained scoring model can cause on average approximately $56\%$ and $41\%$ more stress than a random selection of trajectories among the valid ones. A follow-up study shows that the approach was able to induce similar stress on a deployed commercial quadrotor, with trajectories that deviated up to $6$m from the intended ones. 

## Navigating this repository

A detailed description of how to replicate the results is presented in [workflow.md](./WORKFLOW.md). The reposity is broken up into 4 main software modules:

* [Anafi Simuation](./AnafiSimulation/) - Contains the code used in testing the Anafi quadrotor.
* [Test Generation](./TestGeneration/) - Contains the code used to generate feasible and stressful tests.
* [Unity](./Unity/) - Contains the source code for the redesigned unity simulator.
* [World Engine Simulation](./WorldEngineSimulation/) - Contains the FlightGoggles source code with custom controllers and scripts to launch the simulators.

## Viewing the results

The data can be viewed in two ways. Either the you can view the final results or you can view the raw data.

* [Final Results](./RobotTestGenerationFinalResults/) - Contains the figures used in the paper (and some that werent).
* [Raw Data](./TestGeneration/FinalResults/) - Contains the raw results.
* [Raw Models](./TestGeneration/FinalModels/) - Contains the raw learned scoring models that were used saved as numpy files.

# Developed By:

XXX