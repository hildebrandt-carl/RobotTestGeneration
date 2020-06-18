# Feasible and Stressful Trajectory Generation for Mobile Robots

This repository contains the code used for the implementation of our technique for generating physically feasible yet stressful trajectories. More information on how to use this software can be found on [this website](https://hildebrandt-carl.github.io/RobotTestGenerationArtifact/). This project is licensed under the terms of the MIT license.

## Paper Abstract

While executing nominal tests on mobile robots is required for their validation, such tests may overlook faults that arise under trajectories that accentuate certain aspects of the robot's behavior. Uncovering such stressful trajectories is challenging as the input space for these systems as they move is extremely large, and the relation between a planned trajectory and its potential to induce stress can be subtle. To address this challenge we propose a framework that 1) integrates kinematic and dynamic physical models of the robot into the automated trajectory generation in order to generate valid trajectories, and 2) incorporates a parameterizable scoring model to efficiently generate physically valid yet stressful trajectories for a broad range of mobile robots. We evaluate our approach on four variants of a state-of-the-art quadrotor under a racing simulator. We find that, for non-trivial length trajectories, the incorporation of the kinematic and dynamic model is crucial to generate any valid trajectory, and that the approach with the best hand-crafted scoring model and with a trained scoring model can cause on average approximately 56% and 41% more stress than a random selection of trajectories among the valid ones. A follow-up study shows that the approach was able to induce similar stress on a deployed commercial quadrotor, with trajectories that deviated up to 6m from the intended ones. 

## Quick Start Guide

This repository contains the full working software. The full software can be complex to setup as there are many dependencies (refer to [workflow.md](./WORKFLOW.md)). We recommend starting using [this Virtual Machine](https://www.dropbox.com/s/za7x3c3z132smc7/ISSTA_2020_Artifact.ova?dl=1) for a quick start test. The VM contains a full working installed subset of this software.

## Navigating this repository

A detailed description of how to replicate the results is presented in [workflow.md](./WORKFLOW.md). The repository is broken up into 4 main software modules:

* [Anafi Simuation](./AnafiSimulation/) - Contains the code used in testing the Anafi quadrotor.
* [Test Generation](./TestGeneration/) - Contains the code used to generate feasible and stressful tests.
* [Unity](./Unity/) - It contains the source code for the redesigned unity simulator.
* [World Engine Simulation](./WorldEngineSimulation/) - Contains the FlightGoggles source code with custom controllers and scripts to launch the simulators.

## Viewing the results

The data can be viewed in two ways. You can view the final results or you can view the raw data.

* [Final Results](./RobotTestGenerationFinalResults/) - Contains the figures used in the paper.
* [Raw Data](./TestGeneration/FinalResults/) - Contains the raw results.
* [Raw Models](./TestGeneration/FinalModels/) - Contains the raw learned scoring models that were used saved as numpy files.

## Authors:

Carl Hildebrandt, Sebastian Elbaum, Nicola Bezzo, Matthew B. Dwyer
