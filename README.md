Test Environments for the Minimum Constraint Removal (MCR) Problem in Motion Planning

This repository contains a collection of test environments designed to evaluate algorithms addressing the Minimum Constraint Removal (MCR) problem in robotic motion planning within constrained spaces. The test cases include both discrete and continuous formulations of the problem, enabling comparative analysis across planning paradigms.

Included Test Environments

Discrete Grid Environment
A grid-based representation where motion planning is performed using a Dijkstra-style search algorithm.
This environment provides a simple, fully observable setting for benchmarking discrete MCR formulations.

Continuous Environment (RRT)
A continuous configuration space where planning is carried out using a Rapidly-Exploring Random Tree (RRT) algorithm without collision constraints.
This serves as a baseline for evaluating unconstrained sampling-based methods.

Continuous Environment (MCR-RRT)
A custom implementation of an MCR-aware RRT planner that incorporates constraint removal costs into the exploration and path selection process.
This environment allows for assessing the trade-off between path optimality and constraint violation minimization.

Purpose

These environments are intended for testing and comparison of algorithms that balance path feasibility and constraint removal minimization in robotic motion planning tasks.