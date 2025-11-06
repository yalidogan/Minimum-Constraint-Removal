Here is an updated `README.md` that includes a new section for the `MCDSamplingPlanner` algorithm you just implemented.

---

# Test Environments for the Minimum Constraint Removal (MCR) Problem in Motion Planning

This repository contains a collection of test environments designed to evaluate algorithms addressing the Minimum Constraint Removal (MCR) problem in robotic motion planning within constrained spaces. The test cases include both discrete and continuous formulations of the problem, enabling comparative analysis across planning paradigms.

## Included Test Environments

### Discrete Grid Environment
A grid-based representation where motion planning is performed using a Dijkstra-style search algorithm. This environment provides a simple, fully observable setting for benchmarking discrete MCR formulations.

### Continuous Environment (RRT)
A continuous configuration space where planning is carried out using a Rapidly-Exploring Random Tree (RRT) algorithm without collision constraints. This serves as a baseline for evaluating unconstrained sampling-based methods.

### Continuous Environment (MCR-RRT)
A custom implementation of an MCR-aware RRT planner that incorporates constraint removal costs into the exploration and path selection process. This environment allows for assessing the trade-off between path optimality and constraint violation minimization (e.g., via a weighted-sum cost function).

### Continuous Environment (MCD Sampling Planner)
This environment implements a three-phase **Minimum Constraint Displacement (MCD)** sampling-based planner. This algorithm is designed to find an optimal path by explicitly balancing path length against the fixed cost of *displacing* (or removing) soft constraints.

It operates in three distinct phases:

1.  **Phase 1: Roadmap Expansion (RRT)**

    A Rapidly-Exploring Random Tree (RRT) is built in the continuous space. This RRT *avoids* all hard constraints but *ignores* soft constraints, building a "draft" map of all physically possible routes.

2.  **Phase 2: Displacement Sampling (Unblocking Strategy)**

    As the RRT builds, any time a new branch crosses a soft constraint, the algorithm "samples" a "remove" displacement for that constraint. This adds the *option* to remove that specific constraint to a master list (marking it as "removable" for a fixed cost), but doesn't commit to removing it yet.

3.  **Phase 3: Discrete MCD Solver**

    After the roadmap is built, a final A* search is executed *on the RRT roadmap* (not the grid). This search finds the optimal path from start to goal by calculating the true cost J of each edge:
    J = (Path Length) + (Sum of Penalties for all removed constraints on that edge)
    An edge in this search is only considered passable if all soft constraints blocking it were successfully "sampled" (marked as "removable") in Phase 2.

This approach effectively separates the problem of *exploration* (Phase 1) from *optimization* (Phase 3), allowing it to solve for the optimal set of constraint removals and the shortest corresponding path.

## Purpose

These environments are intended for testing and comparison of algorithms that balance path feasibility and constraint removal minimization in robotic motion planning tasks.