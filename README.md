# Dynamic Obstacle Avoidance Package

## Overview

This package is designed to calculate a force vector that guides a robot to its goal while avoiding both dynamic and static obstacles. The system requires the following inputs:

- **Robot**: Position and velocity based on an inertial frame.
- **Obstacles**: Position, velocity, and radius based on an inertial frame.
- **Goal**: Position based on an inertial frame.

With this information, the package computes a force vector pointing in the direction the robot needs to move to avoid collisions and reach its goal.

## Parameters

Before running the nodes, you need to define the following parameters. Below are the parameters with their default values:

```yaml
attraction_scaling_factor: 6000
obstacle_scaling_factor_dynamic: 20000
obstacle_scaling_factor_static: 3000000
scaling_factor_emergency: 40000
safety_margin_radius: 0.3
robot_domain_radius: 0.5
safe_distance: 1.0
obstacle_influence_range: 5
distance_to_goal: 0.1

# Topics
## Input Topics

These topics are used to provide the necessary data to the system:

    /scenario/output_robot: RobotState message (Robot's position and velocity)
    /scenario/output_obstacles: ObstacleArray message (Obstacles' position, velocity, and radius)
    /scenario/goal: Vector3 message (Goal position)

## Output Topics

The package will publish the computed forces on the following topics:

    /apfm/repulsive_force: Vector3 (Repulsive force)
    /apfm/attractive_force: Vector3 (Attractive force)
    /apfm/total_force: Vector3 (Total force)
    /apfm/dynamic_force: Vector3 (Dynamic obstacle avoidance force)
    /apfm/static_force: Vector3 (Static obstacle avoidance force)
    /apfm/emergency_force: Vector3 (Emergency avoidance force)

## Debugging Topic

For debugging purposes, the following topic is used to provide detailed information about the obstacle avoidance calculations:

    /obstacle_avoidance/custom_info: CustomInfo message
