#!/usr/bin/env python3

import numpy as np
import math

class ObstacleAvoidance:
    def __init__(self, params):
        self.attraction_scaling_factor = params['attraction_scaling_factor']
        self.obstacle_scaling_factor_dynamic = params['obstacle_scaling_factor_dynamic']
        self.obstacle_scaling_factor_static = params['obstacle_scaling_factor_static']
        self.scaling_factor_emergency = params['scaling_factor_emergency']
        self.safety_margin_radius = params['safety_margin_radius']
        self.robot_domain_radius = params['robot_domain_radius']
        self.safe_distance = params['safe_distance']
        self.obstacle_influence_range = params['obstacle_influence_range']
        self.distance_to_goal = params['distance_to_goal']

    def modified_attractive_force(self, pos, vector_to_goal, distance_to_goal):
        return self.attraction_scaling_factor * distance_to_goal * (vector_to_goal / np.linalg.norm(vector_to_goal))

    def calculate_Fre(self, distance_to_obstacle, center_to_center_safe_distance, distance_to_goal, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal, perpendicular_unit_vector_to_obstacle):
        var1 = (1./(distance_to_obstacle - self.safety_margin_radius) - 1/center_to_center_safe_distance)
        var2 = (distance_to_goal**2)/((distance_to_obstacle - self.safety_margin_radius)**2)
        Fre1 = -2 * self.scaling_factor_emergency * self.robot_domain_radius * var1 * var2 * unit_vector_to_obstacle
        Fre2 = 2 * self.scaling_factor_emergency * self.robot_domain_radius * distance_to_goal / distance_to_obstacle * np.linalg.norm(relative_speed_vector)**2 * (np.cos(angle_between_direction_and_velocity) * np.sin(angle_between_direction_and_velocity)) * perpendicular_unit_vector_to_obstacle
        Fre3 = 2 * self.scaling_factor_emergency * self.robot_domain_radius * distance_to_goal * (var1**2 + np.linalg.norm(relative_speed_vector)**2 * np.cos(angle_between_direction_and_velocity)**2) * normalized_vector_to_goal
        return Fre1 + Fre2 + Fre3

    def calculate_Frs(self, distance_to_obstacle, distance_to_goal, unit_vector_to_obstacle, normalized_vector_to_goal):
        var1 = 1/(distance_to_obstacle - self.safety_margin_radius) - 1/self.obstacle_influence_range
        var2 = (distance_to_goal**2)/(distance_to_obstacle**2)
        Frs1 = -self.obstacle_scaling_factor_static * self.robot_domain_radius * var1 * var2 * unit_vector_to_obstacle
        Frs3 = self.obstacle_scaling_factor_static * self.robot_domain_radius * distance_to_goal * var1**2 * normalized_vector_to_goal
        return Frs1 + Frs3  

    def calculate_Frd(self, distance_to_obstacle, center_to_center_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal, perpendicular_unit_vector_to_obstacle):
        var1 = (1./(distance_to_obstacle - center_to_center_safe_distance)) - (1/self.obstacle_influence_range)
        var2 = (center_to_center_safe_distance / (distance_to_obstacle * np.sqrt(distance_to_obstacle**2 - center_to_center_safe_distance**2)))
        var5 = ((np.exp(angle_between_direction_and_velocity) - 1)/((distance_to_obstacle - center_to_center_safe_distance)**2))
        Frd1 = -self.obstacle_scaling_factor_dynamic * self.robot_domain_radius * self.distance_to_goal**2 * (var1 * np.exp(angle_between_direction_and_velocity) * var2 + var5) * normalized_vector_to_goal
        Frd2 = -self.obstacle_scaling_factor_dynamic * self.robot_domain_radius * self.distance_to_goal**2 * (var1 * np.exp(angle_between_direction_and_velocity)) * perpendicular_unit_vector_to_obstacle
        return Frd1 + Frd2

    def modified_potential_field(self, goal_position, obstacle_positions, obstacle_radii, obstacle_velocities, current_position, current_velocity):
        vector_to_goal = goal_position - current_position
        distance_to_goal = np.linalg.norm(vector_to_goal)
        normalized_vector_to_goal = vector_to_goal / distance_to_goal

        attractive_force = self.modified_attractive_force(current_position, vector_to_goal, distance_to_goal)
        repulsive_force = np.zeros_like(current_position)

        for i in range(len(obstacle_positions)):
            distance_to_obstacle = np.linalg.norm(obstacle_positions[i] - current_position)
            center_to_center_safe_distance = self.robot_domain_radius + self.safe_distance + obstacle_radii[i]
            unit_vector_to_obstacle = (obstacle_positions[i] - current_position) / distance_to_obstacle
            relative_speed_vector = current_velocity - obstacle_velocities[i]
            angle_between_direction_and_velocity = np.degrees(np.arccos(np.dot(unit_vector_to_obstacle, relative_speed_vector) / (np.linalg.norm(unit_vector_to_obstacle) * np.linalg.norm(relative_speed_vector))))
            perpendicular_unit_vector_to_obstacle = np.array([-unit_vector_to_obstacle[1], unit_vector_to_obstacle[0]])

            if distance_to_obstacle <= center_to_center_safe_distance:
                Fre = self.calculate_Fre(distance_to_obstacle, center_to_center_safe_distance, distance_to_goal, unit_vector_to_obstacle, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal, perpendicular_unit_vector_to_obstacle)
                Frd = self.calculate_Frd(distance_to_obstacle, center_to_center_safe_distance, relative_speed_vector, angle_between_direction_and_velocity, normalized_vector_to_goal, perpendicular_unit_vector_to_obstacle)
                Frs = self.calculate_Frs(distance_to_obstacle, distance_to_goal, unit_vector_to_obstacle, normalized_vector_to_goal)
                repulsive_force += Fre + Frd + Frs

        total_force = attractive_force + repulsive_force
        return total_force
