"""
Dolphin SLAM ROS2 Nodes
=======================

This package contains the ROS2 node implementations for Dolphin SLAM.

Available Nodes:
----------------
- image_processing_node: Processes camera and sonar images
- local_view_node: Manages visual templates and scene recognition
- place_cell_node: Implements the place cell network
- experience_map_node: Builds and maintains the topological map
- robot_state_node: Fuses sensor data and estimates robot state
- dataset_player_node: Plays back AUV dataset for testing

Each node can be run using:
    ros2 run dolphin_slam <node_name>
"""

# This file can be empty, but we include the docstring for documentation
