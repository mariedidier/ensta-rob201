""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1
    speed = 1.0
    val=lidar.get_sensor_values()[180]
    print(val)
    rotation_speed = 0.0
    if val<100:
        speed=-1.0
        rotation_speed = 1.0
        
    

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2
    d = np.sqrt((goal_pose[0]-current_pose[0])**2 +(goal_pose[1]-current_pose[1])**2)
    K=1
    gradient=(K/d)*(goal_pose-current_pose)
    speed=0.0
    rotation_speed=0

    if d > 1:  # Threshold distance for stopping
        # Compute angle between current orientation and gradient direction
        theta = current_pose[2]
        angle_to_gradient = np.arctan2(gradient[1], gradient[0]) - theta
        
        # Limit angle to be between -pi and pi
        if angle_to_gradient > np.pi:
            angle_to_gradient -= 2 * np.pi
        elif angle_to_gradient < -np.pi:
            angle_to_gradient += 2 * np.pi
        
        # Define rotation speed proportional to the angle to the gradient
        rotation_speed = 1.0 * angle_to_gradient
        
        # Limit rotation speed
        if rotation_speed > 1.0:
            rotation_speed = 1.0
        elif rotation_speed < -1.0:
            rotation_speed = -1.0
            
        # Adjust linear speed based on the distance to the goal
        speed = min(0.5, d)  # Limit the speed to 0.5
    command = {"forward": speed,
               "rotation": rotation_speed}

    return command    
"""
    if grad[2]<0:
        rotation_speed=0.2
    elif grad[2]>0:
        rotation_speed=-0.2
    if d<=50:
        speed=0
        rotation_speed=0
    """
        
