""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1
    speed = 0.5
    val=lidar.get_sensor_values()[135:225]
    min_val=min(val)
    print(min_val)
    rotation_speed = 0.0
    if min_val<50:
        speed=-0.5
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
    d_to_goal = np.linalg.norm (current_pose[0:2]-goal_pose[0:2])
    print(d_to_goal)


    K_goal=0.5
    d_goal_reached = 20

    # Gradient to goal 
    if d_to_goal > d_goal_reached:  
        gradient_goal=(K_goal/d_to_goal)*(goal_pose-current_pose)
    else :
        gradient_goal=(K_goal/d_goal_reached)*(goal_pose-current_pose)
        
    
    #gradient from obstacle
    d_to_obs,ang_to_obs=min(zip(lidar.get_sensor_values(),lidar.get_ray_angles()))
    
    obs_x=d_to_obs*np.cos(ang_to_obs)
    obs_y=d_to_obs*np.sin(ang_to_obs)
    obs_pose = [obs_x,obs_y,ang_to_obs]

    print(obs_pose)
    d_crash = 50
    K_obs = 1000
    if d_to_obs < d_crash :
        gradient_obstacle = K_obs /d_to_obs**3 * (1/d_to_obs - 1/d_crash) * (obs_pose - current_pose)
    else :
        gradient_obstacle = 0


    gradient = gradient_goal - gradient_obstacle
    #print(gradient_obstacle)

    #The closer the obstacle, the stronger the rotation
    alpha= 1 #20*d_crash/d_to_obs**2

    speed = max(-0.9999,min(0.999999,np.linalg.norm(gradient[0:2])))
    #print(speed)
    rotation_speed = max(-0.99999,min(0.99999,alpha*(gradient[2]-current_pose[2])))
    #print(rotation_speed)

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command    

        
