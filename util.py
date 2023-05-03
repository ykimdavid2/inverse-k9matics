import math
import numpy as np
# import matplotlib.pyplot as plt

def leg_ik(x, y, l1, l2):
    """
    Calculates the inverse kinematics of a 2 DOF robot leg given the desired x and y coordinates of the foot
    relative to the shoulder joint and the lengths of the two segments of the leg.
    
    Arguments:
    x -- desired x-coordinate of the foot relative to the shoulder joint
    y -- desired y-coordinate of the foot relative to the shoulder joint
    l1 -- length of the first segment of the leg
    l2 -- length of the second segment of the leg
    
    Returns:
    Tuple containing the angles of the two joints in radians
    """
    
    # calculate the length of the line between the shoulder joint and the foot
    l3 = math.sqrt(x**2 + y**2)
    
    # calculate the angle between the line connecting the shoulder joint and the foot and the horizontal axis
    alpha = math.atan2(y, x)
    
    # calculate the angle between the line connecting the shoulder joint and the foot and the line connecting
    # the shoulder joint and the end of the first segment of the leg
    beta = math.acos((l1**2 + l3**2 - l2**2) / (2 * l1 * l3))
    
    # calculate the angle between the line connecting the shoulder joint and the end of the first segment of the leg
    # and the horizontal axis
    gamma = math.acos((l1**2 + l2**2 - l3**2) / (2 * l1 * l2))
    
    # calculate the angle of the first joint
    theta1 = alpha - beta
    theta1 = 2 * (math.pi - theta1) + theta1
    
    # calculate the angle of the second joint
    theta2 = math.pi - gamma
    theta2 = 2 * (math.pi - theta2) + theta2
    
    return (theta1, theta2)

def foot_path(t, length=0.2, body_height = 0.3, gait_height = 0.05):
    # t in range [0, 1]
    # swing portion of code
    t = t % 1
    # t = 1 - t
    if t <= 0.5:
        t_temp = t * 2 # rescaling for swing portion
        x = ((t_temp) * length) - length/2
        y = math.cos(x) * gait_height
        y = (body_height - y)
    else:
        # contact with floor
        t_temp = (t - 0.5) * 2
        x = ((1-t_temp) * length) - length/2
        y = body_height

    ## DEBUG
    # angle = 2 * math.pi * t
    # radius = 0.1
    # x = radius * math.cos(angle)
    # y = body_height - radius * math.sin(angle)
    return y, x


