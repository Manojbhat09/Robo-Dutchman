#!/usr/bin/env python

import math
import numpy as np

# All units in meters
L1 = 0.381;
L2 = 0.3302;
L3 = 0.130175;

z_offset = 0.06668;


# =============================================================================
# INVERSE KINEMATICS
# =============================================================================

def ik(work, elbow_up = False):
    target_x = work[0]
    target_y = work[1]
    target_z = work[2]
    target_theta = work[3]
    target_wrist = work[4]

    config = [0, 0, 0, 0]

    # Place arm 3 down first, find center of wrist joint
    wrist_center_x = target_x - (L3 * math.cos(target_theta))
    wrist_center_y = target_y - (L3 * math.sin(target_theta))

    # Elbow
    arg1 = ( (wrist_center_x*wrist_center_x) + (wrist_center_y*wrist_center_y) - ((L1*L1) + (L2*L2))) / (2.0 * L1 * L2)
    config[1] = math.acos(arg1)

    # Shoulder
    config[0] = math.atan2( wrist_center_y, wrist_center_x) \
            - math.atan2( L2 * math.sin(config[1]), \
            (L1 + L2 * math.cos(config[1])))

    # Change elbow up or down
    if (elbow_up == get_elbow(config)):
            angle_to_wrist_center = math.atan2(wrist_center_y,wrist_center_x)
            diff_wrist_angle_to_shoulder = angle_to_wrist_center - config[0]
            config[0] += 2 * diff_wrist_angle_to_shoulder
            config[1] = -config[1]


    # Wrist 1
    config[2] = target_theta - config[0] - config[1]

    # Wrist 2
    config[3] = target_wrist

    # Second motor (elbow) has flipped axis
    config[1] = -config[1]

    for i in range(0,4):
        if isinstance(config[i],complex):
            return None

    return config

def get_elbow(config):
    if (config[1] > 0):
        return True
    return False
# =============================================================================
# FORWARD KINEMATICS
# =============================================================================
#

# Does forward kinematics
# The configuration space is a 1 d vector of length 4
# Its elements are the shoulder, elbow, wrist1, wrist2, respectively
# The output is a 1d vector of length 5
# Its elements are [x y z theta1 wrist2]


def fk(config):
    dhps = np.zeros((5,4), dtype = float)
    dhps[0,:] = [z_offset, config[0], L1, np.pi]
    dhps[1,:] = [0, config[1], L2, np.pi]
    dhps[2,:] = [0, config[2], 0, 0]
    dhps[3,:] = [0, np.pi/2, 0, np.pi/2]
    dhps[4,:] = [L3, config[3], 0, 0]

    ee = compose_dh_transforms(dhps)
    workspace = [ee[0,3], ee[1,3], ee[2,3], config[0] - config[1] + config[2], config[3]]
    return workspace

# Creates a size by size identity matrix
def eye(size):
    rv = np.zeros((size,size))
    for i in range(0,size):
        rv[i,i] = 1
    return rv

# Expects an a rray of dhps
# each row is a dhp
def compose_dh_transforms(dhp):
    num_dhp = dhp.shape[0]
    H = eye(4)
    for i in range (0,num_dhp):
        H = H.dot(make_dh_transform(dhp[i,0:4]))
    return H

# Turns a vector of dh params into a 4x4 homogeanous tranform
def make_dh_transform(dhp):
    d = dhp[0]
    theta = dhp[1]
    a = dhp[2]
    alpha = dhp[3]

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    transform = np.zeros((4,4))
    transform[0,0] =  cos_theta
    transform[0,1] = -sin_theta*cos_alpha
    transform[0,2] =  sin_theta*sin_alpha
    transform[0,3] =  cos_theta * a

    transform[1,0] =  sin_theta
    transform[1,1] =  cos_theta*cos_alpha
    transform[1,2] = -cos_theta*sin_alpha
    transform[1,3] =  sin_theta * a

    transform[2,1] =  sin_alpha
    transform[2,2] =  cos_alpha
    transform[2,3] =  d

    transform[3,3] = 1

    return transform


