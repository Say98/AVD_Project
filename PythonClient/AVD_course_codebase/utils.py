import numpy as np
import transforms3d
from math import sin, cos, pi, tan, sqrt

def rotate_x(angle):
    R = np.mat([[ 1,         0,           0],
                 [ 0, cos(angle), -sin(angle) ],
                 [ 0, sin(angle),  cos(angle) ]])
    return R

def rotate_y(angle):
    R = np.mat([[ cos(angle), 0,  sin(angle) ],
                 [ 0,         1,          0 ],
                 [-sin(angle), 0,  cos(angle) ]])
    return R

def rotate_z(angle):
    R = np.mat([[ cos(angle), -sin(angle), 0 ],
                 [ sin(angle),  cos(angle), 0 ],
                 [         0,          0, 1 ]])
    return R
    
def to_rot(r):
    Rx = np.mat([[ 1,         0,           0],
                 [ 0, cos(r[0]), -sin(r[0]) ],
                 [ 0, sin(r[0]),  cos(r[0]) ]])

    Ry = np.mat([[ cos(r[1]), 0,  sin(r[1]) ],
                 [ 0,         1,          0 ],
                 [-sin(r[1]), 0,  cos(r[1]) ]])

    Rz = np.mat([[ cos(r[2]), -sin(r[2]), 0 ],
                 [ sin(r[2]),  cos(r[2]), 0 ],
                 [         0,          0, 1 ]])

    return Rz*Ry*Rx
def transform_world_to_ego_frame(pos, ego, ego_rpy):
    """
    Transforms a position expressed in world frame into vehicle frame. 
    Args:
        pos (x,y,z): Position to transform
        ego (x, y, yaw, current_speed): current state of the vehicle
        ego_rpy (current_roll, current_pitch, current_yaw): current orientation of the vehicle
    Returns:
        loc_relative: position (x,y,z) expressed in vehicle frame
    """
    loc = np.array(pos) - np.array(ego)
    r = transforms3d.euler.euler2mat(ego_rpy[0], ego_rpy[1], ego_rpy[2]).T
    loc_relative = np.dot(r, loc)
    return loc_relative

def compute_angle_diff(a, b):
    """Given two angles, returns the difference in the range [-180, 180]
    Args:
        a ([float]): angle in degree
        b ([float]): angle in degree
    Returns:
        [float]: angle difference in the range [-180,180]
    """
    d = b - a
    if d > 180:
        d -= 360
    if d < -180:
        d += 360
    return d

def transform_to_matrix(transform):
    """This function create a transformation matrix starting from the Transform object of Carla agent 
    Args:
        transform ([Transform]): Transform object of Carla agent containing location and orientation info
    Returns:
        matrix ([np.array]): transformation matrix created
    """
    rotation = transform.rotation
    rotation = np.deg2rad([rotation.roll, rotation.pitch, rotation.yaw])
    location = transform.location
    rotation_matrix = transforms3d.euler.euler2mat(rotation[0], rotation[1], rotation[2])
    matrix = np.append(rotation_matrix, [[location.x], [location.y], [location.z]], axis=1)
    matrix = np.vstack([matrix, [0, 0, 0, 1]])
    return matrix

def translate_position(entity, offset):
    """Returns global frame coordinates of the position for entity translated by offset on the x-axis
    Args:
        entity: Carla agent for which next position has to be translated
        offset (float):  offset on the x-axis of which translate the entity
    Returns:
        [x,y,z]: translated position of the entity
    """
    return (transform_to_matrix(entity.transform) @ np.array([offset, 0, 0, 1]))[:-1]

def line_intersection(line1, line2):
    """
    Returns True if line1 and line2 intersect, False otherwise
    """

    def ccw(x, y, z):
        return (z[1] - x[1]) * (y[0] - x[0]) > (y[1] - x[1]) * (z[0] - x[0])

    a = line1[0]
    b = line1[1]
    c = line2[0]
    d = line2[1]
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)

def estimate_next_entity_pos(entity, speed=None):
    """Returns global frame coordinates of next position for entity moving at a certain speed.
    Args:
        entity: Carla agent for which next position has to be estimated
        speed (float, optional): speed to be used for estimation. Defaults to None, that is current speed of the entity.
    Returns:
        [x,y,z]: estimated next position of the entity
    """
    if speed is None: speed = entity.forward_speed
    return translate_position(entity, speed)

def check_obstacle_future_intersection(entity, dist, loc_relative, ego_state, ego_rpy, speed=None):
    """Check if the estimated trajectory of an entity intersect ego trajectory
    Args:
        entity: Carla agent for which compute trajectory
        dist (float): projection on the x-axis of ego position in the ego frame
        loc_relative ([x,y,z]): location of the entity in the ego frame
        ego_state ([current_x, current_y, current_z]): current location of ego vehicle in the global frame
        ego_rpy ([current_roll, current_pitch, current_yaw]): current orientation of ego vehicle in the global frame
        speed ([float], optional): speed of the entity. Defaults to None, that is current speed.
    Returns:
        [bool]: if entity estimated trajectory intersects ego trajectory
    """
    # ego
    a = (0, 0) # start
    b = (dist,0) # finish

    # entity
    c = transform_world_to_ego_frame(
        translate_position(entity, -5),
        ego_state,
        ego_rpy
    )[:-1] # start
    d = transform_world_to_ego_frame(
        estimate_next_entity_pos(entity, speed = speed),
        ego_state,
        ego_rpy
    )[:-1] # finish

    return line_intersection((a, b), (c, d))

