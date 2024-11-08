import numpy as np
def rotation_matrix(axis, theta):
    """
    Generate a 3D rotation matrix for a specified axis ('x', 'y', or 'z') and angle in radians.
    
    Parameters:
    axis (str): Axis of rotation ('x', 'y', or 'z')
    theta (float): Angle in radians
    
    Returns:
    numpy.ndarray: 3x3 rotation matrix
    """
    if axis == 'x':
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
    elif axis == 'y':
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
    elif axis == 'z':
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
"""
@params
x_offset: the physical distance in the x axis (the way baxter faces) of the camera in meters
y_offset: the physical distance in the y axis (the other horizontal axis) of the camera in meters
z_offset: the phsyical distance in the z axis (height) of the camera in meters
roll: the angular difference of the way the camera's facing in the axis baxter is facing, think tilting your head sideways
pitch: the angular difference of the camera in the x-axis (if I look down upon a child but still face forward, this is pitch)
yaw: the angular difference of the camera in the xy-plane (think shaking your head)
point: the 3D coordinate in meters as an array of floats, like [1, 1, 1.5]

ALL ANGLES IN RADIANS
ALL DISTANCES IN METERS

Returns an numpy array that is 3 by 1, xyz coordinates
"""
def transform_point(x_offset, y_offset, z_offset, roll, pitch, yaw, point):

    # look up roll, pitch, yaw to understand your camera configuration
    # z = 0, is at the bottom of the arm mounts, x=0,y=0  is at the handlebar in front of him
    # or use this link
    # https://www.google.com/url?sa=i&url=https%3A%2F%2Faircraftflightmechanics.com%2FEoMs%2FEulerTransforms.html&psig=AOvVaw3ZJbfoOXqBfpmIGZFbvHmM&ust=1731180476517000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCPiynvy7zYkDFQAAAAAdAAAAABAI

    r_y = rotation_matrix('y', pitch)
    r_z = rotation_matrix('z', yaw)
    r_x = rotation_matrix('x', roll)

    # = r_z * r_y * r_x

    r_zy = np.matmul(r_z, r_y)

    rot_mat = np.matmul(r_zy, r_x)

    t_mat = np.zeros(shape=(4,4))
    for i in range(3):
        for j in range(3):
            t_mat[i, j] = rot_mat[i, j]
    
    t_mat[3, 0] = x_offset
    t_mat[3, 1] = y_offset
    t_mat[3, 2] = z_offset
    t_mat[3, 3] = 1

    pt = np.zeros(shape=(4, ))
    pt[0] = point[0]
    pt[1] = point[1]
    pt[2] = point[2]
    pt[3] = 0

    new_point = np.matmul(t_mat, pt)

    ret_val = np.zeros(shape=(3,))
    ret_val[0] = new_point[0]
    ret_val[1] = new_point[1]
    ret_val[2] = new_point[2]

    return ret_val