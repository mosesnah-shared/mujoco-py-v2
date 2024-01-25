import numpy as np

def rotx( q ):
    Rx = np.array( [ [ 1,            0,            0 ], 
                     [ 0,  np.cos( q ), -np.sin( q ) ],
                     [ 0,  np.sin( q ),  np.cos( q ) ]  ] )

    return Rx

def roty( q ):
    Ry = np.array( [ [  np.cos( q ),  0,  np.sin( q ) ], 
                     [            0,  1,            0 ],
                     [ -np.sin( q ),  0,  np.cos( q ) ]  ] )

    return Ry

def rotz( q ):
    Rz = np.array( [ [ np.cos( q ), -np.sin( q ), 0 ], 
                     [ np.sin( q ),  np.cos( q ), 0 ],
                     [           0,            0, 1 ]  ] )

    return Rz




def geodesicSO3( rot1, rot2, t0i, D, t):
    """
    Compute the geodesic curve between two rotation matrices with respect to time.

    :param rot1: A 3x3 rotation matrix at the start.
    :param tot2: A 3x3 rotation matrix at the end.
    :param t0i: Initial time for the interpolation.
    :param D: Duration of the interpolation.
    :param t: Current time.
    :return: A 3x3 rotation matrix representing the current state of rotation.
    """
    # Before t0i, use rotation_matrix1
    if t <= t0i:
        return rot1
    
    # After t0i + D, use rotation_matrix2
    elif t >= t0i + D:
        return rot2

    # During the interpolation interval
    else:
        # Normalized time factor for interpolation
        tau = (t - t0i) / D
        
        # Interpolate using slerp
        rot_interp = rot1 @ R3_to_SO3( tau * SO3_to_R3( rot1.T @ rot2 ) )

        return rot_interp



def SO3_to_R3(rotation_matrix):
    """
    Logarithmic map from SO(3) to R3.

    :param rotation_matrix: A 3x3 rotation matrix.
    :return: A 3D vector representing the axis-angle representation.
    """

    # Ensure the matrix is close to a valid rotation matrix
    if not np.allclose(np.dot(rotation_matrix.T, rotation_matrix), np.eye(3)) or not np.allclose(np.linalg.det(rotation_matrix), 1):
        raise ValueError("The input matrix is not a valid rotation matrix.")
 
    angle = np.arccos((np.trace(rotation_matrix) - 1) / 2)
    
    # Check for the singularity (angle close to 0)
    if np.isclose(angle, 0) or np.isnan( angle ):
        return np.zeros(3)

    # Compute the skew-symmetric matrix
    skew_symmetric = (rotation_matrix - rotation_matrix.T) / (2 * np.sin(angle))
    
    # Extract the rotation axis
    axis = np.array([skew_symmetric[2, 1], skew_symmetric[0, 2], skew_symmetric[1, 0]])

    return axis * angle

def R3_to_SO3( r3_vector ):
    angle = np.linalg.norm(r3_vector)
    if np.isclose(angle, 0):
        return np.eye(3)
    
    axis = r3_vector / angle
    skew_symmetric = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rotation_matrix = np.eye(3) + np.sin(angle) * skew_symmetric + (1 - np.cos(angle)) * np.dot(skew_symmetric, skew_symmetric)

    return rotation_matrix
