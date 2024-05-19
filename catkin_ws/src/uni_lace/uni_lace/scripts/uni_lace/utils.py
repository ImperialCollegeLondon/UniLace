import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, decompose_matrix, compose_matrix, quaternion_matrix, quaternion_multiply


def tf_mat_to_list(mat):
    _, _, euler, trans, _ = decompose_matrix(mat)
    return np.concatenate((trans, euler))

def tf_list_to_mat(list):
    assert len(list) == 6, f'Wrong input dimension! Got a list with {len(list)} elements.'
    return compose_matrix(translate=list[:3], angles=list[3:])

def add_lists(l_a, l_b):
    assert len(l_a) == len(l_b), 'Lists have different lengths!'
    return [a+b for a,b in zip(l_a, l_b)]

def pose_msg_to_list(pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def list_to_point_msg(list):
    return Point(*list[:3])

def list_to_pose_msg(list):
    p = Pose()
    p.position = Point(*list[:3])
    if len(list) == 6:
        p.orientation = Point(*list[3:])
    elif len(list) == 7:
        p.orientation = Point(*list[3:])
    else:
        print(f'Wrong input dimension! Got list with {len(list)} elements.')
        return
    return p

def euler_mul(euler_b, euler_a):
    ''' euler multiplication. Apply rotation a first. '''
    return euler_from_quaternion(quaternion_multiply(quaternion_from_euler(*euler_a), quaternion_from_euler(*euler_b)))

def concat(a, b):
    ''' concatenate two 1d lists '''
    return [*a, *b]

def ls_concat(a, b):
    ''' concatenate two 1d lists '''
    return [*a, *b]

def ls_add(l_a, l_b):
    ''' add two 1d lists together '''
    assert len(l_a) == len(l_b), f'Input lists have different lengths of {len(l_a)} and {len(l_b)}!'
    return [a+b for a,b in zip(l_a, l_b)]
    
def is_sorted(a):
    ''' check if a list is in ascending order '''
    return np.all(a[:-1] <= a[1:])