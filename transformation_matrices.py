# -*- coding: utf-8 -*-
"""
Functions for calculating Basic Transformation Matrices in 3D space.
"""
from math import cos, radians, sin
from numpy import matrix

def rotate(axis, theta, angular_units='radians'):
    '''Compute Basic Homogeneous Transform Matrix for
    rotation of "theta" about specified axis.'''
    #Verify string arguments are lowercase
    axis=axis.lower()
    angular_units=angular_units.lower()
    #Convert to radians if necessary
    if angular_units=='degrees':
        theta=radians(theta)
    elif angular_units=='radians':
        pass
    else:
        raise Exception('Unknown angular units.  Please use radians or degrees.')
    #Select appropriate basic homogenous matrix
    if axis=='x':
        rotation_matrix=matrix([[1, 0, 0, 0],
                               [0, cos(theta), -sin(theta), 0],
                               [0, sin(theta), cos(theta), 0],
                               [0, 0, 0, 1]])
    elif axis=='y':
        rotation_matrix=matrix([[cos(theta), 0, sin(theta), 0],
                               [0, 1, 0, 0],
                               [-sin(theta), 0, cos(theta), 0],
                               [0, 0, 0, 1]])  
    elif axis=='z':
        rotation_matrix=matrix([[cos(theta), -sin(theta), 0, 0],
                               [sin(theta), cos(theta), 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])
    else:
        raise Exception('Unknown axis of rotation.  Please use x, y, or z.')
    return rotation_matrix

def translate(axis, d):
    '''Calculate Basic Homogeneous Transform Matrix for
    translation of "d" along specified axis.'''   
    #Verify axis is lowercase
    axis=axis.lower()
    #Select appropriate basic homogenous matrix
    if axis=='x':
        translation_matrix=matrix([[1, 0, 0, d],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
    elif axis=='y':
        translation_matrix=matrix([[1, 0, 0, 0],
                                  [0, 1, 0, d],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
    elif axis=='z':
        translation_matrix=matrix([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, d],
                                  [0, 0, 0, 1]])
    else:
        raise Exception('Unknown axis of translation.  Please use x, y, or z.')
    return translation_matrix

if __name__=='__main__':
    #Calculate arbitrary homogeneous transformation matrix for CF0 to CF3
    H0_1=rotate('x', 10, 'degrees')*translate('y', 50)
    H1_2=rotate('y', 30, 'degrees')*translate('z', 10)
    H2_3=rotate('z', -20, 'degrees')*translate('z', 10)
    H0_3=H0_1*H1_2*H2_3
    print(H0_3)