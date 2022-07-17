#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Transform

class KinovaInverseKinematics:
    def __init__(self):
        self.joint_angles = np.zeros((1,3))    
        self.l1 = 0.21
        self.l2 = 0.19
        
    def get_angles(self, transform):
        self.joint_angles = [0,0,0]
        x = transform.translation.x
        y = transform.translation.y
        # y = 0
        l1 = self.l1
        l2 = self.l2
        
        arg2 = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
        if arg2 > 1:
            print("acos limit",arg2)
            arg2 = 1

        elif arg2 < -1:
            print("acos limit",arg2)
            arg2 = -1

        q2 = math.acos(arg2)
        if x == 0:
            q1 = math.pi/2
        else:
            q1 = math.atan2(y,x) - math.atan(self.l2*math.sin(q2)/(self.l1 + self.l2*math.cos(q2)))
        # print(q1,q2)
        self.joint_angles[0] = q1
        self.joint_angles[1] = q2
        # print(self.joint_angles)
        return self.joint_angles


if __name__ == '__main__':
    test_transform = Transform()

    l1 = 132*0.001
    l2 = 213 * 0.001 # shin length
    l3 = 192 *0.001 # thigh length
    # theta = math.radians(30)
    theta = math.pi/8

    test_transform.translation.x = 0.0
    test_transform.translation.y = 0.39
    test_transform.translation.z = 0.15
    test_transform.rotation.x = 0
    test_transform.rotation.y = 0
    test_transform.rotation.z = 0
    test_transform.rotation.w = 1
    ik = KinovaInverseKinematics()
    angles = ik.get_angles(test_transform)
    print(math.degrees(angles[0]), math.degrees(angles[1]),math.degrees(angles[2]))
    initial_angles = [ 150, 180]

    theta1 = initial_angles[0] + math.degrees(angles[0])
    theta2 = initial_angles[1] - math.degrees(angles[1])

    # print((theta1), math.degrees(theta2),math.degrees(theta3))
    print(theta1,theta2)


    




