#!/usr/bin/python

import math
import numpy as np
import rospy as rp
from geometry_msgs.msg import Twist

class VelocityController:
    def __init__(self):
        self.ang_vel = 0.0
        self.lin_vel = 0.0
        self.kp = 1.2
        self.goal_angle = 0.0
        self.current_yaw = 0.0
        self.vel_pub = rp.Publisher('/cmd_vel', Twist, queue_size=10)

    def publish_velocity(self):
        command = Twist()
        
        #calculate error value - if statement accounts for wraparound of angle value
        if self.goal_angle > math.pi/2.0 and self.current_yaw < -math.pi/2.0:
            err = -((math.pi - abs(self.goal_angle)) + (math.pi - abs(self.current_yaw)))
        elif self.goal_angle < -math.pi/2.0 and self.current_yaw > math.pi/2.0:
            err = (math.pi - abs(self.current_yaw)) + (math.pi - abs(self.goal_angle))
        else:
            err = self.goal_angle - self.current_yaw

        #simple proportional controller to set command velocity
        command.angular.z = self.kp * err
        
        self.vel_pub.publish(command)

        # rp.loginfo('Current Angle: {}; Target: {}'.format(self.current_yaw, self.goal_angle))
        # rp.loginfo('Published angular velocity: {}'.format(command.angular.z))

    def set_goal_angle(self, angle):

        #limit angle to range -pi to +pi
        angle = np.clip(angle, a_min=-math.pi, a_max=math.pi)

        #update goal angle and publish a command to turn the robot towards that angle
        self.goal_angle = angle
        self.publish_velocity()

        # rp.loginfo('Set goal angle to {} ({} degrees)'.format(angle, math.degrees(angle)))

    def set_current_yaw(self, yaw):
        self.current_yaw = yaw

        # rp.loginfo('Set current yaw to {} ({} degrees)'.format(yaw, math.degrees(yaw)))