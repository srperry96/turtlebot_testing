#!/usr/bin/python

import math
import rospy as rp
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from velocity_controller import VelocityController
from sensor_input import SensorInput

class SphereFollower():
    def __init__(self):
        self.sphere_location = [0.0, 0.0, 0.0]
        self.angle_to_sphere = 0.0
        self.vel_ctrl = VelocityController()
        self.sensors = SensorInput()

        #wait for gazebo to be ready
        rp.wait_for_service('/gazebo/get_model_state')
        self.get_location_srv = rp.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def get_angle_to_sphere(self):
        """
            Using the positions of the robot and the sphere in gazebo, calculate the
            angle between the two. Throws an error if the sphere or robot don't exist.
        """
        #Get robot and sphere locations
        sphere_model = GetModelStateRequest()
        sphere_model.model_name = 'unit_sphere'
        robot_model = GetModelStateRequest()
        robot_model.model_name = 'turtlebot3_burger'
    
        result_sphere = self.get_location_srv(sphere_model)
        if not result_sphere.success:
            rp.logerr('Error in getting sphere model locations - Have you added the unit_sphere to gazebo?')
            return

        result_robot = self.get_location_srv(robot_model)
        if not result_sphere.success:
            rp.logerr('Error in getting robot model locations - Have you added the unit_sphere to gazebo?')
            return

        sphere_x = result_sphere.pose.position.x
        sphere_y = result_sphere.pose.position.y
        robot_x = result_robot.pose.position.x
        robot_y = result_robot.pose.position.y

        #calculating the angle between the two
        x_diff = sphere_x - robot_x
        y_diff = sphere_y - robot_y

        self.angle_to_sphere = math.atan2(y_diff, x_diff)
        # rp.loginfo('Angle to sphere: {}'.format(self.angle_to_sphere))

    def run(self):
        """
            Run the sphere follower controller at a rate of 10Hz.
            Gets the angle to the sphere before publishing the relevant velocity command.
        """
        rate = rp.Rate(10)
        while not rp.is_shutdown():
            self.get_angle_to_sphere()
            self.vel_ctrl.set_current_yaw(self.sensors.yaw)
            self.vel_ctrl.set_goal_angle(self.angle_to_sphere)
            rate.sleep()


if __name__ == "__main__":
    rp.init_node('sphere_follower')
    follower = SphereFollower()
    follower.run()