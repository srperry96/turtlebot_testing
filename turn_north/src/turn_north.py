#!/usr/bin/python

import rospy as rp

from velocity_controller import VelocityController
from sensor_input import SensorInput


if __name__ == "__main__":
	rp.init_node('turn_north_controller')

	sensors = SensorInput()
	vel_ctrl = VelocityController()

	rate = rp.Rate(10)
	while not rp.is_shutdown():
		vel_ctrl.set_current_yaw(sensors.yaw)
		vel_ctrl.publish_velocity()

		rate.sleep()