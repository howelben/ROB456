#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import atan2, sqrt, sin, tanh


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self, threshold=0.3):
		super().__init__('odom')
		# Set the threshold to a reasonable number
		self._threshold = threshold

	def close_enough_to_waypoint(self, distance, target, lidar):
		'''
		This function is called perioidically if there is a waypoint set.  This is where you should put any code that
		has a smarter stopping criteria then just checking the distance. See get_twist for the parameters; distance
		is the current distance to the target.
		'''
		# Default behavior.
		if distance < self._threshold:
			return True
		return False

	def get_twist(self, target, lidar):
		'''
		This function is called whenever there a current target is set and there is a lidar data
		available.  This is where you should put your code for moving the robot.  The target point
		is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
		positive to the left.

		The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
		code with something that moves the robot more intelligently.

		Parameters:
			target:		The current target point, in the coordinate frame of the robot (base_link) as
						an (x, y) tuple.
			lidar:		A LaserScan containing the new lidar data.

		Returns:
			A Twist message, containing the commanded robot velocities.
		'''
		theta = atan2(target[1], target[0])
		distance = sqrt(target[0] ** 2 + target[1] **2)
		rospy.loginfo(f'Distance: {distance:.2f}, angle: {theta:.2f}')

		# This builds a Twist message with all elements set to zero.
		command = Driver.zero_twist()
		shortest = max(lidar.ranges)
		for i, range in enumerate(lidar.ranges):
			angle_rad = lidar.angle_min + i*lidar.angle_increment
			abs_y = abs(range * sin(angle_rad)) 
			if abs_y < 0.19:
				shortest  = min(shortest, range)#Grabs shortest range
			if range < 0.60: #If lidar detects wall in close to robot, moves robot based off what region the scan is in
				if -0.3 < angle_rad < 0:  # Front right region
					theta += 0.1
				elif 0 < angle_rad < 0.3: #Front left region
					theta -= 0.1
				elif 0.3 <= angle_rad < 1.0:  # Left region
					theta -= 0.25
				elif -1.0 < angle_rad <= -0.3:  # Right region
					theta += 0.25
				if range < 0.2:
					if 1.0 <= angle_rad < 1.4:  # Extreme Left region
						theta -= 0.6
					elif -1.4 < angle_rad <= -1.0:
						theta  += 0.6

		if (shortest - 0.4) < 0.01: #Stops if robot is to close to the wall
			command.linear.x = 0
		else:
			# This sets the move forward speed (as before)
			command.linear.x = tanh(distance)
	# This sets the angular turn speed (in radians per second)
		command.angular.z = theta

		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()