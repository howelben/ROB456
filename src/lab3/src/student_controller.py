#!/usr/bin/env python3


import sys
import rospy
import signal
import numpy as np
import path_planning as pathplan
import exploring as explore

from controller import RobotController


class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()

	def distance_update(self, distance):
		'''
		This function is called every time the robot moves towards a goal.  If you want to make sure that
		the robot is making progress towards it's goal, then you might want to check that the distance to
		the goal is generally going down.  If you want to change where the robot is heading to, you can
		make a call to set_waypoints here.  This call will override the current set of waypoints, and the
		robot will start to drive towards the first waypoint in the new list.

		Parameters:
			distance:	The distance to the current goal.
		'''
		rospy.loginfo(f'Distance: {distance}')

	def map_update(self, point, map, map_data):
		'''
		This function is called every time a new map update is available from the SLAM system.  If you want
		to change where the robot is driving, you can do it in this function.  If you generate a path for
		the robot to follow, you can pass itS to the driver code using set_waypoints().  Again, this will
		override any current set of waypoints that you might have previously sent.

		Parameters:
			point:		A PointStamped containing the position of the robot, in the map coordinate frame.
			map:		An OccupancyGrid containing the current version of the map.
			map_data:	A MapMetaData containing the current map meta data.
		'''
		rospy.loginfo('Got a map update.')

		# It's possible that the position passed to this function is None.  This try-except block will deal
		# with that.  Trying to unpack the position will fail if it's None, and this will raise an exception.
		# We could also explicitly check to see if the point is None.
		size_pix = map.info.resolution
		origin = map.info.origin.position.x
		im_size = (map.info.width, map.info.height)
  

		try:
			# The (x, y) position of the robot can be retrieved like this.
			robot_position = (point.point.x, point.point.y)
			rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
		except:
			rospy.loginfo('No odometry information')

		im = np.array(map.data).reshape(map.info.height, map.info.width)
		rospy.loginfo(f"Height: {map.info.height}, Width: {map.info.height}, Origin:{ map.info.origin.position.x}, {map.info.origin.position.y}")
		rospy.loginfo(f"Pixel size{map.info.resolution}")
		im_thresh = pathplan.convert_image(im, 0.3, 0.7)
		rospy.loginfo(f"Array Size{im_thresh.size}")
		pix = explore.convert_x_y_to_pix(im_size, robot_position, size_pix, origin)
		x_y = explore.convert_pix_to_x_y(im_size, pix, size_pix, origin)
		rospy.loginfo(f"Pixel: {pix}")
		rospy.loginfo(f"x_y:{x_y}")
		#Convert robot position to pixel on the map. Robot position is -4.5, -4.5
		# possible_points = explore.find_all_possible_goals(im_thresh)
		# rospy.loginfo(f"Possible points: {possible_points}")
		# best_point = explore.best_point(im_thresh, possible_points, robot_position)
		# rospy.loginfo(f"Best point: {best_point}")
		# path = pathplan.dijkstra(im_thresh, robot_position, best_point)
		# rospy.loginfo(f"Path: {path}")
		# waypoints = pathplan.find_waypoints(im_thresh, path)
		# rospy.loginfo(f"Waypoint:{waypoints}")

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	controller.set_waypoints(((-4, -3), (-4, 0), (5, 0)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()