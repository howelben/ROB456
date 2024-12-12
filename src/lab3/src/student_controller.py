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
	This class allows you to set _waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()
		self.distance_history = []
		self.seen_goals = []
		self.current_best_point = None
		self.im_thresh = 0
		self.size_pix = 0
		self.origin = 0
		self.im_size = []
		self.curr_position = None
	def distance_update(self,  distance):
		'''
		This function is called every time the robot moves towards a goal.  If you want to make sure that
		the robot is making progress towards it's goal, then you might want to check that the distance to
		the goal is generally going down.  If you want to change where the robot is heading to, you can
		make a call to set__waypoints here.  This call will override the current set of _waypoints, and the
		robot will start to drive towards the first waypoint in the new list.

		Parameters:
			distance:	The distance to the current goal.
		'''
  
		# I don;t know what I am doing here
		rospy.loginfo(f'Distance: {distance}')
		if distance < 0.7:
			rospy.loginfo(f"Waypoint reached.")
			self.distance_history = []  # Reset history when a waypoint is reached
			if len(self._waypoints) <= 1:
				self.path_update(self.curr_position)
				

		
		if len(self.distance_history) == 10 and all(self.distance_history[i] <= self.distance_history[i + 1] for i in range(len(self.distance_history) - 1)):
			rospy.loginfo("Robot stuck. Recalculating path.")
			self.distance_history = []  # Reset history
			if self._waypoints:
				new_position = self._waypoints[0]
			else:
				new_position = self.curr_position
			self._waypoints = []
			self.path_update(new_position)

		



	def map_update(self, point, map, map_data):
		'''
		This function is called every time a new map update is available from the SLAM system.  If you want
		to change where the robot is driving, you can do it in this function.  If you generate a path for
		the robot to follow, you can pass itS to the driver code using set__waypoints().  Again, this will
		override any current set of _waypoints that you might have previously sent.

		Parameters:
			point:		A PointStamped containing the position of the robot, in the map coordinate frame.
			map:		An OccupancyGrid containing the current version of the map.
			map_data:	A MapMetaData containing the current map meta data.
		'''
		rospy.loginfo('Got a map update.')

		# It's possible that the position passed to this function is None.  This try-except block will deal
		# with that.  Trying to unpack the position will fail if it's None, and this will raise an exception.
		# We could also explicitly check to see if the point is None.
		self.size_pix = map.info.resolution
		self.origin = map.info.origin.position.x
		self.im_size = [map.info.width, map.info.height]
		self.im = np.array(map.data).reshape(map.info.height, map.info.width)
		self.im_thresh = pathplan.convert_image(self.im, 0.3, 0.7)

		try:
			# The (x, y) position of the robot can be retrieved like this.
			robot_position = (point.point.x, point.point.y)
			self.curr_position = robot_position
		except:
			rospy.loginfo('No odometry information')
			robot_position = self.curr_position
		rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
		done = self.path_update(robot_position)
		if done == "Done":
			rospy.loginfo("Its finally over")
			controller.set_waypoints([])
   
		
		

		
		
	def path_update(self, robot_position):
		waypoints_xy = []
		if not self._waypoints:
			rospy.loginfo("Calculating new path")
			#Find all possible point
			possible_points = explore.find_all_possible_goals(self.im_thresh)
			robot_pix = tuple(explore.convert_x_y_to_pix(self.im_size, robot_position, self.size_pix, self.origin))
			rospy.loginfo(f"Len possible Points: {len(possible_points)}")
			#Remove goals that are too close to seen goals
			if self.seen_goals:
				temp_points = []
				for point in possible_points:
					point_xy = tuple(explore.convert_pix_to_x_y(self.im_size, point, self.size_pix, self.origin))
					# Check if the point is far enough from all seen goals
		
					if not any(np.linalg.norm(np.array(seen_goal) - np.array(point_xy)) <= 3.0 for seen_goal in self.seen_goals):
						temp_points.append(point)

				possible_points = temp_points
			if not possible_points:
				return "Done"
			#Find best ppinst and calculate path
			rospy.loginfo(f"Len possible Points after removal: {len(possible_points)}")
			best_point = explore.find_best_point(self.im_thresh, possible_points, robot_pix)
			rospy.loginfo(f"Best point: {best_point}")
			#rospy.loginfo(f"Best point XY: {explore.convert_pix_to_x_y(self.im_size, point, self.size_pix, self.origin)}")
			path = pathplan.dijkstra(self.im_thresh, robot_pix, best_point)
			rospy.loginfo(f"Path length: {len(path)}")
			#Find _waypoints and convert to xy tuples
			waypoints = explore.find_waypoints(self.im_thresh, path)
			rospy.loginfo(f"_waypoints: {waypoints}")
			for point in waypoints:
				waypoint  = tuple(explore.convert_pix_to_x_y(self.im_size, point, self.size_pix, self.origin))
				waypoints_xy.append(waypoint)
			#Send waypoint
	
			self.seen_goals.append(waypoints_xy[-1])
			controller.set_waypoints(waypoints_xy)
			rospy.loginfo(f"Current Waypoints: {self._waypoints}")
				


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed _waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	#controller.set_waypoints([()])

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()