#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as path_planning
# Our priority queue
import heapq

# Using imageio to read in the image
#import imageio


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]



def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE      
    for neighbor in path_planning.eight_connected(pix):
        if path_planning.is_free(im, neighbor):
            return True
    return False


def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""

    # YOUR CODE HERE
    neighbors = [(x,y) for x in [-1, 0, -1] for y in [-1, 0, 1] if (x,y) != (0,0)]
    
    unknown_mask = (im == 128) 
    free_mask = (im == 255)  

    goal_mask = np.zeros_like(im, dtype=bool)

    for di, dj in neighbors:
        shifted_free_mask = np.roll(np.roll(free_mask, di, axis=0), dj, axis=1)
        goal_mask |= (unknown_mask & shifted_free_mask)

    possible_goals = np.argwhere(goal_mask)
    flipped_goals = [(coord[1], coord[0]) for coord in possible_goals]

    return flipped_goals


def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go toa
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    # YOUR CODE HERE
    if not possible_points:
        raise ValueError("No possible points to explore.")
    
    distances = [(point, np.linalg.norm(np.array(point) - np.array(robot_loc)))for point in possible_points]
    
    # Select the farthest point
    best_point, _ = max(distances, key=lambda x: x[1])
    free_points = []
    for neighbor in path_planning.eight_connected(best_point):
        if path_planning.is_free(im, neighbor):
            free_points.append(neighbor)
    #Select farthest free pont
    distances = [(point, np.linalg.norm(np.array(point) - np.array(robot_loc)))for point in free_points]
    best_point, _ = max(distances, key=lambda x: x[1])
    return best_point

def find_waypoints(im, path, res=0.95):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    
    # Split the path equally into points
    split_path = []
    for i, point in enumerate(path):
        if i % 5 == 0:
            split_path.append(point)    

    corners = []
    for i in range(1, len(split_path)-2):
        
        # Get the vectors around the central point
        prev_point = split_path[i-1]
        current_point = split_path[i]
        next_point = split_path[i+1]
        
        # Calculate the curvature at the point
        prev_vector = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
        next_vector = (next_point[0] - current_point[0], next_point[1] - current_point[1])
        curvature = np.dot(prev_vector, next_vector) / (np.linalg.norm(prev_vector) * np.linalg.norm(next_vector))
        
        print(f"Curvature at {i} is {curvature}")
        
        # If the curvature is not 1, then we have a corner
        if curvature < res:
            corners.append(split_path[i])
    corners.append(path[-1])
    return corners

if __name__ == '__main__':
    # Doing this here because it is a different yaml than JN
    import yaml_1 as yaml

    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # plt.show()

    print("Done")
