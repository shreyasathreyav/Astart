#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist,Point,Pose
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2
#map
Map = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

class PathPlanner():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
def astar(grid,start, end):
    """Returns a list of tuples as a path from the given start to the given end"""

    # Create start and end node
    start_node = PathPlanner(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = PathPlanner(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    visited =[]
    # Add the start node
    open_list.append(start_node)
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, 1), (0, -1), (-1, 0), (1, 0)]: # Adjacent positions

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # checking node visitied
            if node_position in visited:
                continue
            # Make sure within grid
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if grid[int(node_position[0])][int(node_position[1])] == 1:
                continue
            visited.append(node_position)
            new_node = PathPlanner(current_node, node_position)
            children.append(new_node)

        # Loop through children
        #print(visited)
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
                continue

            # Add the child to the open list
            open_list.append(child)

def world_to_grid(x,y):
    """Returs world co-ordinates as rows and columns in map """
    o_x=-9
    o_y=-10
    col= int(x-o_x)
    row = int(y+o_y)
    return abs(row),col

def grid_to_world(grid_points):
    """Returs map indcies as world co-ordinates """
    o_x=-9
    o_y=-10
    gx,gy= grid_points
    map_x= gy+o_x
    map_y = -gx-o_y
    return map_x,map_y
def distance(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2+(y1-y2)**2)
x=0
y=0
theta=0

def callback(msg):
	global x
	global y
	global theta
	x= msg.pose.pose.position.x
	y= msg.pose.pose.position.y
	rot= msg.pose.pose.orientation
	(roll,pitch,theta)= euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
	
		
if __name__ == '__main__':
	start = world_to_grid(-8, -2)
	goal_x = 4
	goal_y = 9
	goal= world_to_grid(goal_x,goal_y)
	print(goal)
	path = astar(Map, start, goal)
	if Map[goal[0]][goal[1]]==1:
		print("Path not exists")
	else:
		points=[]
		for i in path:
			points.append(grid_to_world(i))
		print(points)
		points= points[1:]
		rospy.init_node("plan")
		sub= rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
		pub= rospy.Publisher("/cmd_vel", Twist, queue_size=4)
		speed= Twist()
		r= rospy.Rate(1)
		for point in points:
			goal= Point()
			goal.x=point[0]
			goal.y=point[1]
			print(" current_goal",(goal.x,goal.y))
			while distance(goal.x,goal.y,x,y)>0.1:
				inc_x= goal.x-x
				inc_y= goal.y-y
				#print(inc_x,inc_y)
				angle_to_goal= atan2(inc_y,inc_x)
				if abs(angle_to_goal-theta)>0.2:
					speed.linear.x=0.0
					#print("angle", angle_to_goal)
					speed.angular.z=1
				else:
			
					speed.linear.x=4.0
					speed.angular.z=0.0
				pub.publish(speed)
				r.sleep()
		

