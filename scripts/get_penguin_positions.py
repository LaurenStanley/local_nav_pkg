#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from local_nav_pkg.msg import Penguin
from local_nav_pkg.msg import PenguinList
from local_nav_pkg.msg import PenguinApproachStatus
import math

list_of_points = []
penguin_list = []
penguin_label_count = 0

x = 0.0
y = 0.0
x_goal = 0.0
y_goal = 0.0

odometry_pose_x = 0.0
odometry_pose_y = 0.0

penguin_label = str(0)
approach_status = "engaging"
visited_points = []

class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('target_obstacle_publisher')
        self.penguin_list_subscriber = self.create_subscription(PenguinList, 'penguin_list',self.penguin_list_callback,10)
        self.point_cloud_subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)
        
        self.penguin_publisher_ = self.create_publisher(PenguinList, 'penguin_list', 10)
        self.penguin_timer_ = self.create_timer(0.1, self.publish_penguins)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose_timer_ = self.create_timer(0.1, self.publish_goal_pose)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)
        self.approach_status_timer_ = self.create_timer(.1, self.publish_approach_status)
        

    def publish_penguins(self):     
        global penguin_list
        global list_of_points
        global penguin_label_count

        final_penguin_list = PenguinList()
        accumulated_penguin_list = []
        already_exists = False
        for i in range(len(list_of_points)):
            for j in range(len(penguin_list)):
                dist = (penguin_list[j].point.x - list_of_points[i][0])**2 + (penguin_list[j].point.y - list_of_points[i][1])**2
                if dist < 0.5:
                    #print('Penguin Exists')
                    old_penguin = Penguin()
                    old_penguin.point.x = list_of_points[i][0]
                    old_penguin.point.y = list_of_points[i][1]
                    old_penguin.point.z = 0.0
                    old_penguin.label = penguin_list[j].label
                    #old_penguin.status = penguin_list[j].status
                    accumulated_penguin_list.append(old_penguin)
                    already_exists = True
            if not already_exists:
                #print('Penguin Does Not Exist')
                new_penguin = Penguin()
                new_penguin.point.x = list_of_points[i][0]
                new_penguin.point.y = list_of_points[i][1]
                new_penguin.point.z = 0.0
                new_penguin.label = str(penguin_label_count)
                #new_penguin.status = "unvisited"
                penguin_label_count += 1
                accumulated_penguin_list.append(new_penguin)
        
        sorted_penguin_list = sorted(accumulated_penguin_list, key=lambda penguin: penguin.label)
        #if len(sorted_penguin_list) > 0:
        #    sorted_penguin_list[0].status = "current"
        final_penguin_list.penguins = sorted_penguin_list
        print(final_penguin_list)    
        self.penguin_publisher_.publish(final_penguin_list)
    
    def penguin_list_callback(self, msg):
        global x_goal
        global y_goal
        global penguin_list
        global visited_points
        global penguin_label
        global odometry_pose_x
        global odometry_pose_y

        distance = 100
        penguin_list = []
        
        for i in range(len(msg.penguins)):
            #print(msg.penguins[i])
            penguin_list.append(msg.penguins[i])
            if msg.penguins[i].label not in visited_points:
                i_distance = (odometry_pose_x - msg.penguins[i].point.x)**2 + (odometry_pose_y - msg.penguins[i].point.y)**2
                if i_distance < distance:
                    penguin_label = msg.penguins[i].label
                    x_goal = msg.penguins[i].point.x
                    y_goal = msg.penguins[i].point.y
                    distance = i_distance

    def marker_array_callback(self, msg):
        global list_of_points
        list_of_points = []
        for i in range(len(msg.markers)):
            point = []
            point.append(msg.markers[i].pose.position.x)
            point.append(msg.markers[i].pose.position.y)
            point.append(msg.markers[i].pose.position.z)
            list_of_points.append(point)
            #print(point)

    def publish_approach_status(self):
        global penguin_label
        global approach_status

        penguin_status = PenguinApproachStatus()

        penguin_status.label = penguin_label
        penguin_status.status = approach_status

        self.approach_status_publisher_.publish(penguin_status)

    def publish_goal_pose(self):     
        global x_goal
        global y_goal
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'velodyne'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        #speed_limit.speed_limit = 1.0
        self.goal_pose_publisher_.publish(goal_pose)


    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
    
    def approach_status_callback(self, msg):
        global penguin_label
        global approach_status
        global visited_points
        penguin_label = msg.label
        approach_status = msg.status
        if approach_status == "complete":
            visited_points.append(penguin_label)


  	
 
def main(args=None):  
    rclpy.init(args=args)
    print('Start')
    node = ObstaclePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
