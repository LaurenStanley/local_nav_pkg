#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math
import time

goal_pose_x = 0.0
goal_pose_y = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0
odometry_pose_theta = 0.0
goal_reached = False

class HuskyVelocityPublisherNode(Node):

    def __init__(self):
        super().__init__('husky_velocity_publisher')
        self.subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.subscriber2 = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pose_timer_ = self.create_timer(
            2.0, self.publish_husky_velocity)
        

    def publish_husky_velocity(self):
        x_dist = odometry_pose_x - goal_pose_x
        y_dist = odometry_pose_y - goal_pose_y
        dist = math.sqrt(x_dist*x_dist + y_dist*y_dist)
        global goal_reached
        speed = Twist()
       
        angle_to_goal = math.atan2(-y_dist, -x_dist)
        print(round(angle_to_goal,2),round(odometry_pose_theta,2))
        print(round(x_dist,2), round(y_dist,2))
        
        if not goal_reached:	
            if angle_to_goal - odometry_pose_theta > 0.1 and angle_to_goal - odometry_pose_theta < math.pi:
                print('Adjusting Angle - positive')
                speed.linear.x = 0.0
                speed.angular.z = 1.0
            elif angle_to_goal - odometry_pose_theta < -0.1 and angle_to_goal - odometry_pose_theta > -math.pi:
                print('Adjusting Angle - negative')
                speed.linear.x = 0.0
                speed.angular.z = -1.0
            elif dist > 5.0:
                print('Fast Approach')
                speed.linear.x = 5.0
                speed.angular.z = 0.0
            elif dist > 2.0:
                print('Moderate Approach')
                speed.linear.x = 1.0
                speed.angular.z = 0.0
            elif dist > 1.0:
                print('Slow Approach')
                speed.linear.x = 0.5
                speed.angular.z = 0.0
            else:
                print('Goal Reached!')
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                goal_reached = True
                time.sleep(10)
        else:
            if dist > 3.0:
                print('Backed Away')
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                goal_reached = False
            elif dist > 2.0:
                print('Moderate Disengagement')
                speed.linear.x = -1.0
                speed.angular.z = 0.0
            elif dist > 0.0:
                print('Slow Approach')
                speed.linear.x = -0.5
                speed.angular.z = 0.0
                
        #speed.linear.x = 0.5
        #speed.linear.y = 0.0
        #speed.linear.z = 0.0
        
        #speed.angular.x = 0.0
        #speed.angular.y = 0.0
        #speed.angular.z = 0.0
        
        
        #speed_limit.speed_limit = 1.0
        self.publisher_.publish(speed)

        
    def goal_pose_callback(self, msg):
        global goal_pose_x
        global goal_pose_y
        goal_pose_x = msg.pose.position.x
        goal_pose_y = msg.pose.position.y
        

    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        global odometry_pose_theta
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        
        #(roll, pitch, odometry_pose_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
        siny_cosp = 2 * (rot_q.w * rot_q.z + rot_q.x * rot_q.y)
        cosy_cosp = 1 - 2 * (rot_q.y * rot_q.y + rot_q.z * rot_q.z)
        odometry_pose_theta = math.atan2(siny_cosp, cosy_cosp)
    	
 
def main(args=None):
    rclpy.init(args=args)
    node = HuskyVelocityPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
