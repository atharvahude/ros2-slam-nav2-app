#!/usr/bin/env python3
'''
Author: Atharva Hude
'''

import math
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from twist_mux_msgs.action import JoyTurbo

class RobotState(Enum):
    SAFE = 0
    WARNING = 1
    DANGER = 2



class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        
        #Initializing the robot state
        self.robot_state = RobotState.SAFE
        self.robot_previous_state = RobotState.SAFE

        #Declaring the parameters
        self.declare_parameter('distance_threshold', 0.4) #This is the distance threshold for the safety stop
        self.declare_parameter('warning_distance',0.8) #This is the distance threshold for the safety slowdown or warning zone
        self.declare_parameter('scan_topic', "/scan") #This is the topic where the laser scan data is published
        self.declare_parameter('safety_stop_topic', "safety_stop") #Topic to publish the safety stop message

        #Getting the parameters
        self.danger_threshold = self.get_parameter("distance_threshold").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value
        self.warning_threshold = self.get_parameter("warning_distance").get_parameter_value().double_value

        #Subscribing to the laser scan topic
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10) #The callback function is laser_callback

        #Publishing the safety stop message
        self.safety_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        
        #Action client Twist Mux
        self.decrease_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_decrease')
        self.increase_speed_client = ActionClient(self, JoyTurbo,  'joy_turbo_increase')

        #Waiting for the action servers

        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('action server decrease not available, waiting again...')
        
        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('action server increase not available, waiting again...')

        self.get_logger().info('action servers available, continuing...')


    
    def laser_callback(self, laser_msg : LaserScan):
        
        #Initializing the robot state
        self.robot_state = RobotState.SAFE

        #Iterating through the laser scan data
        for range_value in laser_msg.ranges:

            #Checking if the robot is in the warning zone
            if not math.isinf(range_value) and range_value <= self.warning_threshold:
                self.robot_state = RobotState.WARNING

                #Checking if the robot is in the danger zone
                if  range_value <= self.danger_threshold:
                    self.robot_state = RobotState.DANGER
                    break

        #Checking if the robot state has changed
        if self.robot_state != self.robot_previous_state:

            safety_lock = Bool()
            
            if self.robot_state == RobotState.WARNING:
                safety_lock.data = False
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())
                self.get_logger().info('WARNING')
            
            elif self.robot_state == RobotState.DANGER:
                safety_lock.data = True
                self.get_logger().info('DANGER')

            elif self.robot_state == RobotState.SAFE:
                safety_lock.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())
            
            self.robot_previous_state = self.robot_state        
            self.safety_pub.publish(safety_lock) #Publishing the safety stop message




def main(args=None):
    rclpy.init(args=args)
    safety_stop = SafetyStop()
    rclpy.spin(safety_stop)
    safety_stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            





