#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,copy,math
import ptvsd
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.srv import TimedMotion

class MazeTrace():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.timed_motion = rospy.ServiceProxy('/timed_motion', TimedMotion)        

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def turn(self,turn_deg):
        if turn_deg >= 0:
            hz = 400
        else:
            hz = -400
        duration = 2222 * turn_deg / hz         # 400 pulse = 90 degree
        self.timed_motion(hz,-hz,duration)

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

        # motion parameter
        accel = 0.02
        decel = 0.02
        vel_max = 0.4
        vel_min = 0.1
        th_slowdown = 100       # threshold of forward sensor to slow down
        th_stop = 2000          # threshold of forward sensor to stop immediately
        th_ignore = 200         # threshold of ignorant side sensor
        steering = 50.0         # Sterring response

        data.linear.x = 0.0
        data.angular.z = 0
        while not rospy.is_shutdown():
            s = self.sensor_values
            print(s.left_side, s.left_forward, s.right_forward, s.right_side, s.sum_forward)

            data.linear.x += accel
            data.angular.z = 0.0

            if s.sum_forward > th_slowdown:
                while data.linear.x > 0:
                    data.linear.x -= decel
                    if data.linear.x < vel_min:
                        data.linear.x = vel_min
                    if self.sensor_values.sum_forward >= th_stop:
                        data.linear.x = 0
                    print(data.linear.x, self.sensor_values.sum_forward)
                    self.cmd_vel.publish(data)
                    rate.sleep()

            elif data.linear.x <= vel_min:
                data.linear.x = vel_min
            elif data.linear.x >= vel_max:
                data.linear.x = vel_max

            if data.linear.x < vel_min:
                data.angular.z = 0.0
            elif s.left_side < th_ignore or s.right_side < th_ignore:
                data.angular.z = 0.0
            else:
                error = (s.right_side - s.left_side) / steering
                data.angular.z = error * 2 * math.pi / 180.0
                print("steering: ", data.angular.z)

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    # Allow other computers to attach to ptvsd at this IP address and port.
    #ptvsd.enable_attach(address=('0.0.0.0', 5678), redirect_output=True)
    # Pause the program until a remote debugger is attached
    #ptvsd.wait_for_attach()
    #breakpoint()

    rospy.init_node('maze_trace')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.wait_for_service('/timed_motion')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    maze = MazeTrace()
    #maze.run()
    maze.turn(-90)      # Turn left
    time.sleep(1)
    maze.turn(90)       # Turn right
    time.sleep(1)
    maze.turn(180)      # Right-about-face
    time.sleep(1)
    maze.turn(-180)     # Left-about-face
