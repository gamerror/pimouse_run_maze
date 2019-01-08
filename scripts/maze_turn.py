#!/usr/bin/env python
#encoding: utf-8

import rospy, copy, math, time
#import ptvsd       # for remote debug
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_ros.srv import TimedMotion

class MazeExplore():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.timed_motion = rospy.ServiceProxy('/timed_motion', TimedMotion)
        
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

        self.move_delay = 0.05          # Delay of movement feedback [sec]
        self.move_correct = 0.970       # Correction distance of movement
        self.turn_delay = 0.050         # Delay of turn feedback [sec]
        self.turn_left_correct = 0.970  # Correction angle of turn left
        self.turn_right_correct = 0.960 # Correction angle of turn right

        self.intensity_slowdown = 100   # threshold of forward sensor to slow down
        self.intensity_stop = 4000      # Sensor intensity sum_forward of positioning
        self.intensity_exist_side = 200 # Sensor intensity of side to detect side wall
        self.steering = 0.002           # Sterring response
        self.steering_smoothest = 0.5   # smoothest steering rate in maximum velocity

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def show_lightsensors(self, comment, s):
        #print comment, s.left_side, s.left_forward, s.right_forward, s.right_side, ' : ', s.sum_forward, s.sum_all
        pass        # Do none.
        
    def move(self, dist_mm, vel_mmsec):
        dist_mm *= self.move_correct
        if dist_mm < 0:
            vel_mmsec = -abs(vel_mmsec)
        vel_hz = 2.82942121052 * vel_mmsec      # 400 / (45 * pi) [pulse/mm]
        duration = 1000 * max(dist_mm / vel_mmsec + self.move_delay, 0)
        self.timed_motion(vel_hz, vel_hz, duration)

    def turn(self, turn_deg, vel_degsec):
        if turn_deg >= 0:       # 200 pulse = 90 degree
            hz = 2.222222 * vel_degsec
            turn_deg = turn_deg * self.turn_right_correct
        else:
            hz = -2.222222 * vel_degsec
            turn_deg = turn_deg * self.turn_left_correct
        duration = 1000 * max(abs(turn_deg) / vel_degsec + self.turn_delay, 0)
        self.timed_motion(hz, -hz, duration)        

    def run(self):
        # motion parameter
        rate_normal = 10    # Normal sampling rate [Hz]
        rate_fine = 20      # Fine sampling rate for positioning [Hz]
        accel = 40          # Acceleration
        decel = 60          # Deceleration
        vel_max = 400       # Maximum velocity [mm/sec]
        vel_min = 100       # Minimum velocity [mm/sec]

        rospy.set_param('lightsensors_freq', rate_normal)
        rate = rospy.Rate(rate_normal)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = 0.0
        while not rospy.is_shutdown():
            s = self.sensor_values
            self.show_lightsensors('Go: ', s)

            data.linear.x += accel
            data.angular.z = 0.0

            if s.left_forward >= self.intensity_slowdown and s.right_forward >= self.intensity_slowdown:    # Detected a front wall.
                rospy.set_param('lightsensors_freq', rate_fine)
                rate = rospy.Rate(rate_fine)
                while data.linear.x > 0:
                    s = self.sensor_values

                    if s.left_forward < self.intensity_slowdown or s.right_forward < self.intensity_slowdown:
                        print 'Restart'
                        break

                    data.linear.x -= decel
                    if data.linear.x < vel_min:
                        data.linear.x = vel_min
                    if s.sum_forward >= self.intensity_stop:
                        data.linear.x = 0       # Arrived at the face of front wall.
                    self.show_lightsensors('front wall: ', s)
                    self.cmd_vel.publish(data)
                    rate.sleep()

                rospy.set_param('lightsensors_freq', rate_normal)
                rate = rospy.Rate(rate_normal)
                    
            elif data.linear.x <= vel_min:
                data.linear.x = vel_min
            elif data.linear.x >= vel_max:
                data.linear.x = vel_max

            if data.linear.x < vel_min:
                data.angular.z = 0.0
            elif s.left_side < self.intensity_exist_side or s.right_side < self.intensity_exist_side:
                data.angular.z = 0.0
            else:
                data.angular.z = (s.right_side - s.left_side) * self.steering
                data.angular.z *= (1.0 - self.steering_smoothest * data.linear.x / vel_max)
                #print 'steering: ', data.angular.z

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
    
    maze = MazeExplore()
    time.sleep(1)
    #maze.run()
    maze.move(50, 100)
    time.sleep(1)
    maze.turn(-90, 120)         # Turn left
    time.sleep(1)
    maze.turn(90, 120)          # Turn right
    time.sleep(1)
    maze.turn(180, 120)         # Right-about-face
    time.sleep(1)
    maze.turn(-180, 120)        # Left-about-face
    time.sleep(1)
    maze.move(-50, 100)
