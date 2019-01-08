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

        self.setback = 19.0             # Setback distance of front nose from front wall to center of block [mm]
        self.sidewall_center = 30.0     # Distance to center of side wall for trun [mm]
        self.move_delay = 0.05          # Delay of movement feedback [sec]
        self.move_correct = 0.950       # Correction distance of movement
        self.slow_velocity = 50         # Velocity of positioning [mm/sec]
        self.interval_time = 0.5        # Interval time between movement [sec]

        self.turn_velocity = 120        # Angular velocity of turn [deg/sec]
        self.turn_slow_velocity = 20    # Angular velocity of positioning [deg/sec]
        self.turn_delay = 0.050         # Delay of turn feedback [sec]
        self.turn_left_correct = 0.920  # Correction angle of turn left
        self.turn_right_correct = 0.920 # Correction angle of turn right
        self.turn_slow_correct = 1.00   # Correction angle of positioning
        self.turn_dir = True            # Alternate turn

        self.intensity_slowdown = 100   # threshold of forward sensor to slow down
        self.intensity_target = 6000    # Sensor intensity sum_forward of positioning
        self.intensity_stop = 5000      # Sensor intensity sum_forward of positioning
        self.intensity_exist_side = 200 # Sensor intensity of side to detect side wall
        self.steering = 0.002           # Sterring response
        self.steering_smoothest = 0.5   # smoothest steering rate in maximum velocity

        self.ratio_face_angle = 1.0     # Sensor intensity ratio in face of front wall

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def show_lightsensors(self, comment, s):
        #print comment, s.left_side, s.left_forward, s.right_forward, s.right_side, ' : ', s.sum_forward, s.sum_all
        pass        # Do none.
        
    def calibration(self):
        # Go back to block center from front wall, read sensor by step.
        self.move(-0.75 * self.setback, self.slow_velocity)
        rospy.sleep(self.interval_time)
        sensor_stop_previos_point = self.sensor_values

        self.move(-0.25 * self.setback, self.slow_velocity)
        rospy.sleep(self.interval_time)
        sensor_stop_front_target = self.sensor_values

        self.intensity_per_mm = (sensor_stop_previos_point.sum_forward - sensor_stop_front_target.sum_forward) / (0.25 * self.setback)
        self.ratio_faced_front = float(sensor_stop_front_target.left_forward) / float(sensor_stop_front_target.right_forward)

        # Divert left from front.
        calib_divert_angle = 10.0
        self.turn(-calib_divert_angle, self.turn_velocity)
        rospy.sleep(self.interval_time)
        sensor_divert_left = self.sensor_values
        self.show_lightsensors('sensor_divert_left: ', sensor_divert_left)
        self.ratio_per_deg_faced_left = float(sensor_divert_left.left_forward) / float(sensor_divert_left.right_forward)
        self.ratio_per_deg_faced_left -= self.ratio_faced_front
        self.ratio_per_deg_faced_left  /= -calib_divert_angle

        # Divert right from front.
        self.turn(calib_divert_angle * 2, self.turn_velocity)
        rospy.sleep(self.interval_time)
        sensor_divert_right = self.sensor_values
        self.show_lightsensors('sensor_divert_right: ', sensor_divert_right)
        self.ratio_per_deg_faced_right = float(sensor_divert_right.left_forward) / float(sensor_divert_right.right_forward)
        self.ratio_per_deg_faced_right -= self.ratio_faced_front
        self.ratio_per_deg_faced_right  /= calib_divert_angle
        
        # Right-about-face
        self.turn(180 - calib_divert_angle, self.turn_velocity)
        rospy.sleep(self.interval_time)
        sensor_none = self.sensor_values
        self.show_lightsensors('sensor_none: ', sensor_none)

        # Position adjustment
        sensor_none = self.sensor_values
        self.intensity_slowdown = max(sensor_stop_front_target.sum_forward * 0.01, sensor_none.sum_forward * 1.0)
        self.intensity_target = sensor_stop_front_target.sum_forward
        self.intensity_stop = self.intensity_target * 0.9
        self.intensity_exist_side = max(sensor_stop_front_target.left_side, sensor_stop_front_target.right_side) * 0.1
        print 'intensity_slowdown = ', self.intensity_slowdown
        print 'intensity_target = ', self.intensity_target
        print 'intensity_exist_side = ', self.intensity_exist_side
        
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

    def positioning(self):
        tolerance = 0.01    # Tolerance rate of positioning

        # Adjust position.
        tol_ideal = self.intensity_target * tolerance
        result = False
        for x in range(0, 4):
            rospy.sleep(self.interval_time)
            s = self.sensor_values
            if (s.sum_forward < self.intensity_target + tol_ideal) and (s.sum_forward > self.intensity_target - tol_ideal):
                print 'Succeeded positioning.'
                result = True
                break
            dist = (s.sum_forward - self.intensity_target) / self.intensity_per_mm
            print 'distance from front wall = ', dist, ' mm'
            self.move(-dist, self.slow_velocity)
        if not result:
            print 'Failed positioning.'

        # Adjust direction
        result = False
        for x in range(0, 4):
            rospy.sleep(self.interval_time)
            s = self.sensor_values
            #self.show_lightsensors('dir: ', s)
            ratio_divert = float(s.left_forward) / float(s.right_forward) - self.ratio_faced_front
            if (ratio_divert < tolerance) and (ratio_divert > -tolerance):
                print 'Succeeded direction adjustment.'
                result = True
                break
            if ratio_divert < 0:    # Faced left.
                turn_deg = ratio_divert / self.ratio_per_deg_faced_left
            else:                   # Faced right.
                turn_deg = ratio_divert / self.ratio_per_deg_faced_right
            #print 'turn_deg = ', turn_deg, 'ratio_divert = ', ratio_divert, 'target = ', self.ratio_faced_front
            self.turn(-turn_deg * self.turn_slow_correct, self.turn_slow_velocity)
        if not result:
            print 'Failed direction adjustment.'

    def run(self):
        # motion parameter
        rate_normal = 10    # Normal sampling rate [Hz]
        rate_fine = 20      # Fine sampling rate for positioning [Hz]
        accel = 40          # Acceleration
        decel = 80          # Deceleration
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

            if s.left_side < self.intensity_exist_side:    # Left wall doesn't exist.
                rospy.set_param('lightsensors_freq', rate_fine)
                rate = rospy.Rate(rate_fine)
                while data.linear.x > 0:
                    s = self.sensor_values
                    data.linear.x -= decel
                    if data.linear.x < vel_min:
                        data.linear.x = vel_min
                    if self.sensor_values.left_side >= self.intensity_exist_side:
                        data.linear.x = 0
                    self.show_lightsensors('left wall: ', s)
                    self.cmd_vel.publish(data)
                    rate.sleep()
                self.move(self.sidewall_center, self.slow_velocity)
                rate.sleep()

                if s.left_forward >= self.intensity_slowdown and s.right_forward >= self.intensity_slowdown:    # Detected a front wall.
                    self.positioning()

                self.turn(-90, self.turn_velocity)
                rate.sleep()
                rospy.set_param('lightsensors_freq', rate_normal)
                rate = rospy.Rate(rate_normal)

            elif s.left_forward >= self.intensity_slowdown and s.right_forward >= self.intensity_slowdown:    # Detected a front wall.
                rospy.set_param('lightsensors_freq', rate_fine)
                rate = rospy.Rate(rate_fine)
                wall_left = True
                wall_right = True
                while data.linear.x > 0:
                    s = self.sensor_values

                    if s.left_forward < self.intensity_slowdown or s.right_forward < self.intensity_slowdown:
                        print 'Restart'
                        break

                    if s.left_side < self.intensity_exist_side:
                        wall_left = False
                    if s.right_side < self.intensity_exist_side:
                        wall_right = False
                    
                    data.linear.x -= decel
                    if data.linear.x < vel_min:
                        data.linear.x = vel_min
                    if s.sum_forward >= self.intensity_stop:
                        data.linear.x = 0       # Arrived at the face of front wall.
                    self.show_lightsensors('front wall: ', s)
                    self.cmd_vel.publish(data)
                    rate.sleep()

                self.positioning()

                # Turn
                if not wall_left:
                    self.turn(-90,self.turn_velocity)
                elif not wall_right:
                    self.turn(90, self.turn_velocity)
                else:
                    if self.turn_dir:
                        self.turn(-180, self.turn_velocity)
                    else:
                        self.turn(180, self.turn_velocity)
                    self.turn_dir = not self.turn_dir
                rospy.set_param('lightsensors_freq', rate_normal)
                rate = rospy.Rate(rate_normal)
                rospy.sleep(self.interval_time)
                    
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
    maze.calibration()
    maze.run()
