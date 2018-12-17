#!/usr/bin/env python
import unittest, rostest
import rosnode, rospy
import time

class MazeTraceTest(unittest.TestCase):
    def set_and_get(self, lf, ls, rs, rf):
        with open("/dev/rtlightsensor0", "w") as f:
            f.write("%d %d %d %d\n" % (rf, rs, ls, lf))

        time.sleep(0.3)

        with open("/dev/rtmotor_raw_l0", "r") as lf,\
             open("/dev/rtmotor_raw_r0", "r") as rf:
            left = int(lf.readline().rstrip())
            right = int(rf.readline().rstrip())

        return left, right

    def test_io(self):
        # accelerate: forward sensor(400 + 100) <= th_slowdown(500)
        left, right = self.set_and_get(400, 200, 200, 100)
        self.assertTrue(left == right > 0, "don't run")

        # curves to right if machine gets being close to left wall(300).
        left, right = self.set_and_get(400, 300, 200, 100)
        self.assertTrue(left > right > 0, "don't curve to right")

        # curves to left if machine gets being close to right wall(300).
        left, right = self.set_and_get(400, 200, 300, 100)
        self.assertTrue(left < right > 0, "don't curve to left")

        time.sleep(2)

        # run on maximum velocity. forward sensor(400 + 100)
        left_max, right_max = self.set_and_get(400, 300, 300, 100)
        self.assertTrue(left_max == right_max > 0, "don't run straight")

        # don't steer if no wall on left.
        left, right = self.set_and_get(0, 199, 200, 0)
        self.assertTrue(0 < left == right, "curve wrongly")

        # don't steer if no wall on right.
        left, right = self.set_and_get(0, 200, 199, 0)
        self.assertTrue(0 < left == right, "curve wrongly")

        # side sensor is not a trigger of stop.
        left, right = self.set_and_get(0, 199, 1000, 0)
        self.assertTrue(left == right != 0, "stop wrongly by side sensors")

        # side sensor is not a trigger of stop.
        left, right = self.set_and_get(0, 1000, 199, 0)
        self.assertTrue(left == right != 0, "stop wrongly by side sensors")


        # slow down: forward sensor(501) > th_slowdown(500)
        left, right = self.set_and_get(400, 300, 200, 101)
        self.assertTrue((left < left_max) and (right < right_max), "can't slow down")

        # stop immediately: forward sensor(600 + 400) >= th_stop(1000)
        left, right = self.set_and_get(600, 300, 200, 400)
        self.assertTrue(left == right == 0, "can't stop")

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('travis_test_maze_trace')
    rostest.rosrun('pimouse_run_maze', 'travis_test_maze_trace', MazeTraceTest)
