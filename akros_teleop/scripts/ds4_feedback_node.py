#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Feedback, Status
from geometry_msgs.msg import Twist
from akros_msgs.msg import Mode

class Handler(object):
    def __init__(self, status_topic='status', feedback_topic='set_feedback', mode_topic='mode', twist_topic='cmd_vel'):
        self._min_interval = 0.01
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            'r': 0,
            'g': 0,
            'b': 0,
        }
        
        self._mode = Mode()
        self._cmd = Twist()
        self._mode.estop = False
        self._mode.rumble = False
        self._mode.auto = False
        self._mode.assist = False
        self._mode.play = False
        self._mode.store = False
        
        self._rumble_val = 0
        
        self._scales = rospy.get_param('~scales')
        self._scale_x = self._scales['linear']['x']
        self._scale_y = self._scales['linear']['y']
        self._scale_z = self._scales['angular']['z']
        
        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        self._pub_mode = rospy.Publisher(mode_topic, Mode, queue_size=1)
        rospy.Subscriber(twist_topic, Twist, self.cb_twist, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.cb_status, queue_size=1)
        
    def cb_twist(self, msg):
        self._cmd = msg

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval: return

        feedback = Feedback()
        
        if msg.button_circle and not self._prev.button_circle:
            self._mode.estop = not self._mode.estop
            
        if msg.button_ps and not self._prev.button_ps:
            self._mode.rumble = not self._mode.rumble
            
        if msg.button_cross and not self._prev.button_cross:
            self._mode.auto = not self._mode.auto
            
        if msg.button_square and not self._prev.button_square:
            self._mode.assist = not self._mode.assist
            
        if msg.button_triangle and not self._prev.button_triangle:
            self._mode.play = not self._mode.play
            
        if msg.button_options and not self._prev.button_options:
            self._mode.store = not self._mode.store

        feedback.set_led = True
        
        if self._mode.estop:
            self._led['r'] = 0
            self._led['g'] = 0
            self._led['b'] = 0
            self._mode.rumble = False
            self._mode.auto = False
            self._mode.assist = False
            self._mode.play = False
            self._mode.store = False
        else:
            if self._mode.auto:
                self._mode.store = False
                if self._mode.play: #auto > playback mode - blueish-red 0xff007f
                    self._led['r'] = 1
                    self._led['g'] = 0
                    self._led['b'] = 0.5
                else: #normal auto mode - greenish-blue 0x007fff
                    self._led['r'] = 0
                    self._led['g'] = 0.5
                    self._led['b'] = 1
                    
            else:
                self._mode.play = False
                if self._mode.store: #teleop > store mode - reddish-green 0x7fff00
                    self._led['r'] = 0.5
                    self._led['g'] = 1
                    self._led['b'] = 0
                else: #normal teleop mode - rgb according to xyz
                    self._led['r'] = abs(self._cmd.linear.x/self._scale_x) #translation in ROS x
                    self._led['g'] = abs(self._cmd.linear.y/self._scale_y) #translation in ROS y
                    self._led['b'] = abs(self._cmd.angular.z/self._scale_z) # rotation around ROS z direction
                    
            if self._mode.assist:
                self._led['r'] = self._led['r']*0.90 + 0.10
                self._led['g'] = self._led['g']*0.90 + 0.10
                self._led['b'] = self._led['b']*0.90 + 0.10
                
            feedback.set_rumble = True    
            if self._mode.rumble:
                feedback.rumble_small = (abs(msg.axis_left_y) + abs(msg.axis_left_x) + abs(msg.axis_right_x) + abs(msg.button_dpad_up - msg.button_dpad_down) + abs(msg.button_dpad_left - msg.button_dpad_right)) / 4
            else:
                feedback.rumble_small = 0
                
            feedback.led_r = self._led['r']
            feedback.led_g = self._led['g']
            feedback.led_b = self._led['b']

        self._pub_feedback.publish(feedback)
        self._pub_mode.publish(self._mode)
        self._prev = msg
        self._last_pub_time = now

def main():
    try:
        rospy.init_node('ds4_feedback')
        Handler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ds4_feedback node interrupted")

if __name__ == '__main__':
    main()