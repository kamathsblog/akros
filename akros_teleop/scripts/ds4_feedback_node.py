#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Feedback, Status


class Handler(object):
    def __init__(self, status_topic='status', feedback_topic='set_feedback'):
        self._min_interval = 0.1
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            'r': 0,
            'g': 0,
            'b': 0,
        }
        self._rumble = False
        self._rumble_val = 0
        
        self._auto_mode = False #auto mode
        self._assist_mode = False # teleop_assist (during teleop only)
        self._estop = False #operational

        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.cb_status, queue_size=1)

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval: return

        feedback = Feedback()
        
        # Turn on/off rumble with PS button 
        if msg.button_ps and not self._prev.button_ps:
            self._rumble = not self._rumble
            
        if msg.button_cross and not self._prev.button_cross:
            self._auto_mode = not self._auto_mode
            self._rumble = False
            
        if msg.button_circle and not self._prev.button_circle:
            self._estop = not self._estop
            
        if msg.button_square and not self._prev.button_square:
            self._assist_mode = not self._assist_mode
            if self._assist_mode: self._rumble = True
            else: self._rumble = False

        feedback.set_led = True
        
        if self._estop:
            feedback.led_r = 0
            feedback.led_g = 0
            feedback.led_b = 0
        else:
            if self._auto_mode:
                feedback.led_r = 1
                feedback.led_g = 1
                feedback.led_b = 1
            else:
                if self._assist_mode:
                    self._led['r'] = abs(msg.axis_left_y + msg.button_dpad_up - msg.button_dpad_down)* 0.90 + 0.10 #translation in ROS x
                    self._led['g'] = abs(msg.axis_left_x + msg.button_dpad_left - msg.button_dpad_right)* 0.90 + 0.10#translation in ROS y
                    self._led['b'] = abs(msg.axis_right_x)* 0.90 + 0.10 # rotation around ROS z direction
                else:
                    self._led['r'] = abs(msg.axis_left_y + msg.button_dpad_up - msg.button_dpad_down) #translation in ROS x
                    self._led['g'] = abs(msg.axis_left_x + msg.button_dpad_left - msg.button_dpad_right) #translation in ROS y
                    self._led['b'] = abs(msg.axis_right_x) # rotation around ROS z direction
                
                feedback.set_rumble = True    
                if self._rumble:
                    feedback.rumble_small = (abs(msg.axis_left_y) + abs(msg.axis_left_x) + abs(msg.axis_right_x) + abs(msg.button_dpad_up - msg.button_dpad_down) + abs(msg.button_dpad_left - msg.button_dpad_right)) / 4
                else:
                    feedback.rumble_small = 0
                
                feedback.led_r = self._led['r']
                feedback.led_g = self._led['g']
                feedback.led_b = self._led['b']
            
                
                

        self._pub_feedback.publish(feedback)
        self._prev = msg
        self._last_pub_time = now

def main():
    rospy.init_node('ds4_feedback')

    Handler()

    rospy.spin()


if __name__ == '__main__':
    main()
