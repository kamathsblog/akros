#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Feedback, Status
from akros2_msgs.msg import Mode
from std_msgs.msg import Bool

class Handler(object):
    def __init__(self, status_topic='status', estop_lock_topic='estop_lock', feedback_topic='set_feedback', mode_topic='mode'):
        self._min_interval = 0.05
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            'r': 0,
            'g': 0,
            'b': 0,
        }
        
        self._mode = Mode()
        self._mode.estop = False
        self._mode.auto_t = False
        
        self._rumble = False
        self._playback = 0
        self._rumble_val = 0
        
        self._scales = rospy.get_param('~scales')
        self._scale_x = self._scales['linear']['x']
        self._scale_y = self._scales['linear']['y']
        self._scale_z = self._scales['angular']['z']
        
        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        self._pub_mode = rospy.Publisher(mode_topic, Mode, queue_size=1)
        self._pub_estop_lock = rospy.Publisher(estop_lock_topic, Bool, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.cb_status, queue_size=1)

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
            self._rumble = not self._rumble
            
        if msg.button_cross and not self._prev.button_cross:
            self._mode.auto_t = not self._mode.auto_t

        feedback.set_led = True
        
        if self._mode.estop: # STOP! - red
            self._rumble = False
            self._mode.auto_t = False
            self._led['r'] = 1
            self._led['g'] = 0
            self._led['b'] = 0
        else:
            if self._mode.auto_t: # AUTO - blue 0x004bff
                self._led['r'] = 0
                self._led['g'] = 75/255
                self._led['b'] = 1                     
                    
            else: # TELEOP - blueish-white 0xf5f5ff
                self._led['r'] = 0
                self._led['g'] = 1
                self._led['b'] = 75/255
                    
        if self._rumble:
            feedback.set_rumble = True
            feedback.rumble_small = (abs(msg.axis_left_y) + abs(msg.axis_left_x) + abs(msg.axis_right_x) + abs(msg.button_dpad_up - msg.button_dpad_down) + abs(msg.button_dpad_left - msg.button_dpad_right)) / 4
        else:
            feedback.set_rumble = False
            feedback.rumble_small = 0
                
        feedback.led_r = self._led['r']
        feedback.led_g = self._led['g']
        feedback.led_b = self._led['b']

        self._pub_feedback.publish(feedback)
        self._pub_mode.publish(self._mode)
        self._pub_estop_lock.publish(self._mode.estop)
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