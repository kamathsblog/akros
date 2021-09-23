#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status

class TwistMixer():
    def __init__(self):

        rospy.init_node('twist_mux')
        
        self._min_interval  = 0.01
        self._last_pub_time = rospy.Time()

        self.ros_sub_assisted = rospy.Subscriber("assisted/cmd_vel", Twist, self.set_assisted_twist, queue_size=1)
        self.ros_sub_teleop   = rospy.Subscriber("teleop/cmd_vel", Twist, self.set_teleop_twist, queue_size=1)
        self.ros_sub_auto     = rospy.Subscriber("auto/cmd_vel", Twist, self.set_auto_twist, queue_size=1)
        self.ros_sub_status   = rospy.Subscriber("status", Status, self.set_status, queue_size=1)

        self._assisted       = Twist()
        self._teleop         = Twist()
        self._auto           = Twist()
        self._zero           = Twist()
        self._zero.linear.x  = 0.0;
        self._zero.linear.y  = 0.0;
        self._zero.angular.z = 0.0;
        self._prev           = Status()
        self._auto_mode      = False # auto mode
        self._assist_mode    = False # teleop_assist (during teleop mode only)
        self._estop          = False # operational

        self._pub_mux = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    
    def set_assisted_twist(self, msg):
        self._assisted = msg
    
    def set_teleop_twist(self, msg):
        self._teleop = msg
        
    def set_auto_twist(self, msg):
        self._auto = msg
        
    def set_status(self, msg):
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval: return
        
        if msg.button_cross and not self._prev.button_cross:
            self._auto_mode = not self._auto_mode
            
        if msg.button_square and not self._prev.button_square:
            self._assist_mode = not self._assist_mode
            
        if msg.button_circle and not self._prev.button_circle:
            self._estop = not self._estop

        self._prev = msg
        self._last_pub_time = now
        
    def run(self):
        rate = rospy.Rate(1000) # 1Khz   
        while not rospy.is_shutdown():
            
            if self._estop:
                self._pub_mux.publish(self._zero)
            else:
                if self._auto_mode:
                    self._pub_mux.publish(self._auto)
                else:
                    if self._assist_mode:
                        self._pub_mux.publish(self._assisted)
                    else:
                        self._pub_mux.publish(self._teleop)

            rate.sleep()
            
if __name__ == "__main__":
    mux = TwistMixer()
    mux.run()