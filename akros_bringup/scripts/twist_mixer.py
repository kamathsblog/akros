#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from akros_msgs.msg import Mode

class TwistMixer():
    def __init__(self):

        self._sub_teleop   = rospy.Subscriber("teleop_cmd_vel", Twist, self.set_teleop_twist, queue_size=1)
        self._sub_auto     = rospy.Subscriber("auto_cmd_vel", Twist, self.set_auto_twist, queue_size=1)
        self._sub_mode     = rospy.Subscriber("mode", Mode, self.set_mode, queue_size=1)

        self._teleop         = Twist()
        self._auto           = Twist()
        self._zero           = Twist()
        self._mux_msg        = Twist()
        self._zero.linear.x  = 0.0
        self._zero.linear.y  = 0.0
        self._zero.angular.z = 0.0
        self._mode           = Mode()

        self._pub_mux = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    
    def set_teleop_twist(self, msg):
        self._teleop = msg
        
    def set_auto_twist(self, msg):
        self._auto = msg
        
    def set_mode(self, msg):
        self._mode = msg
        
    def run(self):
        rate = rospy.Rate(1000) #1000Hz   
        while not rospy.is_shutdown():
            
            if self._mode.estop:
                self._mux_msg = self._zero
            else:
                if self._mode.auto_t:
                    self._mux_msg = self._auto
                else:
                    self._mux_msg = self._teleop
                        
            self._pub_mux.publish(self._mux_msg)

            rate.sleep()
            
def main():
    try:
        rospy.init_node('twist_mux')
        mux = TwistMixer()
        mux.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("twist_mux node interrupted")
            
if __name__ == "__main__":
    main()