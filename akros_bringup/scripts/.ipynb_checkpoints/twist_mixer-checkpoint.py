#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from akros_msgs.msg import Mode

class TwistMixer():
    def __init__(self):

        self._min_interval  = 0.01
        self._last_pub_time = rospy.Time()

        self.ros_sub_assisted = rospy.Subscriber("assisted_cmd_vel", Twist, self.set_assisted_twist, queue_size=1)
        self.ros_sub_teleop   = rospy.Subscriber("teleop_cmd_vel", Twist, self.set_teleop_twist, queue_size=1)
        self.ros_sub_auto     = rospy.Subscriber("auto_cmd_vel", Twist, self.set_auto_twist, queue_size=1)
        self.ros_sub_mode     = rospy.Subscriber("mode", Mode, self.set_mode, queue_size=1)

        self._assisted       = Twist()
        self._teleop         = Twist()
        self._auto           = Twist()
        self._zero           = Twist()
        self._zero.linear.x  = 0.0;
        self._zero.linear.y  = 0.0;
        self._zero.angular.z = 0.0;
        self._mode           = Mode()

        self._pub_mux = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    
    def set_assisted_twist(self, msg):
        self._assisted = msg
    
    def set_teleop_twist(self, msg):
        self._teleop = msg
        
    def set_auto_twist(self, msg):
        self._auto = msg
        
    def set_mode(self, msg):
        self._mode = msg
        
    def run(self):
        rate = rospy.Rate(1000) # 1Khz   
        while not rospy.is_shutdown():
            
            if self._mode.estop:
                self._pub_mux.publish(self._zero)
            else:
                if self._mode.auto:
                    self._pub_mux.publish(self._auto)
                else:
                    if self._mode.assist:
                        self._pub_mux.publish(self._assisted)
                    else:
                        self._pub_mux.publish(self._teleop)

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