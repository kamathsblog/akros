#!/usr/bin/python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from akros_msgs.msg import Mode
from std_msgs.msg import UInt32

class ArduinoInterface():
    def __init__(self):

        self._sub_raw_vec = rospy.Subscriber("raw_vec", Point, self.set_raw_vec, queue_size=1)
        self._sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.set_cmd_vel, queue_size=1)
        self._sub_mode    = rospy.Subscriber("teleop/mode", Mode, self.set_mode, queue_size=1)

        self._raw_vec   = Point()
        self._odom      = Odometry()
        self._cmd_vec   = Point()
        self._cmd_vel   = Twist()
        self._sub_time  = rospy.Time()
        self._x         = 0.0
        self._y         = 0.0
        self._th        = 0.0
        self._pose      = Pose()
        self._transform = TransformStamped()
        self._mode      = Mode()
        
        self._odom_frame_id = rospy.get_param('odom_frame_id', 'enc_odom_frame')
        self._pose_frame_id = rospy.get_param('pose_frame_id', 'enc_pose_frame')
        self._frequency     = rospy.get_param('frequency', 80)
        
        self._pub_cmd_vec  = rospy.Publisher("cmd_vel/vector", Point, queue_size=1)
        self._pub_enc_odom = rospy.Publisher("enc/odom", Odometry, queue_size=1)
        self._pub_mode     = rospy.Publisher("cmd_vel/mode", UInt32, queue_size=1)
        self._broadcaster  = tf.TransformBroadcaster()
    
    def set_raw_vec(self, msg):
        self._raw_vec = msg
        self._sub_time = rospy.Time.now()
    
    def set_cmd_vel(self, msg):
        self._cmd_vel = msg
        
    def set_mode(self, msg):
        self._mode = msg
        
    def run(self):
        rate = rospy.Rate(self._frequency) # 100Hz   
        while not rospy.is_shutdown():
        
            self._odom.header.stamp = rospy.Time.now()
            self._odom.header.frame_id = self._odom_frame_id
            self._odom.child_frame_id = self._pose_frame_id
            
            dt = self._odom.header.stamp.to_sec() - self._sub_time.to_sec()
            self._th += self._raw_vec.z*dt
            if self._th > 2*math.pi or self._th < -2*math.pi : self._th = 0
            quat = tf.transformations.quaternion_from_euler(0, 0, self._th)
            self._x += (self._raw_vec.x*math.cos(self._th) - self._raw_vec.y*math.sin(self._th))*dt
            self._y += (self._raw_vec.y*math.cos(self._th) + self._raw_vec.x*math.sin(self._th))*dt

            self._odom.pose.pose.position.x = self._x
            self._odom.pose.pose.position.y = self._y
            self._odom.pose.pose.orientation = Quaternion(*quat)
            #covariance?
            
            self._odom.twist.twist.linear.x = self._raw_vec.x
            self._odom.twist.twist.linear.y = self._raw_vec.y
            self._odom.twist.twist.angular.z = self._raw_vec.z
            #covariance?
            
            self._cmd_vec.x = self._cmd_vel.linear.x
            self._cmd_vec.y = self._cmd_vel.linear.y
            self._cmd_vec.z = self._cmd_vel.angular.z
            
            mode_out = 100000;
            if self._mode.estop:   mode_out += 1
            if self._mode.auto_t:  mode_out += 10
            if self._mode.play_wp: mode_out += 100
            if self._mode.play_t:  mode_out += 1000
            if self._mode.record:  mode_out += 10000
            
            self._broadcaster.sendTransform((self._x, self._y, 0),
                                            quat,
                                            rospy.Time.now(),
                                            self._pose_frame_id,
                                            self._odom_frame_id)
            self._pub_cmd_vec.publish(self._cmd_vec)
            self._pub_enc_odom.publish(self._odom)
            self._pub_mode.publish(mode_out);

            rate.sleep()
            
def main():
    try:
        rospy.init_node('arduino_interface')
        arduino = ArduinoInterface()
        arduino.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("arduino_interface node interrupted")
            
if __name__ == "__main__":
    main()