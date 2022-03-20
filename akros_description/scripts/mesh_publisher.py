#! /usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

WHEELS_X_DISTANCE = 0.093 #m
WHEELS_Y_DISTANCE = 0.210 #m
WHEEL_DIAMETER = 0.0762 #m 3"

class AKROSMeshPublisher():
    def __init__(self):
        self._sub_raw_vec = rospy.Subscriber("raw_vec", Point, self.set_raw_vec, queue_size=1)
        self._marker_pub  = rospy.Publisher("mesh_array", MarkerArray, queue_size=8)
        self._state_pub   = rospy.Publisher("joint_states", JointState, queue_size=8)
        
        self._raw_vec         = Point()
        self._joint_state_msg = JointState()
        self._velocities      = [0.0, 0.0, 0.0, 0.0] # lf, lb, rf, rb
        self._positions       = [0.0, 0.0, 0.0, 0.0] # lf, lb, rf, rb
        self._vel_lf          = 0.0
        self._vel_lb          = 0.0
        self._vel_rf          = 0.0
        self._vel_rb          = 0.0
        self._pos_lf          = 0.0
        self._pos_lb          = 0.0
        self._pos_rf          = 0.0
        self._pos_rb          = 0.0
        
        self._marker_array_msg = MarkerArray()
        self._num_markers      = 8
        
        self._frame_base      = rospy.get_param('frame_base',      'base_link')
        self._frame_laser     = rospy.get_param('frame_laser',     'laser_link')
        self._frame_t265      = rospy.get_param('frame_t265',      't265_pose_frame')
        self._frame_footprint = rospy.get_param('frame_footprint', 'base_footprint')
        self._frame_wheel_lf  = rospy.get_param('frame_wheel_lf',  'wheel_lf')
        self._frame_wheel_lb  = rospy.get_param('frame_wheel_lb',  'wheel_lb')
        self._frame_wheel_rf  = rospy.get_param('frame_wheel_rf',  'wheel_rf')
        self._frame_wheel_rb  = rospy.get_param('frame_wheel_rb',  'wheel_rb')
        
        self._url_base      = rospy.get_param('url_base',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/navigation_module_centered.stl")
        self._url_laser     = rospy.get_param('url_laser',     "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/ld06.stl")
        self._url_t265      = rospy.get_param('url_t265',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/t265.stl")
        self._url_footprint = rospy.get_param('url_footprint', "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/base_module_centered.stl")
        self._url_wheel_lf  = rospy.get_param('url_wheel_lf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_left_front.stl")
        self._url_wheel_lb  = rospy.get_param('url_wheel_lb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_left_back.stl")
        self._url_wheel_rf  = rospy.get_param('url_wheel_rf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_right_front.stl")
        self._url_wheel_rb  = rospy.get_param('url_wheel_rb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_right_back.stl")
        
        for i in range(self._num_markers):
            self._marker = Marker()
            self._marker.header.stamp = rospy.Time.now()
            self._marker.ns = ""
            self._marker.type = 10
            self._marker.id = i
            self._marker.action = 0
            self._marker.mesh_use_embedded_materials = False
            self._marker.scale.x = 1
            self._marker.scale.y = 1
            self._marker.scale.z = 1
            self._marker.pose.position.x = 0
            self._marker.pose.position.y = 0
            self._marker.pose.position.z = 0
            self._marker.pose.orientation.x = 0.0
            self._marker.pose.orientation.y = 0.0
            self._marker.pose.orientation.z = 0.0
            self._marker.pose.orientation.w = 1.0
            self._marker.frame_locked = True
            if i==0 or i==3:
                self._marker.color.r = 1.0
                self._marker.color.g = 1.0
                self._marker.color.b = 1.0
                self._marker.color.a = 0.75
            else:
                self._marker.color.r = 0.2078
                self._marker.color.g = 0.2549
                self._marker.color.b = 0.2784
                self._marker.color.a = 0.95
            if i==0:
                self._marker.header.frame_id = self._frame_base
                self._marker.mesh_resource = self._url_base
            elif i==1:
                self._marker.header.frame_id = self._frame_laser
                self._marker.mesh_resource = self._url_laser
            elif i==2:
                self._marker.header.frame_id = self._frame_t265
                self._marker.mesh_resource = self._url_t265
            elif i==3:
                self._marker.header.frame_id = self._frame_footprint
                self._marker.mesh_resource = self._url_footprint
            elif i==4:
                self._marker.header.frame_id = self._frame_wheel_lf
                self._marker.mesh_resource = self._url_wheel_lf
            elif i==5:
                self._marker.header.frame_id = self._frame_wheel_lb
                self._marker.mesh_resource = self._url_wheel_lb
            elif i==6:
                self._marker.header.frame_id = self._frame_wheel_rf
                self._marker.mesh_resource = self._url_wheel_rf
            elif i==7:
                self._marker.header.frame_id = self._frame_wheel_rb
                self._marker.mesh_resource = self._url_wheel_rb
            self._marker_array_msg.markers.append(self._marker);
            
    def set_raw_vec(self, msg):
        self._raw_vec = msg
        
    def run(self):
        while not rospy.is_shutdown():
            #compute average rad/s from raw velocities using robot geometry
            avg_rps_x = self._raw_vec.x * (2/WHEEL_DIAMETER)
            avg_rps_y = self._raw_vec.y * (2/WHEEL_DIAMETER)
            avg_rps_a = self._raw_vec.z * ((WHEELS_X_DISTANCE + WHEELS_Y_DISTANCE)/WHEEL_DIAMETER)
  
            #compute individual motor RPMs from average rad/s
            self._velocities[0] = avg_rps_x - avg_rps_y - avg_rps_a; #lf
            self._velocities[1] = avg_rps_x + avg_rps_y - avg_rps_a; #lb
            self._velocities[3] = avg_rps_x - avg_rps_y + avg_rps_a; #rb
            self._velocities[2] = avg_rps_x + avg_rps_y + avg_rps_a; #rf
            
            for j in range(4):
                self._positions[j] += self._velocities[j]/5 #1/5 second frequency
                #limit position between -pi and pi
                if self._positions[j] > math.pi: self._positions[j] = -1*math.pi
                if self._positions[j] < -1*math.pi: self._positions[j] = math.pi
            
            self._joint_state_msg.header = Header()
            self._joint_state_msg.header.frame_id = self._frame_footprint
            self._joint_state_msg.header.stamp = rospy.Time.now()
            self._joint_state_msg.name = ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']
            self._joint_state_msg.velocity = self._velocities
            self._joint_state_msg.position = self._positions
            self._joint_state_msg.effort   = []
            
            self._state_pub.publish(self._joint_state_msg)
            self._marker_pub.publish(self._marker_array_msg)
            rospy.rostime.wallsleep(0.20)
            
def main():
    try:
        rospy.init_node('mesh_publisher')
        pub = AKROSMeshPublisher()
        pub.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("mesh_publisher node interrupted")
            
if __name__ == "__main__":
    main()