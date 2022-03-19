#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class AKROSMeshPublisher():
    def __init__(self):
        self._marker_pub = rospy.Publisher("mesh_array", MarkerArray, queue_size=8)
        
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
        
    def run(self):
        while not rospy.is_shutdown():
            self._marker_pub.publish(self._marker_array_msg)
            rospy.rostime.wallsleep(1.0)
            
def main():
    try:
        rospy.init_node('mesh_publisher')
        pub = AKROSMeshPublisher()
        pub.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("mesh_publisher node interrupted")
            
if __name__ == "__main__":
    main()