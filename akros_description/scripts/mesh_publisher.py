#! /usr/bin/env python
import os.path
import rospkg
import rospy
from visualization_msgs.msg import Marker, MarkerArray

rospy.init_node('mesh_publisher')
marker_pub = rospy.Publisher("/akros_mesh", MarkerArray, queue_size = 5)

marker_array_msg = MarkerArray()
num_markers = 8

rospack = rospkg.RosPack()
my_path = rospack.get_path('akros_description')

for i in range(num_markers):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = ""
    marker.type = 10
    marker.id = i
    marker.action = 0
    marker.mesh_use_embedded_materials = True
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.frame_locked = True
    if i==0 or i==3:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    if i==0:
        marker.header.frame_id = "base_link"
        marker.mesh_resource = os.path.join(my_path, "meshes/navigation_module_centered.stl")
    elif i==1:
        marker.header.frame_id = "laser_frame"
        marker.mesh_resource = os.path.join(my_path, "meshes/ld06.stl")
    elif i==2:
        marker.header.frame_id = "t265_pose_frame"
        marker.mesh_resource = os.path.join(my_path, "meshes/t265.stl")
    elif i==3:
        marker.header.frame_id = "base_footprint"
        marker.mesh_resource = os.path.join(my_path, "meshes/base_module_centered.stl")
    elif i==4:
        marker.header.frame_id = "wheel_lf"
        marker.mesh_resource = os.path.join(my_path, "meshes/wheel_left_front.stl")
    elif i==5:
        marker.header.frame_id = "wheel_lb"
        marker.mesh_resource = os.path.join(my_path, "meshes/wheel_left_back.stl")
    elif i==6:
        marker.header.frame_id = "wheel_rf"
        marker.mesh_resource = os.path.join(my_path, "meshes/wheel_right_front.stl")
    elif i==7:
        marker.header.frame_id = "wheel_rb"
        marker.mesh_resource = os.path.join(my_path, "meshes/wheel_right_back.stl")
    marker_array_msg.markers.append(marker);

while not rospy.is_shutdown():
  marker_pub.publish(marker_array_msg)
  rospy.rostime.wallsleep(1.0)