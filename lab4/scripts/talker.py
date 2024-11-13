#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
import numpy as np

class HuskyHighlevelController:
    def __init__(self):
        rospy.init_node('husky_highlevel_controller', anonymous=True)
        
        # Get params from config file
        self.p_vel = rospy.get_param("controller_gain", 1.0)
        topic = rospy.get_param("subscriber_topic", "/scan")
        queue_size = rospy.get_param("queue_size", 10)

        # Create subscriber and publishers
        self.subscriber = rospy.Subscriber(topic, LaserScan, self.laser_callback, queue_size=queue_size)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.viz_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)

        # Initialize variables
        self.p_ang = 1.0
        self.pillar_pos = [0, 0]
        self.msg = Twist()
        self.marker = Marker()  # Initialize marker here
        self.min_distance_threshold = 0.1  # Minimum distance threshold
        
        # Initialize pillar marker in RViz
        self.init_pillar_marker()

        rospy.loginfo("Husky highlevel controller node launched!")

        # Initialize linear speed
        self.set_vel(3.0, "forward")

    def set_vel(self, vel, dof):
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def drive_husky(self):
        self.vel_pub.publish(self.msg)

    def adjust_speed(self, dist):
        if dist < 0.16:
            self.set_vel(0.0,"forward")
            self.set_vel(0.0,'ang')
            rospy.signal_shutdown("Distance threshold exceeded.")
        else:
            vel = self.p_vel * (dist - 0.17)  
            vel = max(0.0, min(vel, 5.0))  # limit speed between 0 and 5
            self.set_vel(vel, "forward")

    def adjust_heading(self, ang):
        diff = -ang
        if abs(diff) < 0.5:
            self.set_vel(0.0,'ang')
        else:
            self.set_vel(self.p_ang*5, "ang")

    def viz_pillar(self):
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.viz_pub.publish(self.marker)

    def laser_callback(self, msg):
        # Filter out invalid range values
        
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > self.min_distance_threshold]
        # rospy.loginfo(f"A is {msg.ranges}")
            
        if valid_ranges:

            ranges=np.array(valid_ranges)
            # rospy.loginfo(f"range is {ranges}")
            
            diff=np.diff(ranges)
            max_diff=np.argmax(np.abs(diff))

            dist=min(ranges[max_diff],ranges[max_diff+1])
            rospy.loginfo(f"chajudian A is {ranges[max_diff]},chajudian B is {ranges[max_diff+1]}")
            

            ang = msg.angle_min + msg.angle_increment * (max_diff-1)

            rospy.loginfo(f"Pillar is {dist:.2f}m away at {math.degrees(ang):.2f} degrees")
            
            
            self.pillar_pos[0] = dist * math.cos(ang)
            self.pillar_pos[1] = dist * math.sin(ang)
            rospy.loginfo(f"Pillar's coordinate to Husky is [{self.pillar_pos[0]:.2f}, {self.pillar_pos[1]:.2f}]")
            
            self.adjust_heading(ang)
            self.adjust_speed(dist)
            self.drive_husky()

            self.viz_pillar()
            
        else:
            rospy.logwarn("No valid range data received or all are too close.")

    def init_pillar_marker(self):
        self.marker.header.frame_id = "base_laser"
        self.marker.ns = "pillar"
        self.marker.id = 1
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 2
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

if __name__ == '__main__':
    try:
        controller = HuskyHighlevelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
