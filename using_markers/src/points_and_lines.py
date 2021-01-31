#!/usr/bin/python
import rospy
import copy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3
import numpy as np


def points_and_lines():
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('points_and_lines', anonymous=True)
    rate = rospy.Rate(30)  # 30Hz

    f = 0.0
    while not rospy.is_shutdown():
        points = Marker(
            header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
            ns="points_and_lines",
            action=Marker.ADD,
            id=0,
            type=Marker.POINTS,
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0)
        )
        # POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2
        points.scale.y = 0.2
        points.pose.orientation.w = 1.0

        line_strip = Marker(
            header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
            ns="points_and_lines",
            action=Marker.ADD,
            id=1,
            type=Marker.LINE_STRIP,
            # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            color=ColorRGBA(0.0, 0.0, 1.0, 1.0)
        )
        line_strip.scale.x = 0.1
        line_strip.pose.orientation.w = 1.0

        line_list = Marker(
            header=Header(frame_id='my_frame', stamp=rospy.Time.now()),
            ns="points_and_lines",
            action=Marker.ADD,
            id=2,
            type=Marker.LINE_LIST,
            color=ColorRGBA(1.0, 0.0, 0.0, 1.0)
        )
        line_list.scale.x = 0.1
        line_list.pose.orientation.w = 1.0

        # Create the vertices for the points and lines
        for i in range(100):
            y = 5 * np.math.sin(f + i / 100.0 * 2 * np.math.pi)
            z = 5 * np.math.cos(f + i / 100.0 * 2 * np.math.pi)

            p = Point(i - 50, y, z)
            
            points.points.append(p)
            line_strip.points.append(p)

            # The line list needs two points for each line
            line_list.points.append(p)
            q = copy.deepcopy(p)
            q.z += 1.0
            line_list.points.append(q)

        marker_pub.publish(points)
        marker_pub.publish(line_strip)
        marker_pub.publish(line_list)

        rate.sleep()
        f += 0.04

if __name__ == '__main__':
    try:
        points_and_lines()
    except rospy.ROSInterruptException:
        pass
