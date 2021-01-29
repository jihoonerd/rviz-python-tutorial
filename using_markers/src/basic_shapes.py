#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker


def marker():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('basic_shapes', anonymous=True)
    rate = rospy.Rate(1)  # 10Hz

    shape = Marker.CUBE

    while not rospy.is_shutdown():
        marker = Marker(
            header=Header(frame_id="my_frame", stamp=rospy.Time.now()),
            ns="basic_shapes",
            id=0,
            type=shape,
            action=Marker.ADD,
            pose=Pose(Point(np.random.uniform(0, 3), np.random.uniform(0, 3), np.random.uniform(0, 3)),
                      Quaternion(0, 0, 0, 1)),
            scale=Vector3(np.random.uniform(0.5, 3), np.random.uniform(
                0.5, 3), np.random.uniform(0.5, 3)),
            color=ColorRGBA(np.random.uniform(0, 1), np.random.uniform(
                0, 1), np.random.uniform(0, 1), 1),
            lifetime=rospy.Duration()
        )

        while pub.get_num_connections() < 1:
            rospy.logwarn("Please create a subscriber to the marker")
            rospy.sleep(1)
        pub.publish(marker)

        if shape is Marker.CUBE:
            shape = Marker.SPHERE
        elif shape is Marker.SPHERE:
            shape = Marker.ARROW
        elif shape is Marker.ARROW:
            shape = Marker.CYLINDER
        else:
            shape = Marker.CUBE

        rate.sleep()


if __name__ == '__main__':
    try:
        marker()
    except rospy.ROSInterruptException:
        pass
