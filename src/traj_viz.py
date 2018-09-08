#!/usr/bin/env python
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import math
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def distance(p1, p2):
    """Returns the distance between two Points/Vector3s.
    """
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class TaskTrajectory(object):
    DISTANCE_THRESHOLD = 0.05

    def __init__(self, marker_pub):
        self._trajectory = []
        self._marker_pub = marker_pub

    def callback(self, msg):
        point = msg.position
        if (len(self._trajectory) == 0 or
                distance(self._trajectory[-1], point) > TaskTrajectory.DISTANCE_THRESHOLD):
            self._trajectory.append(point)
            self.viz_breadcrumb()
            self.viz_path()

    def viz_breadcrumb(self):
        msg = Marker(
            type=Marker.SPHERE_LIST,
            ns='breadcrumb',
            id=len(self._trajectory),
            points=self._trajectory,
            scale=Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id='world'),
            color=ColorRGBA(1.0, 0.0, 0.0, 0.0))
        self._marker_pub.publish(msg)

    def viz_path(self):
        msg = Marker(
            type=Marker.LINE_STRIP,
            ns='path',
            id=len(self._trajectory),
            points=self._trajectory,
            scale=Vector3(0.002, 0.002, 0.02),
            header=Header(frame_id='world'),
            color=ColorRGBA(0.5, 0.5, 0.5, 1.0))
        self._marker_pub.publish(msg)


def main():
    rospy.init_node('traj_viz')
    wait_for_time()
    marker_publisher = rospy.Publisher(
        'trajectory_marker', Marker, queue_size=5)
    trajectory = TaskTrajectory(marker_publisher)
    rospy.Subscriber('/lwr/ee_pose', Pose, trajectory.callback)

    rospy.spin()


if __name__ == '__main__':
    main()