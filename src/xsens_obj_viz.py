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

class ObjectViz(object):
    

    def __init__(self, marker_pub, obj_id):
        self._object_pose    = []
        self._marker_pub     = marker_pub
        self._obj_id         = obj_id
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg):
        point = msg.position
        # point.z = point.z - 0.4
        point.z = point.z
    
        self._object_pose = point;
        self.viz_cube()

    def viz_cube(self):
       marker = Marker()
       # marker.header.frame_id = "/world"
       marker.header.frame_id = "/xsens_origin"
       # marker.header.frame_id = "/xsens_origin_flipped"
       marker.type = marker.CUBE
       marker.action = marker.ADD
       marker.scale.x = 0.025
       marker.scale.y = 0.025
       marker.scale.z = 0.025
       marker.color.a = 1.0
       if self._obj_id == 0: 
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
       elif self._obj_id == 1:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
       elif self._obj_id == 2: 
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

       marker.pose.orientation.w = 1.0
       marker.pose.position.x = self._object_pose.x
       marker.pose.position.y = self._object_pose.y
       marker.pose.position.z = self._object_pose.z
       self._marker_pub.publish(marker)
    
    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Marker Viz Node")
        rospy.sleep(1)

def main():
    rospy.init_node('obj_viz')
    wait_for_time()
    marker_publisher_rh = rospy.Publisher('xsens_rh_marker', Marker, queue_size=1)
    objectViz_rh = ObjectViz(marker_publisher_rh, obj_id = 1)
    rospy.Subscriber('/xsens_rh_pose', Pose, objectViz_rh.callback)

    marker_publisher_lh = rospy.Publisher('xsens_lf_marker', Marker, queue_size=1)
    objectViz_lh = ObjectViz(marker_publisher_lh, obj_id = 2)
    rospy.Subscriber('/xsens_lh_pose', Pose, objectViz_lh.callback)

    marker_publisher_pelvis = rospy.Publisher('xsens_pelvis_marker', Marker, queue_size=1)
    objectViz_pelvis = ObjectViz(marker_publisher_pelvis, obj_id = 0)
    rospy.Subscriber('/xsens_pelvis_pose', Pose, objectViz_pelvis.callback)

    rospy.spin()

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       objectViz.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')


if __name__ == '__main__':
    main()