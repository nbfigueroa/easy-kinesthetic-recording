#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
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
    def __init__(self, marker_pub):
        self._object_pose    = []
        self._marker_pub     = marker_pub
        rospy.on_shutdown(self.shutdown)

    def poseCallback(self, msg):
        self._object_pose = msg.pose
        self.viz_cube()

    def viz_cube(self):
       marker = Marker()
       marker.header.frame_id = "panda_link0"
       marker.type = marker.CUBE
       marker.action = marker.ADD
       marker.scale.x = 0.025
       marker.scale.y = 0.025
       marker.scale.z = 0.025
       marker.color.a = 1.0       
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = self._object_pose.position.x
       marker.pose.position.y = self._object_pose.position.y
       marker.pose.position.z = self._object_pose.position.z
       self._marker_pub.publish(marker)
    
    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Marker Viz Node")
        rospy.sleep(1)

def main():
    rospy.init_node('franka_obj_viz')
    wait_for_time()

    
    marker_publisher = rospy.Publisher('/franka_grasped_object', Marker, queue_size=1)
    objectViz = ObjectViz(marker_publisher)

    # Add subscriber for "object grasped state" should be a boolean!
    # rospy.Subscriber('/franka_gripper/object_grasped', Bool, objectViz_rh.graspedCallback)
    rospy.Subscriber('/franka_state_controller/O_T_EE', PoseStamped, objectViz.poseCallback)
    
    rospy.spin()

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       objectViz.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')


if __name__ == '__main__':
    main()