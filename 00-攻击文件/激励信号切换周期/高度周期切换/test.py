import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_node')
try:
    msg = rospy.wait_for_message('/iris_0/mavros/local_position/pose', PoseStamped, timeout=5)
    rospy.loginfo("Received message: %s", msg)
except rospy.ROSException as e:
    rospy.logwarn("Failed to receive message: %s", e)
