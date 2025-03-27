#!/usr/bin/env python
import rospy
import json
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String

class ChairTFBroadcaster:
    def __init__(self):
        rospy.init_node('chair_tf_broadcaster', anonymous=True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.chair_positions = {}  # Store latest chair positions
        self.chair_sub = rospy.Subscriber('/chair_positions', String, self.chair_callback)

    def chair_callback(self, msg):
        """Callback function to receive chair positions and update stored data."""
        try:
            self.chair_positions = json.loads(msg.data)  # Convert JSON string to dictionary
        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON message!")

    def broadcast_tf(self):
        """Continuously publishes chair positions as TF transforms."""
        rate = rospy.Rate(1)  # Reduce update frequency to 1 Hz
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            for chair_id, chair_info in self.chair_positions.items():
                if "position" in chair_info:
                    x, y, z = chair_info["position"]

                    transform = geometry_msgs.msg.TransformStamped()
                    transform.header.stamp = current_time  # Ensure timestamps are updated
                    transform.header.frame_id = "map"
                    transform.child_frame_id = chair_id  # TF frame name for each chair

                    transform.transform.translation.x = x
                    transform.transform.translation.y = y
                    transform.transform.translation.z = z

                    transform.transform.rotation.x = 0
                    transform.transform.rotation.y = 0
                    transform.transform.rotation.z = 0
                    transform.transform.rotation.w = 1  # No rotation

                    self.tf_broadcaster.sendTransform(transform)
                    rospy.loginfo(f"Published TF for {chair_id} at {x}, {y}, {z}")

            rate.sleep()  # Maintain the loop rate

if __name__ == '__main__':
    try:
        broadcaster = ChairTFBroadcaster()
        broadcaster.broadcast_tf()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
