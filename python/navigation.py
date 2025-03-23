#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionResult
import tf.transformations as tf
import json
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# map info
# resolution: 0.05
# width: 238
# height: 236
# origin: x y z {-2.75 -5.6 0.0} 
# frame: map

# Chair 1: (1.88, 0.83, 3.43) -> (3.684297883136837, -1.4521300096470415, -0.7249385538403157)
# Chair 2: (0.75, 1.12, 8.4) -> (8.509858507416597, 0.21297738285356893, -0.7844677892095936)
# Chair 3: (-1.33, 0.83, 3.61) -> (3.512881381696345, 1.7570635315974463, -0.8032891220488297)

Chairs_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "detected": False}}

update_flag = True

class TransformChairPosition:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform_point(self, x, y, z):
        try:
            # Wait for the transform to be available
            self.tf_buffer.can_transform('map', '435_head_camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'D435_head_camera_color_optical_frame',  # Source frame
                rospy.Time(0),  # Get the latest transform
                rospy.Duration(1.0)  # Timeout duration
            )

            # Create a PointStamped message
            point = PointStamped()
            point.header.frame_id = "435_head_camera_color_optical_frame"
            point.header.stamp = rospy.Time.now()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform_stamped)
            
            # rospy.loginfo(f"Transformed Position: ({transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z})")
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
        return None

obj_dict = {}

def callback(msg):
    global obj_dict
    global update_flag
    try:
        obj_dict = json.loads(msg.data)  # Convert JSON string back to dictionary
        # rospy.loginfo(f"Received Object Data: {obj_dict}")

        # Access specific objects
        if "chair1" in obj_dict and update_flag:
            x, y, z = obj_dict["chair1"]
            update_flag = False
            rospy.loginfo(f"chair1 Position: x={x}, y={y}, z={z}")

    except json.JSONDecodeError:
        rospy.logerr("Failed to decode JSON message!")

transformed_pos = []

def send_waypoints():
    # Initialize the ROS node
    rospy.init_node('send_waypoints', anonymous=True)
    global transformed_pos
    transformer = TransformChairPosition()


    # Create an action client for the move_base action server
    client = SimpleActionClient('/move_base', MoveBaseAction)

    # Wait for the move_base action server to start
    rospy.loginfo("Waiting for move_base action server to start...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")
    # quaternion1 = [0, 0, 0, 1]
    # quaternion2 = [0, 0, 0.5, 0.8]
    # quaternion3 = [0, 0, 0.8, 0.6]
    
    # roll1, pitch1, yaw1 = tf.euler_from_quaternion(quaternion1)
    # roll2, pitch2, yaw2 = tf.euler_from_quaternion(quaternion2)
    # roll3, pitch3, yaw3 = tf.euler_from_quaternion(quaternion3)
    while not rospy.is_shutdown():
        
        if "chair1" in obj_dict:
            transformed_pos = transformer.transform_point(*obj_dict["chair1"])
            # # rospy.loginfo(f"Chair1 Position: x={transformed_pos[0]}, y={transformed_pos[1]}, z={transformed_pos[2]}")
            if not Chairs_dict["chair_1"]["detected"]:
                Chairs_dict["chair_1"]["position"] = transformed_pos
                Chairs_dict["chair_1"]["detected"] = True
                print(f"Chair 1 transformed_pos -> {transformed_pos}")
            # # Define a list of waypoints (x, y, theta)
            # waypoints = [
            #     (transformed_pos[0], transformed_pos[1]-1.3, 0)   
            # ]

            # # Iterate over the waypoints and send them to move_base one by one
            # for waypoint in waypoints:
            #     x, y, theta = waypoint

            #     # Create a MoveBaseGoal message to specify the target pose
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header = Header()
            #     goal.target_pose.header.stamp = rospy.Time.now()
            #     goal.target_pose.header.frame_id = "map"  # Set the frame_id for the pose

            #     # Set the position and orientation
            #     goal.target_pose.pose.position.x = x
            #     goal.target_pose.pose.position.y = y
            #     goal.target_pose.pose.position.z = 0.0
            #     goal.target_pose.pose.orientation.z = theta
            #     goal.target_pose.pose.orientation.w = 1.0  # Ensure orientation is normalized

            #     # Send the goal to the move_base action server
            #     rospy.loginfo("Sending goal: {}".format(waypoint))
            #     client.send_goal(goal)

            #     # Wait for the result (blocking)
            #     client.wait_for_result()

            #     # Check if the goal was achieved successfully
            #     state = client.get_state()
            #     if state == 3:  # State 3 means the goal was succeeded
            #         obj_dict.clear()
            #         # update_flag = True
            #         print("object clear ...")
            #         rospy.loginfo("Successfully reached goal: {}".format(waypoint))
            #     else:
            #         rospy.logwarn("Failed to reach goal: {} with state: {}".format(waypoint, state))

            #     # Wait a little before sending the next goal (optional)
        rospy.sleep(1)
        
rospy.Subscriber('object_positions', String, callback)

if __name__ == "__main__":
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted. Exiting...")
