#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import threading
import geometry_msgs.msg
import tf2_ros
import json
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import tf
import math
import os
from dynamic_reconfigure.client import Client

# map info
# resolution: 0.05
# width: 238 x 0.05 = 12
# height: 236 x 0.05 = 12
# origin: x y z {-2.75 -5.6 0.0} 
# frame: map

# chair 0.56 x 0.56
# centauro 0.7 x 0.7

# Chairs_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "record": False}}


from tf.transformations import euler_from_quaternion
import tf2_ros
from geometry_msgs.msg import PoseStamped



search_tag = False
width = 10
height = 10

Chairs_dict = {}
update_flag = True
tag_yaw = None
tag_roll = None

tf_buffer = None
tf_listener = None

# def publish_chair_positions():
#     """Continuously publishes chair positions as PoseStamped messages to RViz."""
#     rate = rospy.Rate(1)  # Publish at 1 Hz
#     tf_broadcaster = tf2_ros.TransformBroadcaster()
#     while not rospy.is_shutdown():
#         for chair_id, chair_data in Chairs_dict.items():
#             if "position" in chair_data:
#                 x, y, z = chair_data["position"]
#             transform = geometry_msgs.msg.TransformStamped()
#             transform.header.stamp = rospy.Time.now()
#             transform.header.frame_id = "map"  # Adjust this based on your setup
#             transform.child_frame_id = chair_id

#             transform.transform.translation.x = x
#             transform.transform.translation.y = y
#             transform.transform.translation.z = z
            
#             transform.transform.rotation.x = 0
#             transform.transform.rotation.y = 0
#             transform.transform.rotation.z = 0
#             transform.transform.rotation.w = 1

#             tf_broadcaster.sendTransform(transform)
#             # rospy.loginfo(f"Published TF for {transform.child_frame_id}")


#         rate.sleep()
# Setup TF2 buffer and listener (should be created globally once)


def tag_detections_callback(msg):
    global tag_yaw, tag_roll
    global tf_buffer

    if not msg.detections:
        # rospy.loginfo("No AprilTags detected.")
        return

    for detection in msg.detections:
        # tf_buffer = tf2_ros.Buffer()
        # tf_listener = tf2_ros.TransformListener(tf_buffer)
        tag_id = detection.id[0]
        
        pose = detection.pose.pose.pose  # geometry_msgs/Pose
        
        position = pose.position
        orientation = pose.orientation
        # print("tag_id:---", tag_id)
        # rospy.loginfo(f"Detected AprilTag ID: {tag_id}")
        # rospy.loginfo(f"Position in camera frame: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")
        # rospy.loginfo(f"Orientation in camera frame (Quaternion): ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f})")

        try:
            # Create PoseStamped from detection
            cam_pose_stamped = PoseStamped()
            cam_pose_stamped.header.frame_id = "D435_head_camera_color_optical_frame"
            cam_pose_stamped.header.stamp = rospy.Time.now()
            cam_pose_stamped.pose = pose

            # Transform the pose to map frame
            map_pose_stamped = tf2_geometry_msgs.do_transform_pose(
                cam_pose_stamped,
                tf_buffer.lookup_transform("map", cam_pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            )

            # Extract orientation in map frame
            map_orientation = map_pose_stamped.pose.orientation
            map_tag_x = map_pose_stamped.pose.position.x
            map_tag_y = map_pose_stamped.pose.position.y
            map_tag_z = map_pose_stamped.pose.position.z
            
            
            euler_map = euler_from_quaternion([
                map_orientation.x,
                map_orientation.y,
                map_orientation.z,
                map_orientation.w
            ])

            if "chair" in obj_dict:
                transformer = TransformChairPosition()
                transformed_pos = transformer.transform_point(*obj_dict["chair"]["position"])
                chair_x, chair_y, chair_z = transformed_pos
                distance = math.sqrt((map_tag_x - chair_x)**2 + 
                                    (map_tag_y - chair_y)**2 + 
                                    (map_tag_z - chair_z)**2)
                # print("distance: ", distance)
                # print("tag_id:---", tag_id)
                # print("transformed_pos: ", transformed_pos)
                if distance < 0.7:
                    tag_yaw = euler_map[2]
                # print("tag_roll: ", tag_roll)
            else:
                tag_yaw = None

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(f"TF transform failed: {e}")

def compute_waypoint(chair_x, chair_y, chair_yaw, distance = 1.2):
    """
    Computes a goal pose in front of the chair, facing it.
    """
    # Move backwards from the chair along its facing direction (yaw)
    chair_yaw_degrees = math.degrees(chair_yaw)
    yaw_goal = chair_yaw
    print("chair_x: ", chair_x)
    print("chair_y: ", chair_y)
    print("chair_yaw_degrees: ", chair_yaw_degrees)

    
    # chair_yaw_degrees = chair_yaw_degrees - 180
    if chair_yaw >= 0 and chair_yaw <= 1.57:      # x-, y-
        x_goal = chair_x - distance * math.cos(chair_yaw)
        y_goal = chair_y - distance * math.sin(chair_yaw)
    elif chair_yaw >= 1.57 and chair_yaw <= 3.14:     # x+, y-
        chair_yaw = chair_yaw - 1.57
        x_goal = chair_x + distance * math.sin(chair_yaw)
        y_goal = chair_y - distance * math.cos(chair_yaw)
    elif chair_yaw <= -1.57:       # x+, y+
        chair_yaw = chair_yaw + 3.14
        x_goal = chair_x + distance * math.cos(chair_yaw)
        y_goal = chair_y + distance * math.sin(chair_yaw)
    elif chair_yaw <= 0 and chair_yaw >= -1.57:
        chair_yaw = chair_yaw + 1.57
        x_goal = chair_x - distance * math.sin(chair_yaw)
        y_goal = chair_y + distance * math.cos(chair_yaw)
    
    # Robot should face the chair, so its yaw is same as chair's yaw
    
    
    return x_goal, y_goal, yaw_goal


class TransformChairPosition:
    global tf_buffer


    # def __init__(self):
    #     # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform_point(self, x, y, z):
        try:
            tf_buffer.can_transform('map', '435_head_camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            transform_stamped = tf_buffer.lookup_transform(
                'map',                                                  # Target frame
                'D435_head_camera_color_optical_frame',                 # Source frame
                rospy.Time(0),                                          # Get the latest transform
                rospy.Duration(1.0)                                     # Timeout duration
            )

            point = PointStamped()
            point.header.frame_id = "435_head_camera_color_optical_frame"
            point.header.stamp = rospy.Time.now()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform_stamped)
            
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
        return None

obj_dict = {"chair": {"position":(0.0, 0.0, 0.0), "detected": False}}
def callback(msg):
    global obj_dict
    try:
        obj_dict = json.loads(msg.data)                                 # Convert JSON string back to dictionary
        # print("obj_dict in callback .. ", obj_dict)
        if "chair1" in obj_dict :
            x, y, z = obj_dict["chair1"]
            rospy.loginfo(f"chair1 Position: x={x}, y={y}, z={z}")
        # else:
        #     obj_dict = {}
    except json.JSONDecodeError:
        rospy.logerr("Failed to decode JSON message!")

transformed_pos = []
cnt = 1
def send_waypoints():
    global cnt, tag_roll, search_tag, tag_yaw
    global tf_buffer, tf_listener
    rospy.init_node('send_waypoints', anonymous=True)
    os.environ['ROSCONSOLE_CONFIG_FILE'] = os.path.expanduser('~/.ros/rosconsole.config')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Start the chair position publisher in a separate thread
    publisher_thread = threading.Thread(target=publish_chair_positions)
    publisher_thread.daemon = True  # Stops when the main script exits
    publisher_thread.start()
    
    global transformed_pos
    transformer = TransformChairPosition()
    client = SimpleActionClient('/move_base', MoveBaseAction)

    pre_pos = []

    rospy.loginfo("Waiting for move_base action server to start...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")
    while not rospy.is_shutdown():    
        print("tag_yaw = ", tag_yaw)
        if "chair" in obj_dict and tag_yaw != None:
            print("obj_dict :", obj_dict)
            transformed_pos = transformer.transform_point(*obj_dict["chair"]["position"])
            x_curr, y_curr, _ = transformed_pos
            is_new_chair = True
            for chair_key, chair_data in Chairs_dict.items():
                if "position" in chair_data:
                    x_other, y_other, _ = chair_data["position"]
                    distance = math.sqrt((x_curr - x_other)**2 + (y_curr - y_other)**2)
                    rospy.loginfo(f"Distance between current chair and {chair_key}: {distance:.2f} meters")

                    if distance < 0.56:
                        rospy.loginfo(f"Distance < 0.56 meters. Chair is too close to {chair_key}. Sending stop flag.")
                        is_new_chair = False
                        # You can publish or trigger a flag here
                        # stop_pub.publish("stop") or similar logic
                        break
            
            
            if is_new_chair:
                Chairs_dict[f"chair_{cnt}"] = {}  # Initialize dictionary entry
                Chairs_dict[f"chair_{cnt}"]["position"] = transformed_pos
                Chairs_dict[f"chair_{cnt}"]["record"] = True
                Chairs_dict[f"chair_{cnt}"]["ori"] = tag_yaw 
                if Chairs_dict[f"chair_{cnt}"]["position"][1] < 0:
                    x_chair, y_chair, _ = Chairs_dict[f"chair_{cnt}"]["position"]
                    yaw_chair = Chairs_dict[f"chair_{cnt}"]["ori"]
                    waypoints = [compute_waypoint(x_chair, y_chair, yaw_chair)]
                    pre_pos = waypoints
                elif Chairs_dict[f"chair_{cnt}"]["position"][1] > 0:
                    x_chair, y_chair, _ = Chairs_dict[f"chair_{cnt}"]["position"]
                    yaw_chair = Chairs_dict[f"chair_{cnt}"]["ori"]
                    waypoints = [compute_waypoint(x_chair, y_chair, yaw_chair)]
                    pre_pos = waypoints


                for waypoint in waypoints:
                    x, y, theta = waypoint
                    goal = MoveBaseGoal()
                    goal.target_pose.header = Header()
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.header.frame_id = "map"                
                    
                    goal.target_pose.pose.position.x = x
                    goal.target_pose.pose.position.y = y
                    goal.target_pose.pose.position.z = 0.0
                    goal.target_pose.pose.orientation.z = theta
                    print("theta: ", theta)
                    goal.target_pose.pose.orientation.w = 1.0  

                    rospy.loginfo("Sending goal: {}".format(waypoint))
                    client.cancel_all_goals()
                    rospy.sleep(0.5)
                    client.send_goal(goal)
                    client.wait_for_result()

                    state = client.get_state()
                    if state == 3:  
                        cnt = cnt + 1
                        tag_yaw = None
                        rospy.loginfo("Successfully reached goal: {}".format(waypoint))
                        print("\n")
                    else:
                        del Chairs_dict[f"chair_{cnt}"]
                        search_tag = True
                        print(f"searching {cnt}'th chair ...")
                        twist = Twist()
                        twist.angular.z = -0.5  # Positive = counter-clockwise rotation
                        cmd_vel_pub.publish(twist)
                        rospy.logwarn("Failed to reach goal: {} with state: {}".format(waypoint, state))
                        print("\n")
            else:
                for pre_pos_ in pre_pos:
                    print(f"searching {cnt}'th chair ...")
                    twist = Twist()
                    twist.angular.z = -0.5  # Positive = counter-clockwise rotation
                    cmd_vel_pub.publish(twist)
        else:
            for pre_pos_ in pre_pos:
                print(f"searching {cnt}'th chair ...")
                twist = Twist()
                twist.angular.z = -0.3  # Positive = counter-clockwise rotation
                cmd_vel_pub.publish(twist)
        
        # if search_tag:
        #         print(f"searching {cnt}'th chair ...")
        #         twist = Twist()
        #         twist.angular.z = -1  # Positive = counter-clockwise rotation
        #         cmd_vel_pub.publish(twist)
        
        rospy.sleep(1)
        
rospy.Subscriber('object_positions', String, callback)

chairs_pub = rospy.Publisher('/chair_positions', String, queue_size=10)  # Define the publisher

cmd_vel_pub = rospy.Publisher('/omnisteering/cmd_vel', Twist, queue_size=10)



def publish_chair_positions():
    """Continuously publishes chair positions as a JSON string."""
    # rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        if Chairs_dict:  # Check if dictionary is not empty
            chairs_json = json.dumps(Chairs_dict)  # Convert dictionary to JSON string
            chairs_pub.publish(chairs_json)  # Publish the JSON string
            # rospy.loginfo(f"Published Chairs_dict: {chairs_json}")
        # rate.sleep()


rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_detections_callback)


if __name__ == "__main__":
    try:
        send_waypoints()
        # Setup TF2 buffer and listener (should be created globally once)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted. Exiting...")
