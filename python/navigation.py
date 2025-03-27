#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionResult
import tf.transformations as tf
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import threading
import geometry_msgs.msg
import tf2_ros
import json
# map info
# resolution: 0.05
# width: 238
# height: 236
# origin: x y z {-2.75 -5.6 0.0} 
# frame: map

# chair 0.56 x 0.56
# centauro 0.7 x 0.7

# Chairs_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "record": False}}
Chairs_dict = {}
update_flag = True

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


class TransformChairPosition:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform_point(self, x, y, z):
        try:
            self.tf_buffer.can_transform('map', '435_head_camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            transform_stamped = self.tf_buffer.lookup_transform(
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
    global cnt

    rospy.init_node('send_waypoints', anonymous=True)
    
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

        if "chair" in obj_dict:
            print("obj_dict :", obj_dict)

            transformed_pos = transformer.transform_point(*obj_dict["chair"]["position"])
            
            
            Chairs_dict[f"chair_{cnt}"] = {}  # Initialize dictionary entry

            Chairs_dict[f"chair_{cnt}"]["position"] = transformed_pos
            Chairs_dict[f"chair_{cnt}"]["record"] = True
            pos_o = obj_dict["chair"]["position"]
            print(f"Original Chair 1 pos -> {pos_o}")
            print(f"Chair 1 transformed_pos -> {transformed_pos}")
            print("cnt: ", cnt)
            
            if Chairs_dict[f"chair_{cnt}"]["position"][1] < 0:
                waypoints = [(Chairs_dict[f"chair_{cnt}"]["position"][0] + 1.0, Chairs_dict[f"chair_{cnt}"]["position"][1] - 1.0, 0.0)]
                pre_pos = waypoints
            elif Chairs_dict[f"chair_{cnt}"]["position"][1] > 0:
                waypoints = [(Chairs_dict[f"chair_{cnt}"]["position"][0] + 1.0, Chairs_dict[f"chair_{cnt}"]["position"][1] + 1.0, 0.0)]
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
                goal.target_pose.pose.orientation.w = 1.0  

                rospy.loginfo("Sending goal: {}".format(waypoint))
                client.send_goal(goal)
                client.wait_for_result()

                state = client.get_state()
                if state == 3:  
                    cnt = cnt + 1
                    rospy.loginfo("Successfully reached goal: {}".format(waypoint))
                    print("\n")
                else:
                    rospy.logwarn("Failed to reach goal: {} with state: {}".format(waypoint, state))
                    print("\n")
        else:
            for pre_pos_ in pre_pos:
                print(f"searching {cnt}'th chair ...")
                x, y, theta = pre_pos_
                goal = MoveBaseGoal()
                goal.target_pose.header = Header()
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.header.frame_id = "map"                
                
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.position.z = 0.0
                goal.target_pose.pose.orientation.z = 6.28
                goal.target_pose.pose.orientation.w = 1.0  
                client.send_goal(goal)
                state = client.get_state()
            
        rospy.sleep(1)
        
rospy.Subscriber('object_positions', String, callback)

chairs_pub = rospy.Publisher('/chair_positions', String, queue_size=10)  # Define the publisher

def publish_chair_positions():
    """Continuously publishes chair positions as a JSON string."""
    # rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        if Chairs_dict:  # Check if dictionary is not empty
            chairs_json = json.dumps(Chairs_dict)  # Convert dictionary to JSON string
            chairs_pub.publish(chairs_json)  # Publish the JSON string
            # rospy.loginfo(f"Published Chairs_dict: {chairs_json}")
        # rate.sleep()


if __name__ == "__main__":
    

    
    
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted. Exiting...")
