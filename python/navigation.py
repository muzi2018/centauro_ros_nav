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

Chairs_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "record": False}}
update_flag = True

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

obj_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "detected": False}}
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
cnt = 0
def send_waypoints():
    global cnt

    rospy.init_node('send_waypoints', anonymous=True)
    global transformed_pos
    transformer = TransformChairPosition()
    client = SimpleActionClient('/move_base', MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server to start...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")
    while not rospy.is_shutdown():    

        if "chair_1" in obj_dict:
            print("obj_dict 111 :", obj_dict)

            transformed_pos = transformer.transform_point(*obj_dict["chair_1"]["position"])
            if not Chairs_dict["chair_1"]["record"]:
                Chairs_dict["chair_1"]["position"] = transformed_pos
                Chairs_dict["chair_1"]["record"] = True
                pos_o = obj_dict["chair_1"]["position"]
                print(f"Original Chair 1 pos -> {pos_o}")
                print(f"Chair 1 transformed_pos -> {transformed_pos}")

            if cnt == 0:
                waypoints = [(Chairs_dict["chair_1"]["position"][0], Chairs_dict["chair_1"]["position"][1] - 1.0, 0.45)]
            elif cnt == 1:
                waypoints = [(Chairs_dict["chair_1"]["position"][0] + 0.8, Chairs_dict["chair_1"]["position"][1], 1.45)]
            elif cnt == 2:
                waypoints = [(Chairs_dict["chair_1"]["position"][0], Chairs_dict["chair_1"]["position"][1] + 1.0, -1.45)]
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
                    if obj_dict["chair_1"]["detected"]:
                        Chairs_dict["chair_1"]["record"] = False
                        cnt += 1
                        print("reset chair_1 ...")
                    rospy.loginfo("Successfully reached goal: {}".format(waypoint))
                else:
                    rospy.logwarn("Failed to reach goal: {} with state: {}".format(waypoint, state))
        else:
                goal = MoveBaseGoal()
                goal.target_pose.header = Header()
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.header.frame_id = "map"                
                
                goal.target_pose.pose.position.x = 0
                goal.target_pose.pose.position.y = 0
                goal.target_pose.pose.position.z = 0.0
                goal.target_pose.pose.orientation.z = 1.45
                goal.target_pose.pose.orientation.w = 1.0  
                client.send_goal(goal)
            
        rospy.sleep(1)
        
rospy.Subscriber('object_positions', String, callback)

if __name__ == "__main__":
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted. Exiting...")
