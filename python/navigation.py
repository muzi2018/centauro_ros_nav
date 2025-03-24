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
import geometry_msgs.msg

# map info
# resolution: 0.05
# width: 238
# height: 236
# origin: x y z {-2.75 -5.6 0.0} 
# frame: map

# Chair 1: (1.88, 0.83, 3.43) -> (3.684297883136837, -1.4521300096470415, -0.7249385538403157)
# Chair 2: (0.75, 1.12, 8.4) -> (8.509858507416597, 0.21297738285356893, -0.7844677892095936)
# Chair 3: (-1.33, 0.83, 3.61) -> (3.512881381696345, 1.7570635315974463, -0.8032891220488297)

'''
tf2_ros.StaticTransformBroadcaster
'''

class ChairTFBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
    
    def publish_chair_tf(self, chair_name, x, y, z, yaw):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"  # The chair should be aligned with the map
        t.child_frame_id = chair_name  # e.g., "chair_1"
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        quat = tf.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

        
        
chair_tf_broadcaster = ChairTFBroadcaster()
Chairs_dict = {"chair_1": {"position":(0.0, 0.0, 0.0), "record": False},
               "chair_2": {"position":(0.0, 0.0, 0.0), "record": False},
               "chair_3": {"position":(0.0, 0.0, 0.0), "record": False}
               }
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

obj_dict = {}
def callback(msg):
    global obj_dict
    global update_flag
    try:
        obj_dict = json.loads(msg.data)                                 # Convert JSON string back to dictionary
        if "chair1" in obj_dict and update_flag:
            x, y, z = obj_dict["chair1"]
            update_flag = False
            rospy.loginfo(f"chair1 Position: x={x}, y={y}, z={z}")
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
            transformed_pos = transformer.transform_point(*obj_dict["chair_1"]["position"])
            x, y, z = transformed_pos
            yaw = 0
            chair_tf_broadcaster.publish_chair_tf("chair", x, y, z, yaw)
            if not Chairs_dict["chair_1"]["record"] and cnt == 0:
                Chairs_dict["chair_1"]["position"] = transformed_pos
                Chairs_dict["chair_1"]["record"] = True
                pos_o = obj_dict["chair_1"]["position"]
                waypoints = [(Chairs_dict["chair_1"]["position"][0], Chairs_dict["chair_1"]["position"][1] - 1.3, 0.45)]
                print(f"Original Chair 1 pos -> {pos_o}")
                print(f"Chair 1 transformed_pos -> {transformed_pos}") # Chair 1 transformed_pos -> (3.492636803450569, -0.6283296429808908, -0.9809153977970663)

            elif Chairs_dict["chair_1"]["record"] and not Chairs_dict["chair_2"]["record"] and cnt == 1:
                Chairs_dict["chair_2"]["position"] = transformed_pos
                Chairs_dict["chair_2"]["record"] = True
                pos_o = obj_dict["chair_1"]["position"]
                print("chair_2 original: ", pos_o)
                waypoints = [(Chairs_dict["chair_2"]["position"][0], Chairs_dict["chair_2"]["position"][1] - 1.3, 1.45)]
                
                # x, y, z = transformed_pos
                # yaw = 0
                # chair_tf_broadcaster.publish_chair_tf("chair_2", x, y, z, yaw)
                
            elif Chairs_dict["chair_1"]["record"] and Chairs_dict["chair_2"]["record"] and not Chairs_dict["chair_3"]["record"] and cnt == 2:
                Chairs_dict["chair_3"]["position"] = transformed_pos
                Chairs_dict["chair_3"]["record"] = True
                pos_o = obj_dict["chair_1"]["position"]
                waypoints = [(Chairs_dict["chair_3"]["position"][0], Chairs_dict["chair_3"]["position"][1] + 1.3, -1.45)]
                
                # x, y, z = transformed_pos
                # yaw = 0
                # chair_tf_broadcaster.publish_chair_tf("chair_3", x, y, z, yaw)
                
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
                        # Chairs_dict["chair_1"]["record"] = False
                        cnt += 1
                        print("reset chair_1 ...")
                    rospy.loginfo("Successfully reached goal: {}".format(waypoint))
                else:
                    rospy.logwarn("Failed to reach goal: {} with state: {}".format(waypoint, state))
        
        # x, y, z = transformed_pos
        # yaw = 0
        # chair_tf_broadcaster.publish_chair_tf("chair_1", x_1, y_1, z_1, yaw)
        # chair_tf_broadcaster.publish_chair_tf("chair_2", x_1, y_1, z_1, yaw)
        # chair_tf_broadcaster.publish_chair_tf("chair_3", x_1, y_1, z_1, yaw)
        
        rospy.sleep(1)
        
rospy.Subscriber('object_positions', String, callback)

if __name__ == "__main__":
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted. Exiting...")


