#!/usr/bin/env python3

import rospy
import tf.transformations
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
# from franka_msgs.msg import FrankaState

marker_pose = PoseStamped()
#initial_pose_found = False
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisherCallback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)#actually pub


# def franka_state_callback(msg):
#     initial_quaternion = \
#         tf.transformations.quaternion_from_matrix(
#             np.transpose(np.reshape(msg.O_T_EE,
#                                     (4, 4))))
#     initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
#     marker_pose.pose.orientation.x = initial_quaternion[0]
#     marker_pose.pose.orientation.y = initial_quaternion[1]
#     marker_pose.pose.orientation.z = initial_quaternion[2]
#     marker_pose.pose.orientation.w = initial_quaternion[3]
#     marker_pose.pose.position.x = msg.O_T_EE[12]
#     marker_pose.pose.position.y = msg.O_T_EE[13]
#     marker_pose.pose.position.z = msg.O_T_EE[14]
#     global initial_pose_found
#     initial_pose_found = True


def processFeedback(feedback):#Called when marker is moved
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation#update the marker_pose, but doesn't actually publish it
        print(marker_pose.pose.orientation)
    #server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("desired_pose_node")
    # state_sub = rospy.Subscriber("franka_state_controller/franka_states",
    #                              FrankaState, franka_state_callback)
    listener = tf.TransformListener()
    link_name = "base_link"#rospy.get_param("~link_name")#??????
    
    #"ee_link"
    #maybe the root of the world????

    # Get initial pose for the interactive marker
    # while not initial_pose_found:
    #     rospy.sleep(1)
    # state_sub.unregister()#???IMPORTANT!

    pose_pub = rospy.Publisher(
        "desired_pose", PoseStamped, queue_size=10)
    
    server = InteractiveMarkerServer("desired_pose_marker")
    
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name#????
    int_marker.scale = 0.3
    int_marker.name = "/cartesian_impedance_controller/desired_pose"
    int_marker.description = ("Desired Pose\nBE CAREFUL! "
                              "If you move the \nequilibrium "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    #########################################################
    # int_marker.pose = marker_pose.pose
    listener.waitForTransform("base_link", "ee_link", rospy.Time(0), rospy.Duration(4.0))

    # Get the transform: position and quaternion
    (trans, rot) = listener.lookupTransform("base_link", "ee_link", rospy.Time(0))

    # Fill the pose
    marker_pose = PoseStamped()
    marker_pose.header.frame_id = "base_link"
    marker_pose.pose.position.x = trans[0]
    marker_pose.pose.position.y = trans[1]
    marker_pose.pose.position.z = trans[2]
    marker_pose.pose.orientation.x = rot[0]
    marker_pose.pose.orientation.y = rot[1]
    marker_pose.pose.orientation.z = rot[2]
    marker_pose.pose.orientation.w = rot[3]

    # Then assign to marker
    int_marker.pose = marker_pose.pose
    #######################################################
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisherCallback(msg, link_name))
    # Starts a background loop that publishes the pose at a fixed interval (every 5 ms).
    # Runs on a separate thread, so it doesn't block anything.

    # insert a box
    ###########################To add the interactive axis+rotations in RViz#####################################
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    ###########################To add the interactive axis+rotations in RViz#####################################
    

    # Registers your InteractiveMarker (int_marker) with the interactive marker server.
    # Associates it with a callback (processFeedback) that will be triggered whenever the marker is interacted with (moved, rotated, etc.).
    # Think of it like:
    # "Hey server, here's a new marker I want to show in RViz, and here's what to do when the user interacts with it."
    server.insert(int_marker, processFeedback)
    server.applyChanges() 
    #Applies all pending updates to the marker server and publishes the updated state to RViz.
    #Any insertions, deletions, or modifications made to markers are batched until this is called.
    #MANDATORY TO SHOW THE MARKER IN RViz, and should be after insert
    

    rospy.spin()
    # Keeps the main thread alive and handles ROS callbacks (like when you move the marker).
    # Makes sure processFeedback, franka_state_callback, and others keep working.
