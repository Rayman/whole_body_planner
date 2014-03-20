#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_whole_body_controller')
import rospy
import tf
import actionlib
from amigo_whole_body_controller.msg import *
from arm_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
#from amigo_arm_navigation.msg._grasp_precomputeGoal import grasp_precomputeGoal
#from amigo_arm_navigation.msg._grasp_precomputeAction import grasp_precomputeAction

def euler_z_to_quaternion(roll, pitch, yaw):
    
    rospy.logwarn("These should be euler angles")
    orientation_goal = geometry_msgs.msg.Quaternion()
    
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation_goal.x = quaternion[0]
    orientation_goal.y = quaternion[1]
    orientation_goal.z = quaternion[2]
    orientation_goal.w = quaternion[3]
    
    return orientation_goal

if __name__ == '__main__':

    rospy.init_node('add_motion_objective')
    #/whole_body_planner/motion_constraint/goal
    rospy.loginfo("Node initialized")
    
    tf_listener = tf.TransformListener()
    
    move_arm = actionlib.SimpleActionClient("/whole_body_planner/motion_constraint", ArmTaskAction)
    move_arm.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    marker_pub = rospy.Publisher('/visualization_marker', Marker)
    
    # Target in map frame
    map_pose = geometry_msgs.msg.PoseStamped()
    map_pose.header.frame_id = "/map"
    map_pose.header.stamp = rospy.Time()
    map_pose.pose.position.x = float(sys.argv[1])
    map_pose.pose.position.y = float(sys.argv[2])
    map_pose.pose.position.z = float(sys.argv[3])
    map_pose.pose.orientation = euler_z_to_quaternion(float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]))
    
    # Transform to base_link
    tf_listener.waitForTransform("/map", "/amigo/base_link", rospy.Time(), rospy.Duration(2.0))
    baselink_pose = tf_listener.transformPose("/amigo/base_link", map_pose)
    
    rospy.loginfo("Goal pose map = {0}".format(baselink_pose))   

    goal = ArmTaskGoal()
    #rospy.loginfo(goal)
    #goal.goal_type = "grasp"
    goal.goal_type = sys.argv[7]
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "grippoint_left"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    position_constraint.position = baselink_pose.pose.position
    goal.position_constraint = position_constraint
    rospy.logwarn("Position constraint region shapes etc. not yet defined")
    
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"
    orientation_constraint.link_name = "grippoint_left"
    orientation_constraint.orientation = baselink_pose.pose.orientation
    goal.orientation_constraint = orientation_constraint
    rospy.loginfo("Type link or header not yet taken into account")
    rospy.logwarn("Orientation constraint tolerances etc not yet defined")

    goal.stiffness.force.x = 70.0
    goal.stiffness.force.y = 60.0
    goal.stiffness.force.z = 50.0
    goal.stiffness.torque.x = 2.0
    goal.stiffness.torque.y = 2.0
    goal.stiffness.torque.z = 2.0
        
    rospy.loginfo(goal)
    
    goal_marker = Marker()
    goal_marker.header = goal.position_constraint.header
    goal_marker.id = 5432
    goal_marker.type = 0 # Arrow
    goal_marker.pose.position = goal.position_constraint.position
    goal_marker.pose.orientation = goal.orientation_constraint.orientation
    goal_marker.scale.x = 0.2
    goal_marker.scale.y = 0.05
    goal_marker.scale.z = 0.05
    goal_marker.color.r = 0.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 1.0
    goal_marker.color.a = 1.0
    goal_marker.lifetime = rospy.Duration(5.0)

    rospy.loginfo(goal_marker)

    ctr = 0;
    while (not rospy.is_shutdown() and ctr < 10):
        marker_pub.publish(goal_marker)
        rospy.sleep(rospy.Duration(0.1))
        ctr = ctr + 1
        rospy.sleep(0.02)
        
    #actionClients.move_arm.send_goal_and_wait(goal, rospy.Duration(time_out))
    result = move_arm.send_goal_and_wait(goal, rospy.Duration(1.0))
    rospy.loginfo("Result = {0}".format(result))
    
    #ctr = 5
    #while (ctr > 0):
    #    rospy.loginfo("Waiting for {0} s".format(ctr))
    #    rospy.sleep(rospy.Duration(1.0))
    #    ctr = ctr - 1
    
    #goal = ArmTaskGoal()
    #goal.goal_type = "reset"
    #goal.remove_tip_frame = "grippoint_left"
    #goal.remove_root_frame = "base_link"
    
    #result = move_arm.send_goal_and_wait(goal, rospy.Duration(1.0))
    #rospy.loginfo("Result = {0}".format(result))
    
    #ctr = 5
    #while (ctr > 0):
    #    rospy.loginfo("Waiting for {0} s".format(ctr))
    #    rospy.sleep(rospy.Duration(1.0))
    #    ctr = ctr - 1
    
    #goal = ArmTaskGoal()
    #goal.goal_type = "reset"
    #goal.remove_tip_frame = "grippoint_right"
    
    #result = move_arm.send_goal_and_wait(goal, rospy.Duration(1.0))
    #rospy.loginfo("Result = {0}".format(result))
    
    #ctr = 5
    #while (ctr > 0):
    #    rospy.loginfo("Waiting for {0} s".format(ctr))
    #    rospy.sleep(rospy.Duration(1.0))
    #    ctr = ctr - 1
    
    #goal = ArmTaskGoal()
    #goal.goal_type = "reset"
    #goal.remove_tip_frame = "grippoint_left"
    
    #result = move_arm.send_goal_and_wait(goal, rospy.Duration(1.0))
    #rospy.loginfo("Result = {0}".format(result))

    
