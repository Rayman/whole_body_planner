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
    
    orientation_goal = geometry_msgs.msg.Quaternion()
    
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation_goal.x = quaternion[0]
    orientation_goal.y = quaternion[1]
    orientation_goal.z = quaternion[2]
    orientation_goal.w = quaternion[3]
    
    return orientation_goal

def getPose(environment, iter, max_iter):

    position_constraint = PositionConstraint()
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "grippoint_right"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0

    if environment=="Tunnel":
        if iter < max_iter/4:
            position_constraint.position.x = 0.5
            position_constraint.position.y = 0.05
            position_constraint.position.z = 0.95

        elif iter < (2*max_iter)/4:
            position_constraint.position.x = 0.5
            position_constraint.position.y = 0.05
            position_constraint.position.z = 0.95

        elif iter < (3*max_iter)/4:    
            position_constraint.position.x = 0.50
            position_constraint.position.y = 0.05
            position_constraint.position.z = 0.90

        else:
            position_constraint.position.x = 0.5
            position_constraint.position.y = 0.05
            position_constraint.position.z = 0.90

    elif environment=="Table":

        if iter < max_iter/4:
            position_constraint.position.x = 0.50
            position_constraint.position.y = 0.35
            position_constraint.position.z = 0.92

        elif iter < (2*max_iter)/4:
            position_constraint.position.x = 0.50
            position_constraint.position.y = 0.25
            position_constraint.position.z = 0.92
        elif iter < (3*max_iter)/4:    
            position_constraint.position.x = 0.50
            position_constraint.position.y = 0.35
            position_constraint.position.z = 0.92

        else:
            position_constraint.position.x = 0.50
            position_constraint.position.y = 0.35
            position_constraint.position.z = 0.92

    elif environment=="Kitchen":

        if iter < max_iter/4:
            position_constraint.position.x = 0.6
            position_constraint.position.y = 0.155
            position_constraint.position.z = 1.3

        elif iter < (2*max_iter)/4:
            position_constraint.position.x = 0.6
            position_constraint.position.y = 0.435
            position_constraint.position.z = 0.65
        elif iter < (3*max_iter)/4:    
            position_constraint.position.x = 0.6
            position_constraint.position.y = 0.155
            position_constraint.position.z = 1.3

        else:
            position_constraint.position.x = 0.6
            position_constraint.position.y = 0.435
            position_constraint.position.z = 0.65

    return position_constraint

if __name__ == '__main__':


    rospy.init_node('add_motion_objective')
    #/whole_body_planner/motion_constraint/goal
    rospy.loginfo("Node initialized")
    move_arm = actionlib.SimpleActionClient("/whole_body_planner/motion_constraint", ArmTaskAction)
    move_arm.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    marker_pub = rospy.Publisher('/visualization_marker', Marker)

    goal = ArmTaskGoal()
    #rospy.loginfo(goal)
    #goal.goal_type = "grasp"
    environment = sys.argv[1]
    max_iter = float(sys.argv[2])
    goal.goal_type = "grasp"
        
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"
    orientation_constraint.link_name = "grippoint_right"
    orientation_constraint.orientation = euler_z_to_quaternion(0.0,0.0,0.0)
    goal.orientation_constraint = orientation_constraint

    goal.stiffness.force.x = 120.0
    goal.stiffness.force.y = 120.0
    goal.stiffness.force.z = 100.0
    goal.stiffness.torque.x = 28.0
    goal.stiffness.torque.y = 28.0
    goal.stiffness.torque.z = 0.0
    goal.orientation_constraint.absolute_roll_tolerance = 0.1
    goal.orientation_constraint.absolute_pitch_tolerance = 0.1
    goal.orientation_constraint.absolute_yaw_tolerance = 3.2
    
    ctr = 0;
    while (not rospy.is_shutdown() and ctr < max_iter):
        #marker_pub.publish(goal_marker)
        goal.position_constraint  = getPose(environment, ctr, max_iter)
        
        result = move_arm.send_goal_and_wait(goal, rospy.Duration(5.0))
        rospy.loginfo("Result = {0}".format(result))
        ctr = ctr + 1
        
    
