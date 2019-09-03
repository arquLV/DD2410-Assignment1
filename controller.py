#!/usr/bin/env python2

# DD2410 Introduction to Robotics
# Assignment 1
#
# Arturs Kurzemnieks (artursk@kth.se)


import rospy
import actionlib
from irob_assignment_1.msg import GetNextGoalAction, GetNextGoalGoal, GetNextGoalActionResult
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot, fabs, copysign

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0

gain_threshold = 20

def on_goal_active():
    rospy.loginfo("Path goal sent, waiting for response...")

def on_goal_feedback(feedback):
    global gain_threshold
    if feedback.gain > gain_threshold:
        goal_client.cancel_all_goals()

        rospy.loginfo("Sending on a new path...")
        move(feedback.path) 

def on_goal_done(state, result):
    global gain_threshold
    if state == actionlib.TerminalState.SUCCEEDED:
        if result.gain > 0:
            move(result.path)
            gain_threshold = result.gain / 2

        elif result.gain == 0:
            rospy.loginfo("Gain is 0, nothing left to explore!")


def move(path):
    global control_client, robot_frame_id, pub
    rate = rospy.Rate(20)

    while path.poses:
        # Call service client with path
        res = control_client(path)

        new_path = res.new_path
        setpoint = res.setpoint

        # Lookup transform from map frame to base link frame
        try:
            # target frame = robot_frame_id (base_link)
            # source frame = setpoint frame (map)
            transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        # Transform Setpoint from service client
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Create Twist message from the transformed Setpoint
        twist_msg = Twist()

        linear_vel = hypot(transformed_setpoint.point.x, transformed_setpoint.point.y)
        # Limit maximum linear velocity
        if linear_vel >= max_linear_velocity:
            linear_vel = copysign(max_linear_velocity, linear_vel)

        angular_vel = atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        # Limit maximum angular velocity
        if fabs(angular_vel) >= max_angular_velocity:
            angular_vel = copysign(max_angular_velocity, angular_vel)

        # If a heavy turn needed, stop moving forward     
        if fabs(angular_vel) > (0.9 * max_angular_velocity):
            linear_vel = 0

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        # Publish Twist
        pub.publish(twist_msg)
        rate.sleep()
        
        path = new_path

    rospy.loginfo("Path exhausted, getting new path...")

    # Send empty Twist so the robot stops while new path is fetched
    zero_twist_msg = Twist()
    pub.publish(zero_twist_msg)

    get_path()



def get_path():
    global goal_client
    rospy.loginfo("Getting path from action server...")

    goal = GetNextGoalGoal()  # no arguments for the goal needed
    goal_client.send_goal(goal, active_cb=on_goal_active, feedback_cb=on_goal_feedback, done_cb=on_goal_done)


if __name__ == "__main__":

    # Init node
    rospy.init_node('burger_controller', anonymous=True)    # anonymous ensures unique node name for the instance

    # Init TF2 listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # creates publisher for "cmd_vel" topic with Twist message type

    # Init simple action server
    goal_client = actionlib.SimpleActionClient('get_next_goal', GetNextGoalAction)
    goal_client.wait_for_server()
    
    # Init service client
    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get path
    get_path()
    # Spin
    rospy.spin()