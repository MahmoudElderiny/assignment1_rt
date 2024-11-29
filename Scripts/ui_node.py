#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

# Globals
distance = float('inf')
turtle1_pose = None
turtle2_pose = None

# Thresholds
DISTANCE_THRESHOLD = 2.0
BOUNDARY_MIN = 1.0
BOUNDARY_MAX = 10.0

def distance_callback(data):
    """Update the global distance variable with the latest distance."""
    global distance
    distance = data.data

def pose_callback_turtle1(data):
    """Update the global pose for turtle1."""
    global turtle1_pose
    turtle1_pose = data

def pose_callback_turtle2(data):
    """Update the global pose for turtle2."""
    global turtle2_pose
    turtle2_pose = data

def predict_new_position(pose, twist, duration=1.0):
    """Predict the new position of a turtle given its current pose and velocity."""
    theta = pose.theta
    new_x = pose.x + twist.linear.x * math.cos(theta) * duration
    new_y = pose.y + twist.linear.x * math.sin(theta) * duration
    return new_x, new_y

def spawn_turtles():
    """Ensure both turtles are spawned."""
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/kill')
    try:
        kill_service = rospy.ServiceProxy('/kill', Kill)
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)

        # Remove default turtle1
        try:
            kill_service('turtle1')
        except rospy.ServiceException:
            rospy.loginfo("Default turtle1 does not exist. Continuing...")

        # Spawn turtles
        spawn_service(8.0, 5.5, 0.0, 'turtle1')
        spawn_service(3.0, 5.5, 0.0, 'turtle2')

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtles: {e}")
        rospy.signal_shutdown("Failed to spawn turtles.")

def control_turtle():
    """Control turtles while enforcing safety rules."""
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        rospy.loginfo("Select turtle to control: 1 for turtle1, 2 for turtle2")
        turtle_choice = input("Enter 1 or 2: ").strip()
        if turtle_choice not in ['1', '2']:
            rospy.logwarn("Invalid input. Please enter 1 or 2.")
            continue

        linear_velocity = float(input("Enter linear velocity: ").strip())
        angular_velocity = float(input("Enter angular velocity: ").strip())

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Predict new positions
        if turtle1_pose and turtle2_pose:
            if turtle_choice == '1':
                new_x, new_y = predict_new_position(turtle1_pose, twist)
                new_distance = math.sqrt((new_x - turtle2_pose.x)**2 + (new_y - turtle2_pose.y)**2)
            else:
                new_x, new_y = predict_new_position(turtle2_pose, twist)
                new_distance = math.sqrt((new_x - turtle1_pose.x)**2 + (new_y - turtle1_pose.y)**2)

            # Validate movement
            if distance < DISTANCE_THRESHOLD:
                if new_distance < DISTANCE_THRESHOLD:
                    rospy.logwarn("Turtles are too close! Only commands increasing distance are allowed.")
                    continue
                else:
                    rospy.loginfo("Command allowed as it increases the distance.")
            if not (BOUNDARY_MIN <= new_x <= BOUNDARY_MAX and BOUNDARY_MIN <= new_y <= BOUNDARY_MAX):
                rospy.logwarn("Command would move turtle out of bounds. Ignored.")
                continue

        # Publish the twist
        if turtle_choice == '1':
            pub_turtle1.publish(twist)
        else:
            pub_turtle2.publish(twist)

        rospy.sleep(1)  # Move for 1 second
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if turtle_choice == '1':
            pub_turtle1.publish(twist)
        else:
            pub_turtle2.publish(twist)

if __name__ == '__main__':
    rospy.init_node('ui_node')

    spawn_turtles()
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)
    rospy.Subscriber('/turtles_distance', Float32, distance_callback)

    control_turtle()

