#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose  # Import the Pose message
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

distance = float('inf')  # Current distance between turtles
DISTANCE_THRESHOLD = 2.0  # Minimum safe distance between turtles

# Current poses of turtles
turtle1_pose = None
turtle2_pose = None


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
    """
    Predict the new position of a turtle given its current pose and velocity.
    :param pose: The current pose of the turtle (Pose message).
    :param twist: The velocity command (Twist message).
    :param duration: Duration for which the velocity will be applied (default: 1 second).
    :return: A tuple (new_x, new_y) representing the new position.
    """
    theta = pose.theta
    new_x = pose.x + twist.linear.x * math.cos(theta) * duration
    new_y = pose.y + twist.linear.x * math.sin(theta) * duration
    return new_x, new_y


def calculate_distance(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def spawn_turtles():
    """Spawns turtle1 and turtle2, ensuring no naming conflicts."""
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/kill')
    try:
        kill_service = rospy.ServiceProxy('/kill', Kill)
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)

        # Remove default turtle1 if it exists
        try:
            kill_service('turtle1')
            rospy.loginfo("Default turtle1 removed.")
        except rospy.ServiceException:
            rospy.loginfo("Default turtle1 does not exist. Continuing...")

        # Spawn a new turtle1
        spawn_service(8.0, 5.5, 0.0, 'turtle1')
        rospy.loginfo("Turtle1 spawned successfully.")

        # Spawn turtle2
        spawn_service(3.0, 5.5, 0.0, 'turtle2')
        rospy.loginfo("Turtle2 spawned successfully.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtles: {e}")


def control_turtle():
    """Allows the user to control the turtles while enforcing safety rules."""
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        rospy.loginfo("Select the turtle to control: 1 for turtle1, 2 for turtle2")
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
        if turtle_choice == '1' and turtle1_pose and turtle2_pose:
            new_x, new_y = predict_new_position(turtle1_pose, twist)
            new_distance = calculate_distance(new_x, new_y, turtle2_pose.x, turtle2_pose.y)
        elif turtle_choice == '2' and turtle1_pose and turtle2_pose:
            new_x, new_y = predict_new_position(turtle2_pose, twist)
            new_distance = calculate_distance(new_x, new_y, turtle1_pose.x, turtle1_pose.y)
        else:
            rospy.logwarn("Unable to predict new positions. Skipping...")
            continue

        if distance < DISTANCE_THRESHOLD and new_distance < DISTANCE_THRESHOLD:
            rospy.logwarn("Turtles are too close! Movement is blocked.")
            continue
        elif distance < DISTANCE_THRESHOLD and new_distance >= DISTANCE_THRESHOLD:
            rospy.loginfo("Movement unblocked as it increases the distance.")
        
        # Publish the twist command
        if turtle_choice == '1':
            pub_turtle1.publish(twist)
        elif turtle_choice == '2':
            pub_turtle2.publish(twist)

        rospy.sleep(1)  # Publish for 1 second
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if turtle_choice == '1':
            pub_turtle1.publish(twist)
        elif turtle_choice == '2':
            pub_turtle2.publish(twist)


if __name__ == '__main__':
    rospy.init_node('ui_node')

    # Subscribe to the distance topic and poses
    rospy.Subscriber('/turtles_distance', Float32, distance_callback)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)

    spawn_turtles()
    control_turtle()

