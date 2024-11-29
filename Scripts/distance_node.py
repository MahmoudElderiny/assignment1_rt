#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Global pose variables for turtles
turtle1_pose = Pose()
turtle2_pose = Pose()
distance_pub = None

# Thresholds
DISTANCE_THRESHOLD = 2.0  # Minimum safe distance between turtles
BOUNDARY_MIN = 1.0  # Minimum x, y boundaries
BOUNDARY_MAX = 10.0  # Maximum x, y boundaries


def turtle1_callback(data):
    """Update the global pose for turtle1."""
    global turtle1_pose
    turtle1_pose = data
    check_safety("turtle1")


def turtle2_callback(data):
    """Update the global pose for turtle2."""
    global turtle2_pose
    turtle2_pose = data
    check_safety("turtle2")


def calculate_distance():
    """Calculate the Euclidean distance between the two turtles."""
    x_diff = turtle1_pose.x - turtle2_pose.x
    y_diff = turtle1_pose.y - turtle2_pose.y
    return (x_diff**2 + y_diff**2)**0.5


def check_boundaries(pose):
    """Check if a turtle is within the allowed boundaries."""
    return BOUNDARY_MIN <= pose.x <= BOUNDARY_MAX and BOUNDARY_MIN <= pose.y <= BOUNDARY_MAX


def stop_turtle(turtle_name):
    """Stop the specified turtle."""
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    stop_twist = Twist()
    pub.publish(stop_twist)
    rospy.logwarn(f"{turtle_name} stopped due to safety rules.")


def check_safety(turtle_name):
    """Check if movement violates safety rules."""
    global turtle1_pose, turtle2_pose

    # Calculate distance
    distance = calculate_distance()
    distance_pub.publish(distance)

    # Check if turtles are too close
    if distance < DISTANCE_THRESHOLD:
        rospy.logwarn("Turtles are too close! Movement blocked.")
        stop_turtle(turtle_name)
        return

    # Check boundaries
    pose = turtle1_pose if turtle_name == "turtle1" else turtle2_pose
    if not check_boundaries(pose):
        rospy.logwarn(f"{turtle_name} is too close to the boundary! Movement blocked.")
        stop_turtle(turtle_name)


if __name__ == '__main__':
    rospy.init_node('distance_node')

    # Subscribers for turtle poses
    rospy.Subscriber('/turtle1/pose', Pose, turtle1_callback)
    rospy.Subscriber('/turtle2/pose', Pose, turtle2_callback)

    # Publisher for distance
    distance_pub = rospy.Publisher('/turtles_distance', Float32, queue_size=10)

    rospy.spin()

