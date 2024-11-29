#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Globals
turtle1_pose = Pose()
turtle2_pose = Pose()
distance_pub = None

# Thresholds
DISTANCE_THRESHOLD = 2.0  # Minimum safe distance between turtles
BOUNDARY_MIN = 1.0  # Minimum x, y boundaries
BOUNDARY_MAX = 10.0  # Maximum x, y boundaries

def calculate_distance():
    """Calculate the Euclidean distance between the two turtles."""
    x_diff = turtle1_pose.x - turtle2_pose.x
    y_diff = turtle1_pose.y - turtle2_pose.y
    return (x_diff**2 + y_diff**2)**0.5

def check_boundaries(pose):
    """Check if a turtle is within the allowed boundaries."""
    return BOUNDARY_MIN <= pose.x <= BOUNDARY_MAX and BOUNDARY_MIN <= pose.y <= BOUNDARY_MAX

def turtle1_callback(data):
    """Update the global pose for turtle1 and publish distance."""
    global turtle1_pose
    turtle1_pose = data
    publish_distance()

def turtle2_callback(data):
    """Update the global pose for turtle2 and publish distance."""
    global turtle2_pose
    turtle2_pose = data
    publish_distance()

def publish_distance():
    """Publish the distance between turtle1 and turtle2."""
    global distance_pub
    if turtle1_pose and turtle2_pose:
        distance = calculate_distance()
        distance_pub.publish(distance)
        rospy.loginfo(f"Distance between turtles: {distance:.2f}")

if __name__ == '__main__':
    rospy.init_node('distance_node')

    # Subscribers for poses
    rospy.Subscriber('/turtle1/pose', Pose, turtle1_callback)
    rospy.Subscriber('/turtle2/pose', Pose, turtle2_callback)

    # Publisher for distance
    distance_pub = rospy.Publisher('/turtles_distance', Float32, queue_size=10)

    rospy.spin()

