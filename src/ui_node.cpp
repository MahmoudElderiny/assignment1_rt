#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <cmath>  // For sqrt()

// Publishers for turtle velocities
ros::Publisher turtle1_vel_pub;
ros::Publisher turtle2_vel_pub;

// Subscribers for turtle poses
ros::Subscriber turtle1_pose_sub;
ros::Subscriber turtle2_pose_sub;

// Variables to store the current poses of the turtles
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

// Constants for boundary checking and distance threshold
const float boundary_limit = 10.5; // Boundary limit of turtlesim screen
const float min_distance_threshold = 1.0; // Minimum distance for the turtles to be apart

// Function to calculate the distance between two turtles
float calculateDistance() {
    float x_diff = turtle1_pose.x - turtle2_pose.x;
    float y_diff = turtle1_pose.y - turtle2_pose.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

// Function to check if turtles are too close
bool canMove() {
    float distance = calculateDistance();
    return distance >= min_distance_threshold; // Allow movement only if distance is large enough
}

// Function to handle movement of turtle1
void moveTurtle1(const geometry_msgs::Twist& vel_msg) {
    if (turtle1_pose.x < boundary_limit && turtle1_pose.y < boundary_limit && canMove()) {
        turtle1_vel_pub.publish(vel_msg);
    } else {
        ROS_WARN("Movement blocked: Boundary or too close to turtle2");
    }
}

// Function to handle movement of turtle2
void moveTurtle2(const geometry_msgs::Twist& vel_msg) {
    if (turtle2_pose.x < boundary_limit && turtle2_pose.y < boundary_limit && canMove()) {
        turtle2_vel_pub.publish(vel_msg);
    } else {
        ROS_WARN("Movement blocked: Boundary or too close to turtle1");
    }
}

// Callback function to update turtle1 pose
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

// Callback function to update turtle2 pose
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
}

// Callback function to spawn turtles
void spawnTurtles() {
    // Spawn turtle1 at (5, 5)
    ros::service::call("/spawn", turtlesim::SpawnRequest(5.0, 5.0, 0.0));
    // Spawn turtle2 at (2, 2)
    ros::service::call("/spawn", turtlesim::SpawnRequest(2.0, 2.0, 0.0));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Advertise the velocity topics for both turtles
    turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle2_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Subscribe to both turtles' poses
    turtle1_pose_sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    turtle2_pose_sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Ensure both turtles are spawned
    spawnTurtles();

    // Command loop for user input (simulating user controlling the turtles)
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);  // Set loop rate to 10 Hz
    while (ros::ok()) {
        // Simulate user input here or use std::cin for interactive control
        vel_msg.linear.x = 2.0;  // Move forward for turtle1
        vel_msg.angular.z = 0.0; // No rotation for turtle1
        moveTurtle1(vel_msg);     // Move turtle1

        vel_msg.linear.x = 0.0;   // Stop turtle2 forward movement
        vel_msg.angular.z = 1.0;  // Rotate turtle2
        moveTurtle2(vel_msg);     // Move turtle2

        ros::spinOnce();          // Process incoming messages
        loop_rate.sleep();        // Sleep to maintain loop rate
    }

    return 0;
}

