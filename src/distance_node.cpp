#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <cmath>  // For sqrt()

// Subscribers for turtle poses
ros::Subscriber turtle1_pose_sub;
ros::Subscriber turtle2_pose_sub;

// Publisher for distance between turtles
ros::Publisher distance_pub;

// Variables to store the current poses of the turtles
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

// Function to calculate the distance between two turtles
float calculateDistance() {
    float x_diff = turtle1_pose.x - turtle2_pose.x;
    float y_diff = turtle1_pose.y - turtle2_pose.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

// Callback function to update turtle1 pose
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

// Callback function to update turtle2 pose
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
}

// Callback function to publish distance between turtles
void publishDistance() {
    float distance = calculateDistance();
    std_msgs::Float32 msg;
    msg.data = distance;
    distance_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Subscribe to both turtles' poses
    turtle1_pose_sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    turtle2_pose_sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Publisher to send the calculated distance
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);

    // Main loop
    ros::Rate loop_rate(10);  // Set loop rate to 10 Hz
    while (ros::ok()) {
        publishDistance();  // Publish the distance between the turtles
        ros::spinOnce();    // Process incoming messages
        loop_rate.sleep();  // Sleep to maintain loop rate
    }

    return 0;
}

