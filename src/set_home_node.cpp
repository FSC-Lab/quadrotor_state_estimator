#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class SetHomeNode
{
public:
    SetHomeNode()
    {
        home_position_.x = 0.0;
        home_position_.y = 0.0;
        home_position_.z = 0.0;

        current_odom_.pose.pose.position.x = 0.0;
        current_odom_.pose.pose.position.y = 0.0;
        current_odom_.pose.pose.position.z = 0.0;

        diff_position_.pose.pose.position.x = 0.0;
        diff_position_.pose.pose.position.y = 0.0;
        diff_position_.pose.pose.position.z = 0.0;

        ros::NodeHandle nh;
        set_home_server_ = nh.advertiseService("/state_estimator/override_set_home", &SetHomeNode::setHomeCallback, this);

        // subscribe to the current position in mavros gps
        current_position_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &SetHomeNode::currentPositionCallback, this);

        // advertise the difference between the home position and the current position
        diff_position_pub_ = nh.advertise<nav_msgs::Odometry>("/state_estimator/local_position/odom_adjusted", 1);

        ROS_INFO("Set Home Node Initialized"); 

        ros::Rate rate(60);

        while (ros::ok())
        {
            computeDifference();
            diff_position_pub_.publish(diff_position_);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::ServiceServer set_home_server_;
    ros::Subscriber current_position_sub_;
    ros::Publisher diff_position_pub_;

    geometry_msgs::Point home_position_;
    nav_msgs::Odometry current_odom_;
    nav_msgs::Odometry diff_position_;

    bool setHomeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        home_position_.x = current_odom_.pose.pose.position.x;
        home_position_.y = current_odom_.pose.pose.position.y;
        home_position_.z = current_odom_.pose.pose.position.z;

        ROS_INFO("Home position set to: x: %f, y: %f, z: %f", home_position_.x, home_position_.y, home_position_.z);

        return true;
    }

    void currentPositionCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // convert odom to pose stamped
        current_odom_.header = msg->header;

        current_odom_.pose = msg->pose;
        current_odom_.twist = msg->twist;
    }

    void computeDifference()
    {
        diff_position_ = current_odom_; // copy the current config
        diff_position_.pose.pose.position.x = current_odom_.pose.pose.position.x - home_position_.x;
        diff_position_.pose.pose.position.y = current_odom_.pose.pose.position.y - home_position_.y;
        diff_position_.pose.pose.position.z = current_odom_.pose.pose.position.z - home_position_.z;
        diff_position_.twist = current_odom_.twist;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_home_node");
    SetHomeNode set_home_node;
    return 0;
}
