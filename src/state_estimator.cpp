/*

MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include "state_estimator/state_estimator.hpp"

namespace fsc
{

    using namespace std::string_literals;
    StateEstimatorNode::StateEstimatorNode(ros::NodeHandle &n)
    // statePub(nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom/UAV0", 1)),
    // estimatorTypePub(nh.advertise<std_msgs::Bool>("/estimator_type", 1))
    {
        // ros::NodeHandle nh("~");
        // nh.param("indoorMode", indoorMode, true);
        std::string uav_prefix;
        ros::NodeHandle pnh("~");
        pnh.param("uav_prefix", uav_prefix, ""s);
        pnh.param("indoorMode", indoorMode, true);

        // initialize subscriber
        if (indoorMode)
        {
            localPositionSub = n.subscribe(uav_prefix + "/mocap/UAV0", 1, &StateEstimatorNode::GetMocapMsg, this);
            visionPosePub = n.advertise<geometry_msgs::PoseStamped>(uav_prefix + "/mavros/vision_pose/pose", 1);
        }
        else
        {
            localPositionSub = n.subscribe(uav_prefix + "/state_estimator/local_position/odom_adjusted", 1, &StateEstimatorNode::GetGPSMsg, this);
        }
        statePub = n.advertise<nav_msgs::Odometry>(uav_prefix + "/state_estimator/local_position/odom", 1);
        estimatorTypePub = n.advertise<std_msgs::Bool>(uav_prefix + "/estimator_type", 1);

        ROS_INFO("Starting state estimator node for UAV %s in %s mode", uav_prefix.c_str(), (indoorMode ? "indoor" : "outdoor"));
    }

    void StateEstimatorNode::CheckEstimator(void)
    {
        if (loopCounter >= loopThreshold)
        {
            loopCounter = 0;
            std_msgs::Bool success;
            success.data = (indoorMode) ? true : false;
            estimatorTypePub.publish(success);
        }
        loopCounter++;
    }

    void SetMocapFlag(void)
    {
    }

    void StateEstimatorNode::GetMocapMsg(const optitrack_broadcast::Mocap::ConstPtr &msg)
    {
        state.header.stamp = ros::Time::now();
        state.header.frame_id = "map";
        state.header.seq = msg->header.seq;

        state.header = msg->header;
        state.pose.pose = msg->pose;
        state.twist.twist = msg->twist;

        vision_pose.header = msg->header;
        vision_pose.pose = msg->pose;
    }

    void StateEstimatorNode::GetGPSMsg(const nav_msgs::Odometry::ConstPtr &msg)
    {
        state.header.stamp = ros::Time::now();
        state.header.frame_id = "map";
        state.header.seq = msg->header.seq;
        state.pose = msg->pose;
        state.twist = msg->twist;
    }

    void StateEstimatorNode::PubPose(void)
    {
        statePub.publish(state);
        if (indoorMode)
        {
            visionPosePub.publish(vision_pose);
        }
    }

    void StateEstimatorNode::Update(void)
    {
        CheckEstimator();
        PubPose();
    }

}
