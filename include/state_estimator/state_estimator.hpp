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
#ifndef STATE_ESTIMATOR_STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_STATE_ESTIMATOR_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <optitrack_broadcast/Mocap.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>

namespace fsc
{

    class StateEstimatorNode
    {
    public:
        struct mocapWatchdogState
        {
            bool mocapRecieved{false};
            uint32_t mocapTimeoutTime{10};
        };

        void Update(void);
        StateEstimatorNode(ros::NodeHandle &n);
        ~StateEstimatorNode() = default;

    private:
        void GetMocapMsg(const optitrack_broadcast::Mocap::ConstPtr &msg);
        void GetGPSMsg(const nav_msgs::Odometry::ConstPtr &msg);
        void PubPose(void);
        void CheckEstimator(void);
        void SetMocapFlag(void);

        ros::Subscriber localPositionSub;
        ros::Subscriber velocitySub;
        ros::Publisher statePub;
        ros::Publisher estimatorTypePub;
        ros::Publisher visionPosePub;
        bool indoorMode{false};
        uint64_t loopCounter{0};
        uint64_t loopThreshold{20};
        nav_msgs::Odometry state;
        geometry_msgs::PoseStamped vision_pose;
    };
};
#endif
