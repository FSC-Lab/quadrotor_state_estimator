#ifndef STATE_ESTIMATOR_STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_STATE_ESTIMATOR_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <state_estimator/Mocap.h>
#include <std_msgs/Bool.h>

namespace fsc {

class StateEstimatorNode
{
public:

    struct mocapWatchdogState {
        bool mocapRecieved{false};
        uint32_t mocapTimeoutTime{10};
    };

    void Update(void);
    StateEstimatorNode(ros::NodeHandle& n);
    ~StateEstimatorNode() = default;

private:
    void GetMocapMsg(const state_estimator::Mocap::ConstPtr &msg);
    void GetGPSMsg(const nav_msgs::Odometry::ConstPtr &msg);
    void PubPose(void);
    void CheckEstimator(void);
    void SetMocapFlag(void);
    
    ros::Subscriber localPositionSub;
    ros::Publisher statePub;
    ros::Publisher estimatorTypePub;
    bool indoorMode{false};
    uint64_t loopCounter{0};
    uint64_t loopThreshold{20};
    nav_msgs::Odometry state;
};
};
#endif