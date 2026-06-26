
#pragma once
#include "ros/ros.h"
#include <thread>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>
#include <memory>
#include <mutex>
#include <fstream>
#include <vector>
#include <atomic>
#include "rosdemo_v4/EnableRobot.h"
#include "rosdemo_v4/DisableRobot.h"
#include "rosdemo_v4/ClearError.h"
#include "rosdemo_v4/GetErrorID.h"
#include "rosdemo_v4/MovJ.h"

class RosDemoCRV4
{
public:
    RosDemoCRV4(ros::NodeHandle* nh);
    ~RosDemoCRV4();
    void movePoint(const std::vector<double>& pointA, int& id);
    void finishPoint(int id);
    void stop();

private:
    ros::ServiceClient m_enableRobot;
    ros::ServiceClient m_disableRobot;
    ros::ServiceClient m_clearError;
    ros::ServiceClient m_getErrorID;
    ros::ServiceClient m_movj;
    ros::Subscriber subFeedInfo;
    std::mutex m_mutex;
    std::thread threadClearRobotError;
    std::thread threadParseRobotError;
    std::atomic<bool> m_running{ true };
    
    struct FeedInfo
    {
        int EnableStatus;
        int ErrorStatus;
        int RobotMode;
        int CurrentCommandID;
    };
    FeedInfo feedbackData;
    bool stateFinish{ false };
    nlohmann::json m_JsonDataController;
    nlohmann::json m_JsonDataServo;

private:
    void warmRobotError();
    void getFeedBackInfo(const std_msgs::String::ConstPtr& msg);

    template <typename T>
    bool SendService(ros::ServiceClient serviceClient, T& arg);
    void parseRobotAlarm();
};

template <typename T>
bool RosDemoCRV4::SendService(ros::ServiceClient serviceClient, T& arg)
{
    if (serviceClient.call(arg)) {
        return true;
    }
    return false;
}