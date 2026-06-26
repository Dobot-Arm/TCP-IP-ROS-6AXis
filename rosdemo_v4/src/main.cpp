#include "rosDemoCRV4.h"
#include <atomic>
#include <signal.h>

std::atomic<bool> g_shutdown_requested(false);

void sigintHandler(int sig)
{
    g_shutdown_requested.store(true);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "rosdemo_v4", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigintHandler);
    
    ros::NodeHandle nh("~");
    
    std::vector<double> pointA;
    std::vector<double> pointB;
    
    if (!nh.getParam("/rosdemo_v4/pointA", pointA)) {
        ROS_WARN("Failed to get parameter 'pointA', using default values");
        pointA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    if (!nh.getParam("/rosdemo_v4/pointB", pointB)) {
        ROS_WARN("Failed to get parameter 'pointB', using default values");
        pointB = {0.5, 0.0, 0.5, 0.0, 0.0, 0.0};
    }
    
    if (pointA.size() < 6) {
        ROS_ERROR("pointA must have at least 6 elements");
        return -1;
    }
    
    if (pointB.size() < 6) {
        ROS_ERROR("pointB must have at least 6 elements");
        return -1;
    }
    
    RosDemoCRV4 serviceHandler(&nh);
    
    std::thread threadMove([&serviceHandler, pointA, pointB]() {
        int currentCommandID = 2147483647;
        while (!g_shutdown_requested.load()) {
            serviceHandler.movePoint(pointA, currentCommandID);
            serviceHandler.finishPoint(currentCommandID);
            
            if (g_shutdown_requested.load()) {
                break;
            }
            
            serviceHandler.movePoint(pointB, currentCommandID);
            serviceHandler.finishPoint(currentCommandID);
        }
        ROS_INFO("Move thread exiting");
    });
    
    ros::spin();
    
    g_shutdown_requested.store(true);
    if (threadMove.joinable()) {
        threadMove.join();
    }
    
    ROS_INFO("rosdemo_v4 node exiting");
    return 0;
}
