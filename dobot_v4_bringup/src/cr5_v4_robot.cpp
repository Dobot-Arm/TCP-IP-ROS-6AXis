/**
 ***********************************************************************************************************************
 *
 * @author YangXiBo
 * @date   2023/08/18
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

// GetToolDo is not definded

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_v4_bringup/cr5_v4_robot.h>

// #include <dobot_v4_bringup/parseTool.h>

CRRobot::CRRobot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

CRRobot::~CRRobot()
{
    ROS_INFO("~CRRobot");
}

void CRRobot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");
    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    int numRobotNodes = control_nh_.param("num_nodes", 1);
    std::string serviceRobotName{ "" };
    std::string serviceProjectName{ "/dobot_v4_bringup/" };
    if (numRobotNodes > 1) {
        serviceRobotName = control_nh_.param<std::string>("robot_node_name", "robot");
        serviceRobotName = serviceRobotName + "/";
    };

    std::string serviceEnableRobot = serviceProjectName + serviceRobotName + "srv/EnableRobot";
    std::string serviceDisableRobot = serviceProjectName + serviceRobotName + "srv/DisableRobot";
    std::string serviceClearError = serviceProjectName + serviceRobotName + "srv/ClearError";
    std::string serviceSpeedFactor = serviceProjectName + serviceRobotName + "srv/SpeedFactor";
    std::string serviceUser = serviceProjectName + serviceRobotName + "srv/User";
    std::string serviceTool = serviceProjectName + serviceRobotName + "srv/Tool";
    std::string serviceRobotMode = serviceProjectName + serviceRobotName + "srv/RobotMode";
    std::string serviceSetPayLoad = serviceProjectName + serviceRobotName + "srv/SetPayLoad";
    std::string serviceDO = serviceProjectName + serviceRobotName + "srv/DO";
    std::string serviceDOInstant = serviceProjectName + serviceRobotName + "srv/DOInstant";
    std::string serviceToolDO = serviceProjectName + serviceRobotName + "srv/ToolDO";
    std::string serviceToolDOInstant = serviceProjectName + serviceRobotName + "srv/ToolDOInstant";
    std::string serviceAO = serviceProjectName + serviceRobotName + "srv/AO";
    std::string serviceAOInstant = serviceProjectName + serviceRobotName + "srv/AOInstant";
    std::string serviceAccJ = serviceProjectName + serviceRobotName + "srv/AccJ";
    std::string serviceAccL = serviceProjectName + serviceRobotName + "srv/AccL";
    std::string serviceVelJ = serviceProjectName + serviceRobotName + "srv/VelJ";
    std::string serviceVelL = serviceProjectName + serviceRobotName + "srv/VelL";
    std::string serviceCP = serviceProjectName + serviceRobotName + "srv/CP";
    std::string servicePowerOn = serviceProjectName + serviceRobotName + "srv/PowerOn";
    std::string serviceRunScript = serviceProjectName + serviceRobotName + "srv/RunScript";
    std::string serviceStop = serviceProjectName + serviceRobotName + "srv/Stop";
    std::string servicePause = serviceProjectName + serviceRobotName + "srv/Pause";
    std::string serviceContinue = serviceProjectName + serviceRobotName + "srv/Continue";

    std::string serviceEnableSafeSkin = serviceProjectName + serviceRobotName + "srv/EnableSafeSkin";
    std::string serviceSetSafeSkin = serviceProjectName + serviceRobotName + "srv/SetSafeSkin";
    std::string serviceGetStartPose = serviceProjectName + serviceRobotName + "srv/GetStartPose";
    std::string serviceStartPath = serviceProjectName + serviceRobotName + "srv/StartPatht";
    std::string servicePositiveKin = serviceProjectName + serviceRobotName + "srv/PositiveKin";
    std::string serviceInverseKin = serviceProjectName + serviceRobotName + "srv/InverseKin";
    std::string serviceGetAngle = serviceProjectName + serviceRobotName + "srv/GetAngle";
    std::string serviceGetPose = serviceProjectName + serviceRobotName + "srv/GetPose";
    std::string serviceSetCollisionLevel = serviceProjectName + serviceRobotName + "srv/SetCollisionLevel";
    std::string serviceEmergencyStop = serviceProjectName + serviceRobotName + "srv/EmergencyStop";
    std::string serviceModbusRTUCreate = serviceProjectName + serviceRobotName + "srv/ModbusRTUCreate";
    std::string serviceModbusCreate = serviceProjectName + serviceRobotName + "srv/ModbusCreate";
    std::string serviceModbusClose = serviceProjectName + serviceRobotName + "srv/ModbusClose";
    std::string serviceGetInBits = serviceProjectName + serviceRobotName + "srv/GetInBits";
    std::string serviceGetInRegs = serviceProjectName + serviceRobotName + "srv/GetInRegs";
    std::string serviceGetCoils = serviceProjectName + serviceRobotName + "srv/GetCoils";
    std::string serviceSetCoils = serviceProjectName + serviceRobotName + "srv/SetCoils";
    std::string serviceGetHoldRegs = serviceProjectName + serviceRobotName + "srv/GetHoldRegs";
    std::string serviceSetHoldRegs = serviceProjectName + serviceRobotName + "srv/SetHoldRegs";

    std::string serviceGetErrorID = serviceProjectName + serviceRobotName + "srv/GetErrorID";
    std::string serviceDI = serviceProjectName + serviceRobotName + "srv/DI";
    std::string serviceToolDI = serviceProjectName + serviceRobotName + "srv/ToolDI";
    std::string serviceAI = serviceProjectName + serviceRobotName + "srv/AI";
    std::string serviceToolAI = serviceProjectName + serviceRobotName + "srv/ToolAI";
    std::string serviceDIGroup = serviceProjectName + serviceRobotName + "srv/DIGroup";
    std::string serviceDOGroup = serviceProjectName + serviceRobotName + "srv/DoGroup";
    std::string serviceBrakeControl = serviceProjectName + serviceRobotName + "srv/BrakeControl";
    std::string serviceStartDrag = serviceProjectName + serviceRobotName + "srv/StartDrag";
    std::string serviceStopDrag = serviceProjectName + serviceRobotName + "srv/StopDrag";
    std::string serviceDragSensivity = serviceProjectName + serviceRobotName + "srv/DragSensivity";
    std::string serviceGetDO = serviceProjectName + serviceRobotName + "srv/GetDO";
    std::string serviceGetAO = serviceProjectName + serviceRobotName + "srv/GetAO";
    std::string serviceGetDOGroup = serviceProjectName + serviceRobotName + "srv/GetDOGroup";
    std::string serviceSetTool485 = serviceProjectName + serviceRobotName + "srv/SetTool485";
    std::string serviceSetSafeWallEnable = serviceProjectName + serviceRobotName + "srv/SetSafeWallEnable";
    std::string serviceSetToolPower = serviceProjectName + serviceRobotName + "srv/SetToolPower";
    std::string serviceSetToolMode = serviceProjectName + serviceRobotName + "srv/SetToolMode";
    std::string serviceSetBackDistance = serviceProjectName + serviceRobotName + "srv/SetBackDistance";
    std::string serviceSetPostCollisionMode = serviceProjectName + serviceRobotName + "srv/SetPostCollisionMode";
    std::string serviceSetUser = serviceProjectName + serviceRobotName + "srv/SetUser";
    std::string serviceSetTool = serviceProjectName + serviceRobotName + "srv/SetTool";
    std::string serviceCalcUser = serviceProjectName + serviceRobotName + "srv/CalcUser";
    std::string serviceCalcTool = serviceProjectName + serviceRobotName + "srv/CalcTool";
    std::string serviceGetInputBool = serviceProjectName + serviceRobotName + "srv/GetInputBool";
    std::string serviceGetInputInt = serviceProjectName + serviceRobotName + "srv/GetInputInt";
    std::string serviceGetInputFloat = serviceProjectName + serviceRobotName + "srv/GetInputFloat";
    std::string serviceGetOutputBool = serviceProjectName + serviceRobotName + "srv/GetOutputBool";
    std::string serviceGetOutputInt = serviceProjectName + serviceRobotName + "srv/GetOutputInt";
    std::string serviceGetOutputFloat = serviceProjectName + serviceRobotName + "srv/GetOutputFloat";
    std::string serviceSetOutputBool = serviceProjectName + serviceRobotName + "srv/SetOutputBool";
    std::string serviceSetOutputInt = serviceProjectName + serviceRobotName + "srv/SetOutputInt";
    std::string serviceSetOutputFloat = serviceProjectName + serviceRobotName + "srv/SetOutputFloat";

    std::string serviceMovJ = serviceProjectName + serviceRobotName + "srv/MovJ";
    std::string serviceMovL = serviceProjectName + serviceRobotName + "srv/MovL";
    std::string serviceMovLIO = serviceProjectName + serviceRobotName + "srv/MovLIO";
    std::string serviceMovJIO = serviceProjectName + serviceRobotName + "srv/MovJIO";
    std::string serviceArc = serviceProjectName + serviceRobotName + "srv/Arc";
    std::string serviceCircle = serviceProjectName + serviceRobotName + "srv/Circle";
    std::string serviceMoveJog = serviceProjectName + serviceRobotName + "srv/MoveJog";
    std::string serviceStopMoveJog = serviceProjectName + serviceRobotName + "srv/StopMoveJog";
    std::string serviceRelMovJTool = serviceProjectName + serviceRobotName + "srv/RelMovJTool";
    std::string serviceRelMovLTool = serviceProjectName + serviceRobotName + "srv/RelMovLTool";
    std::string serviceRelMovJUser = serviceProjectName + serviceRobotName + "srv/RelMovJUser";
    std::string serviceRelMovLUser = serviceProjectName + serviceRobotName + "srv/RelMovLUser";
    std::string serviceRelJointMovJ = serviceProjectName + serviceRobotName + "srv/RelJointMovJ";
    std::string serviceGetCurrentCommandId = serviceProjectName + serviceRobotName + "srv/GetCurrentCommandId";
    std::string serviceServoJ = serviceProjectName + serviceRobotName + "srv/ServoJ";
    std::string serviceServoP = serviceProjectName + serviceRobotName + "srv/ServoP";
    std::string topicFeedInfo = serviceProjectName + serviceRobotName + "/msg/FeedInfo";

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();
    server_tbl_.push_back(control_nh_.advertiseService(serviceEnableRobot, &CRRobot::enableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDisableRobot, &CRRobot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceClearError, &CRRobot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSpeedFactor, &CRRobot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceUser, &CRRobot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceTool, &CRRobot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRobotMode, &CRRobot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetPayLoad, &CRRobot::setPayload, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDO, &CRRobot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDOInstant, &CRRobot::DOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDO, &CRRobot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDOInstant, &CRRobot::toolDOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAO, &CRRobot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAOInstant, &CRRobot::AOInstant, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAccJ, &CRRobot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAccL, &CRRobot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceVelJ, &CRRobot::velJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceVelL, &CRRobot::velL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCP, &CRRobot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePowerOn, &CRRobot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRunScript, &CRRobot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStop, &CRRobot::stop, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePause, &CRRobot::pause, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceContinue, &CRRobot::Continue, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceEnableSafeSkin, &CRRobot::EnableSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeSkin, &CRRobot::SetSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetStartPose, &CRRobot::GetStartPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartPath, &CRRobot::StartPath, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePositiveKin, &CRRobot::PositiveKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceInverseKin, &CRRobot::InverseKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAngle, &CRRobot::GetAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetPose, &CRRobot::GetPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCollisionLevel, &CRRobot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceEmergencyStop, &CRRobot::EmergencyStop, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusRTUCreate, &CRRobot::ModbusRTUCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusCreate, &CRRobot::ModbusCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusClose, &CRRobot::ModbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInBits, &CRRobot::GetInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInRegs, &CRRobot::GetInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetCoils, &CRRobot::GetCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCoils, &CRRobot::SetCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetHoldRegs, &CRRobot::GetHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetHoldRegs, &CRRobot::SetHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetErrorID, &CRRobot::GetErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDI, &CRRobot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDI, &CRRobot::ToolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAI, &CRRobot::AI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolAI, &CRRobot::ToolAI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDIGroup, &CRRobot::DIGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDOGroup, &CRRobot::doGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceBrakeControl, &CRRobot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartDrag, &CRRobot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopDrag, &CRRobot::StopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDragSensivity, &CRRobot::DragSensivity, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDO, &CRRobot::GetDO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAO, &CRRobot::GetAO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDOGroup, &CRRobot::GetDOGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool485, &CRRobot::SetTool485, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeWallEnable, &CRRobot::SetSafeWallEnable, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolPower, &CRRobot::SetToolPower, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolMode, &CRRobot::SetToolMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetBackDistance, &CRRobot::SetBackDistance, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceSetPostCollisionMode, &CRRobot::SetPostCollisionMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetUser, &CRRobot::SetUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool, &CRRobot::SetTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcUser, &CRRobot::CalcUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcTool, &CRRobot::CalcTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputBool, &CRRobot::GetInputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputInt, &CRRobot::GetInputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputFloat, &CRRobot::GetInputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputBool, &CRRobot::GetOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputInt, &CRRobot::GetOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputFloat, &CRRobot::GetOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputBool, &CRRobot::SetOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputInt, &CRRobot::SetOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputFloat, &CRRobot::SetOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJ, &CRRobot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovL, &CRRobot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovLIO, &CRRobot::MovLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJIO, &CRRobot::MovJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceArc, &CRRobot::Arc, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCircle, &CRRobot::Circle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMoveJog, &CRRobot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopMoveJog, &CRRobot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJTool, &CRRobot::RelMovJTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLTool, &CRRobot::RelMovLTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJUser, &CRRobot::RelMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLUser, &CRRobot::RelMovLUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelJointMovJ, &CRRobot::relJointMovJ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceGetCurrentCommandId, &CRRobot::GetCurrentCommandId, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoJ, &CRRobot::ServoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoP, &CRRobot::ServoP, this));

    registerGoalCallback(boost::bind(&CRRobot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CRRobot::cancelHandle, this, _1));
    backend_task_ = control_nh_.createTimer(ros::Duration(1.5), &CRRobot::backendTask, this);

    pubFeedInfo = control_nh_.advertise<std_msgs::String>(topicFeedInfo, 1000);
    threadPubFeedBackInfo = std::thread(&CRRobot::pubFeedBackInfo, this);
    threadPubFeedBackInfo.detach();
    start();
}

void CRRobot::pubFeedBackInfo()
{
    RealTimeData* realTimeData = nullptr;
    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok()) {
        realTimeData = (const_cast<RealTimeData*>(commander_->getRealData()));
        nlohmann::json root;
        root["EnableStatus"] = realTimeData->EnableStatus;
        root["ErrorStatus"] = realTimeData->ErrorStatus;
        root["RobotMode"] = realTimeData->robot_mode;
        root["CurrentCommandID"] = realTimeData->currentCommandId;
        std::string qActualVecStr = root.dump();

        std_msgs::String msgFeedInfo;
        msgFeedInfo.data = qActualVecStr;
        pubFeedInfo.publish(msgFeedInfo);
        rate.sleep();
    }
}

void CRRobot::feedbackHandle(const ros::TimerEvent& tm,
                             ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++) {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

std::vector<double> CRRobot::sample_traj(const trajectory_msgs::JointTrajectoryPoint& P0,
                                         const trajectory_msgs::JointTrajectoryPoint& P1, const double& time_index)
{
    double a, b, c, d;
    double T = P1.time_from_start.toSec() - P0.time_from_start.toSec();
    double t = time_index;
    std::vector<double> interp_traj;
    for (int i = 0; i < P0.positions.size(); i++) {
        a = P0.positions[i];
        b = P0.velocities[i];
        c = (-3.0 * P0.positions[i] + 3.0 * P1.positions[i] - 2.0 * T * P0.velocities[i] - T * P1.velocities[i]) /
            (T * T);
        d = (2.0 * P0.positions[i] - 2.0 * P1.positions[i] + T * P0.velocities[i] + T * P1.velocities[i]) / (T * T * T);
        interp_traj.push_back((a + b * t + c * t * t + d * t * t * t) * 180.0 / M_PI);
    }
    return interp_traj;
}

void CRRobot::moveHandle(const ros::TimerEvent& tm,
                         ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = handle.getGoal();

    static const double SERVOJ_DURATION = 0.08;
    double t = SERVOJ_DURATION * 1.5;
    ros::Rate timer(1.0 / SERVOJ_DURATION);    // servoj发布频率
    double t0 = ros::Time::now().toSec();

    try {
        for (int i = 0; i < goal->trajectory.points.size() - 1; i++) {
            trajectory_msgs::JointTrajectoryPoint interp_traj_begin = goal->trajectory.points[i];
            trajectory_msgs::JointTrajectoryPoint interp_traj_end = goal->trajectory.points[i + 1];
            double real_time;    // 实际间隔时间
            double t1;
            t1 = ros::Time::now().toSec();
            real_time = t1 - t0;
            while (real_time < interp_traj_end.time_from_start.toSec() - SERVOJ_DURATION) {
                double time_index = real_time - interp_traj_begin.time_from_start.toSec();
                std::vector<double> tmp = sample_traj(interp_traj_begin, interp_traj_end, time_index);
                char cmd[100];
                sprintf(cmd, "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f)", tmp[0], tmp[1], tmp[2], tmp[3],
                        tmp[4], tmp[5], t);
                int32_t err_id;
                commander_->motionDoCmd(cmd, err_id);
                timer.sleep();
                t1 = ros::Time::now().toSec();
                real_time = t1 - t0;
            }
        }
        std::vector<double> last_traj;
        int point_num = goal->trajectory.points.size();
        for (int i = 0; i < 6; i++) {
            last_traj.push_back(goal->trajectory.points[point_num - 1].positions[i] * 180 / M_PI);
        }
        char cmd[100];
        sprintf(cmd, "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f)", last_traj[0], last_traj[1], last_traj[2],
                last_traj[3], last_traj[4], last_traj[5], t);
        int32_t err_id;
        commander_->motionDoCmd(cmd, err_id);
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        return;
    }

    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CRRobot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++) {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CRRobot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&CRRobot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CRRobot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CRRobot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CRRobot::isEnable() const
{
    return commander_->isEnable();
}

bool CRRobot::isConnected() const
{
    return commander_->isConnected();
}

void CRRobot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CRRobot::enableRobot(dobot_v4_bringup::EnableRobot::Request& request,
                          dobot_v4_bringup::EnableRobot::Response& response)
{
    return commander_->callRosService(parseTool::parserenableRobotRequest2String(request), response.res);
}

bool CRRobot::disableRobot(dobot_v4_bringup::DisableRobot::Request& request,
                           dobot_v4_bringup::DisableRobot::Response& response)
{
    return commander_->callRosService(parseTool::parserdisableRobotRequest2String(request), response.res);
}

bool CRRobot::clearError(dobot_v4_bringup::ClearError::Request& request,
                         dobot_v4_bringup::ClearError::Response& response)
{
    return commander_->callRosService(parseTool::parserclearErrorRequest2String(request), response.res);
}

bool CRRobot::speedFactor(dobot_v4_bringup::SpeedFactor::Request& request,
                          dobot_v4_bringup::SpeedFactor::Response& response)
{
    return commander_->callRosService(parseTool::parserspeedFactorRequest2String(request), response.res);
}

bool CRRobot::user(dobot_v4_bringup::User::Request& request, dobot_v4_bringup::User::Response& response)
{
    return commander_->callRosService(parseTool::parseruserRequest2String(request), response.res);
}

bool CRRobot::tool(dobot_v4_bringup::Tool::Request& request, dobot_v4_bringup::Tool::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolRequest2String(request), response.res);
}

bool CRRobot::robotMode(dobot_v4_bringup::RobotMode::Request& request, dobot_v4_bringup::RobotMode::Response& response)
{
    return commander_->callRosService(parseTool::parserrobotModeRequest2String(request), response.res);
}

bool CRRobot::setPayload(dobot_v4_bringup::SetPayload::Request& request,
                         dobot_v4_bringup::SetPayload::Response& response)
{
    return commander_->callRosService(parseTool::parsersetPayloadRequest2String(request), response.res);
}

bool CRRobot::DO(dobot_v4_bringup::DO::Request& request, dobot_v4_bringup::DO::Response& response)
{
    return commander_->callRosService(parseTool::parserDORequest2String(request), response.res);
}

bool CRRobot::DOInstant(dobot_v4_bringup::DOInstant::Request& request, dobot_v4_bringup::DOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parserDOInstantRequest2String(request), response.res);
}

bool CRRobot::toolDO(dobot_v4_bringup::ToolDO::Request& request, dobot_v4_bringup::ToolDO::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolDORequest2String(request), response.res);
}

bool CRRobot::toolDOInstant(dobot_v4_bringup::ToolDOInstant::Request& request,
                            dobot_v4_bringup::ToolDOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parsertoolDOInstantRequest2String(request), response.res);
}

bool CRRobot::AO(dobot_v4_bringup::AO::Request& request, dobot_v4_bringup::AO::Response& response)
{
    return commander_->callRosService(parseTool::parserAORequest2String(request), response.res);
}

bool CRRobot::AOInstant(dobot_v4_bringup::AOInstant::Request& request, dobot_v4_bringup::AOInstant::Response& response)
{
    return commander_->callRosService(parseTool::parserAOInstantRequest2String(request), response.res);
}

bool CRRobot::accJ(dobot_v4_bringup::AccJ::Request& request, dobot_v4_bringup::AccJ::Response& response)
{
    return commander_->callRosService(parseTool::parseraccJRequest2String(request), response.res);
}

bool CRRobot::accL(dobot_v4_bringup::AccL::Request& request, dobot_v4_bringup::AccL::Response& response)
{
    return commander_->callRosService(parseTool::parseraccLRequest2String(request), response.res);
}

bool CRRobot::velJ(dobot_v4_bringup::VelJ::Request& request, dobot_v4_bringup::VelJ::Response& response)
{
    return commander_->callRosService(parseTool::parservelJRequest2String(request), response.res);
}

bool CRRobot::velL(dobot_v4_bringup::VelL::Request& request, dobot_v4_bringup::VelL::Response& response)
{
    return commander_->callRosService(parseTool::parservelLRequest2String(request), response.res);
}

bool CRRobot::cp(dobot_v4_bringup::CP::Request& request, dobot_v4_bringup::CP::Response& response)
{
    return commander_->callRosService(parseTool::parsercpRequest2String(request), response.res);
}

bool CRRobot::powerOn(dobot_v4_bringup::PowerOn::Request& request, dobot_v4_bringup::PowerOn::Response& response)
{
    return commander_->callRosService(parseTool::parserpowerOnRequest2String(request), response.res);
}

bool CRRobot::runScript(dobot_v4_bringup::RunScript::Request& request, dobot_v4_bringup::RunScript::Response& response)
{
    return commander_->callRosService(parseTool::parserrunScriptRequest2String(request), response.res);
}

bool CRRobot::stop(dobot_v4_bringup::Stop::Request& request, dobot_v4_bringup::Stop::Response& response)
{
    return commander_->callRosService(parseTool::parserstopRequest2String(request), response.res);
}

bool CRRobot::pause(dobot_v4_bringup::Pause::Request& request, dobot_v4_bringup::Pause::Response& response)
{
    return commander_->callRosService(parseTool::parserpauseRequest2String(request), response.res);
}

bool CRRobot::Continue(dobot_v4_bringup::Continue::Request& request, dobot_v4_bringup::Continue::Response& response)
{
    return commander_->callRosService(parseTool::parserContinueRequest2String(request), response.res);
}

bool CRRobot::setCollisionLevel(dobot_v4_bringup::SetCollisionLevel::Request& request,
                                dobot_v4_bringup::SetCollisionLevel::Response& response)
{
    return commander_->callRosService(parseTool::parsersetCollisionLevelRequest2String(request), response.res);
}

bool CRRobot::EnableSafeSkin(dobot_v4_bringup::EnableSafeSkin::Request& request,
                             dobot_v4_bringup::EnableSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserEnableSafeSkinRequest2String(request), response.res);
}

bool CRRobot::SetSafeSkin(dobot_v4_bringup::SetSafeSkin::Request& request,
                          dobot_v4_bringup::SetSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeSkinRequest2String(request), response.res);
}

bool CRRobot::GetStartPose(dobot_v4_bringup::GetStartPose::Request& request,
                           dobot_v4_bringup::GetStartPose::Response& response)
{
    return commander_->callRosService(parseTool::parserGetStartPoseRequest2String(request), response.res);
}

bool CRRobot::StartPath(dobot_v4_bringup::StartPath::Request& request, dobot_v4_bringup::StartPath::Response& response)
{
    return commander_->callRosService(parseTool::parserStartPathRequest2String(request), response.res);
}

bool CRRobot::PositiveKin(dobot_v4_bringup::PositiveKin::Request& request,
                          dobot_v4_bringup::PositiveKin::Response& response)
{
    return commander_->callRosService(parseTool::parserPositiveKinRequest2String(request), response.res);
}

bool CRRobot::InverseKin(dobot_v4_bringup::InverseKin::Request& request,
                         dobot_v4_bringup::InverseKin::Response& response)
{
    return commander_->callRosService(parseTool::parserInverseKinRequest2String(request), response.res);
}

bool CRRobot::GetAngle(dobot_v4_bringup::GetAngle::Request& request, dobot_v4_bringup::GetAngle::Response& response)
{
    return commander_->callRosService(parseTool::parserGetAngleRequest2String(request), response.res);
}

bool CRRobot::GetPose(dobot_v4_bringup::GetPose::Request& request, dobot_v4_bringup::GetPose::Response& response)
{
    return commander_->callRosService(parseTool::parserGetPoseRequest2String(request), response.res);
}

bool CRRobot::EmergencyStop(dobot_v4_bringup::EmergencyStop::Request& request,
                            dobot_v4_bringup::EmergencyStop::Response& response)
{
    return commander_->callRosService(parseTool::parserEmergencyStopRequest2String(request), response.res);
}

bool CRRobot::ModbusRTUCreate(dobot_v4_bringup::ModbusRTUCreate::Request& request,
                              dobot_v4_bringup::ModbusRTUCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusRTUCreateRequest2String(request), response.res);
}

bool CRRobot::ModbusCreate(dobot_v4_bringup::ModbusCreate::Request& request,
                           dobot_v4_bringup::ModbusCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCreateRequest2String(request), response.res);
}

bool CRRobot::ModbusClose(dobot_v4_bringup::ModbusClose::Request& request,
                          dobot_v4_bringup::ModbusClose::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCloseRequest2String(request), response.res);
}

bool CRRobot::GetInBits(dobot_v4_bringup::GetInBits::Request& request, dobot_v4_bringup::GetInBits::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInBitsRequest2String(request), response.res);
}

bool CRRobot::GetInRegs(dobot_v4_bringup::GetInRegs::Request& request, dobot_v4_bringup::GetInRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInRegsRequest2String(request), response.res);
}

bool CRRobot::GetCoils(dobot_v4_bringup::GetCoils::Request& request, dobot_v4_bringup::GetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCoilsRequest2String(request), response.res);
}

bool CRRobot::SetCoils(dobot_v4_bringup::SetCoils::Request& request, dobot_v4_bringup::SetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserSetCoilsRequest2String(request), response.res);
}

bool CRRobot::GetHoldRegs(dobot_v4_bringup::GetHoldRegs::Request& request,
                          dobot_v4_bringup::GetHoldRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetHoldRegsRequest2String(request), response.res);
}

bool CRRobot::SetHoldRegs(dobot_v4_bringup::SetHoldRegs::Request& request,
                          dobot_v4_bringup::SetHoldRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserSetHoldRegsRequest2String(request), response.res);
}

bool CRRobot::GetErrorID(dobot_v4_bringup::GetErrorID::Request& request,
                         dobot_v4_bringup::GetErrorID::Response& response)
{
    return getErrorID(request, response);
}

bool CRRobot::DI(dobot_v4_bringup::DI::Request& request, dobot_v4_bringup::DI::Response& response)
{
    return commander_->callRosService(parseTool::parserDIRequest2String(request), response.res);
}

bool CRRobot::ToolDI(dobot_v4_bringup::ToolDI::Request& request, dobot_v4_bringup::ToolDI::Response& response)
{
    return commander_->callRosService(parseTool::parserToolDIRequest2String(request), response.res);
}

bool CRRobot::AI(dobot_v4_bringup::AI::Request& request, dobot_v4_bringup::AI::Response& response)
{
    return commander_->callRosService(parseTool::parserAIRequest2String(request), response.res);
}

bool CRRobot::ToolAI(dobot_v4_bringup::ToolAI::Request& request, dobot_v4_bringup::ToolAI::Response& response)
{
    return commander_->callRosService(parseTool::parserToolAIRequest2String(request), response.res);
}

bool CRRobot::DIGroup(dobot_v4_bringup::DIGroup::Request& request, dobot_v4_bringup::DIGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserDIGroupRequest2String(request), response.res);
}

bool CRRobot::doGroup(dobot_v4_bringup::DOGroup::Request& request, dobot_v4_bringup::DOGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserdoGroupRequest2String(request), response.res);
}

bool CRRobot::brakeControl(dobot_v4_bringup::BrakeControl::Request& request,
                           dobot_v4_bringup::BrakeControl::Response& response)
{
    return commander_->callRosService(parseTool::parserbrakeControlRequest2String(request), response.res);
}

bool CRRobot::startDrag(dobot_v4_bringup::StartDrag::Request& request, dobot_v4_bringup::StartDrag::Response& response)
{
    return commander_->callRosService(parseTool::parserstartDragRequest2String(request), response.res);
}

bool CRRobot::StopDrag(dobot_v4_bringup::StopDrag::Request& request, dobot_v4_bringup::StopDrag::Response& response)
{
    return commander_->callRosService(parseTool::parserStopDragRequest2String(request), response.res);
}

bool CRRobot::DragSensivity(dobot_v4_bringup::DragSensivity::Request& request,
                            dobot_v4_bringup::DragSensivity::Response& response)
{
    return commander_->callRosService(parseTool::parserDragSensivityRequest2String(request), response.res);
}

bool CRRobot::GetDO(dobot_v4_bringup::GetDO::Request& request, dobot_v4_bringup::GetDO::Response& response)
{
    return commander_->callRosService(parseTool::parserGetDORequest2String(request), response.res);
}

bool CRRobot::GetAO(dobot_v4_bringup::GetAO::Request& request, dobot_v4_bringup::GetAO::Response& response)
{
    return commander_->callRosService(parseTool::parserGetAORequest2String(request), response.res);
}

bool CRRobot::GetDOGroup(dobot_v4_bringup::GetDOGroup::Request& request,
                         dobot_v4_bringup::GetDOGroup::Response& response)
{
    return commander_->callRosService(parseTool::parserGetDOGroupRequest2String(request), response.res);
}

bool CRRobot::SetTool485(dobot_v4_bringup::SetTool485::Request& request,
                         dobot_v4_bringup::SetTool485::Response& response)
{
    return commander_->callRosService(parseTool::parserSetTool485Request2String(request), response.res);
}

bool CRRobot::SetSafeWallEnable(dobot_v4_bringup::SetSafeWallEnable::Request& request,
                                dobot_v4_bringup::SetSafeWallEnable::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeWallEnableRequest2String(request), response.res);
}

bool CRRobot::SetToolPower(dobot_v4_bringup::SetToolPower::Request& request,
                           dobot_v4_bringup::SetToolPower::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolPowerRequest2String(request), response.res);
}

bool CRRobot::SetToolMode(dobot_v4_bringup::SetToolMode::Request& request,
                          dobot_v4_bringup::SetToolMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolModeRequest2String(request), response.res);
}

bool CRRobot::SetBackDistance(dobot_v4_bringup::SetBackDistance::Request& request,
                              dobot_v4_bringup::SetBackDistance::Response& response)
{
    return commander_->callRosService(parseTool::parserSetBackDistanceRequest2String(request), response.res);
}

bool CRRobot::SetPostCollisionMode(dobot_v4_bringup::SetPostCollisionMode::Request& request,
                                   dobot_v4_bringup::SetPostCollisionMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetPostCollisionModeRequest2String(request), response.res);
}

bool CRRobot::SetUser(dobot_v4_bringup::SetUser::Request& request, dobot_v4_bringup::SetUser::Response& response)
{
    return commander_->callRosService(parseTool::parserSetUserRequest2String(request), response.res);
}

bool CRRobot::SetTool(dobot_v4_bringup::SetTool::Request& request, dobot_v4_bringup::SetTool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolRequest2String(request), response.res);
}

bool CRRobot::CalcUser(dobot_v4_bringup::CalcUser::Request& request, dobot_v4_bringup::CalcUser::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcUserRequest2String(request), response.res);
}

bool CRRobot::CalcTool(dobot_v4_bringup::CalcTool::Request& request, dobot_v4_bringup::CalcTool::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcToolRequest2String(request), response.res);
}

bool CRRobot::GetInputBool(dobot_v4_bringup::GetInputBool::Request& request,
                           dobot_v4_bringup::GetInputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputboolRequest2String(request), response.res);
}

bool CRRobot::GetInputInt(dobot_v4_bringup::GetInputInt::Request& request,
                          dobot_v4_bringup::GetInputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputIntRequest2String(request), response.res);
}

bool CRRobot::GetInputFloat(dobot_v4_bringup::GetInputFloat::Request& request,
                            dobot_v4_bringup::GetInputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputFloatRequest2String(request), response.res);
}

bool CRRobot::GetOutputBool(dobot_v4_bringup::GetOutputBool::Request& request,
                            dobot_v4_bringup::GetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputboolRequest2String(request), response.res);
}

bool CRRobot::GetOutputInt(dobot_v4_bringup::GetOutputInt::Request& request,
                           dobot_v4_bringup::GetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputIntRequest2String(request), response.res);
}

bool CRRobot::GetOutputFloat(dobot_v4_bringup::GetOutputFloat::Request& request,
                             dobot_v4_bringup::GetOutputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputFloatRequest2String(request), response.res);
}

bool CRRobot::SetOutputBool(dobot_v4_bringup::SetOutputBool::Request& request,
                            dobot_v4_bringup::SetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputboolRequest2String(request), response.res);
}

bool CRRobot::SetOutputInt(dobot_v4_bringup::SetOutputInt::Request& request,
                           dobot_v4_bringup::SetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputIntRequest2String(request), response.res);
}

bool CRRobot::SetOutputFloat(dobot_v4_bringup::SetOutputFloat::Request& request,
                             dobot_v4_bringup::SetOutputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputFloatRequest2String(request), response.res);
}

bool CRRobot::movJ(dobot_v4_bringup::MovJ::Request& request, dobot_v4_bringup::MovJ::Response& response)
{
    return commander_->callRosService(parseTool::parsermovJRequest2String(request), response.res);
}

bool CRRobot::movL(dobot_v4_bringup::MovL::Request& request, dobot_v4_bringup::MovL::Response& response)
{
    return commander_->callRosService(parseTool::parsermovLRequest2String(request), response.res);
}

bool CRRobot::MovLIO(dobot_v4_bringup::MovLIO::Request& request, dobot_v4_bringup::MovLIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovLIORequest2String(request), response.res);
}

bool CRRobot::MovJIO(dobot_v4_bringup::MovJIO::Request& request, dobot_v4_bringup::MovJIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovJIORequest2String(request), response.res);
}

bool CRRobot::Arc(dobot_v4_bringup::Arc::Request& request, dobot_v4_bringup::Arc::Response& response)
{
    return commander_->callRosService(parseTool::parserArcRequest2String(request), response.res);
}

bool CRRobot::Circle(dobot_v4_bringup::Circle::Request& request, dobot_v4_bringup::Circle::Response& response)
{
    return commander_->callRosService(parseTool::parserCircleRequest2String(request), response.res);
}

bool CRRobot::moveJog(dobot_v4_bringup::MoveJog::Request& request, dobot_v4_bringup::MoveJog::Response& response)
{
    return commander_->callRosService(parseTool::parsermoveJogRequest2String(request), response.res);
}

bool CRRobot::stopmoveJog(dobot_v4_bringup::StopMoveJog::Request& request,
                          dobot_v4_bringup::StopMoveJog::Response& response)
{
    return commander_->callRosService(parseTool::parserStopmoveJogRequest2String(request), response.res);
}

bool CRRobot::RelMovJTool(dobot_v4_bringup::RelMovJTool::Request& request,
                          dobot_v4_bringup::RelMovJTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJToolRequest2String(request), response.res);
}

bool CRRobot::RelMovLTool(dobot_v4_bringup::RelMovLTool::Request& request,
                          dobot_v4_bringup::RelMovLTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLToolRequest2String(request), response.res);
}

bool CRRobot::RelMovJUser(dobot_v4_bringup::RelMovJUser::Request& request,
                          dobot_v4_bringup::RelMovJUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJUserRequest2String(request), response.res);
}

bool CRRobot::RelMovLUser(dobot_v4_bringup::RelMovLUser::Request& request,
                          dobot_v4_bringup::RelMovLUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLUserRequest2String(request), response.res);
}

bool CRRobot::relJointMovJ(dobot_v4_bringup::RelJointMovJ::Request& request,
                           dobot_v4_bringup::RelJointMovJ::Response& response)
{
    return commander_->callRosService(parseTool::parserrelJointMovJRequest2String(request), response.res);
}

bool CRRobot::GetCurrentCommandId(dobot_v4_bringup::GetCurrentCommandId::Request& request,
                                  dobot_v4_bringup::GetCurrentCommandId::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCurrentCommandIdRequest2String(request), response.res);
}

bool CRRobot::ServoJ(dobot_v4_bringup::ServoJ::Request& request, dobot_v4_bringup::ServoJ::Response& response)
{
    return commander_->callRosService(parseTool::parserServoJRequest2String(request), response.res);
}

bool CRRobot::ServoP(dobot_v4_bringup::ServoP::Request& request, dobot_v4_bringup::ServoP::Response& response)
{
    return commander_->callRosService(parseTool::parserServoPRequest2String(request), response.res);
}

bool CRRobot::tcpDashboard(dobot_v4_bringup::TcpDashboard::Request& request,
                           dobot_v4_bringup::TcpDashboard::Response& response)
{
    try {
        std::vector<std::string> result;
        int res;
        commander_->callRosService(request.command, res, result);
        if (!result.empty()) {
            for (int i = 0; i < result.size(); i++) {
                if (i == result.size()) {
                    response.result = response.result + result[i];
                    break;
                }
                response.result = response.result + result[i] + " ";
            }
        }
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CRRobot::getErrorID(dobot_v4_bringup::GetErrorID::Request& request,
                         dobot_v4_bringup::GetErrorID::Response& response)
{
    dobot_v4_bringup::TcpDashboard::Request req = {};
    dobot_v4_bringup::TcpDashboard::Response res = {};
    req.command = "GetErrorID()";
    if (tcpDashboard(req, res)) {
        std::stringstream ss(res.result);    // 创建一个字符串流对象，并将输入字符串传入
        int errorid;
        while (ss >> errorid) {    // 使用字符串流读取数字
            if (errorid != 0) {
                response.error_id.push_back(errorid);    // 将读取到的数字添加到容器中
            }
        }

        return true;
    }
    return false;
}

std::string CRRobot::parseString(const std::string& str)
{
    std::string returnInfomation = "ErrorID: ";

    std::size_t pos = str.find(',');
    if (pos == std::string::npos)
        throw std::logic_error(std::string("Has no ',' found : ") + str);
    returnInfomation += str.substr(0, pos);
    returnInfomation += " ReturnValue: ";

    // parse result
    std::size_t start_pos = str.find('{');
    if (start_pos == std::string::npos)
        throw std::logic_error(std::string("Has no '{': ") + str);
    std::size_t end_pos = str.find('}');
    if (end_pos == std::string::npos)
        throw std::logic_error(std::string("Has no '}': ") + str);

    assert(end_pos > start_pos);
    returnInfomation += str.substr(start_pos + 1, end_pos - start_pos - 1);
    return returnInfomation;
}

void CRRobot::Info(const std::string& info)
{
    ROS_INFO("%s", parseString(info).c_str());
}

void CRRobot::backendTask(const ros::TimerEvent& e)
{
    uint16_t robot_mode = commander_->getRobotMode();
    if (robot_mode == 9 && last_robot_mode_ != 9) {
        dobot_v4_bringup::GetErrorID::Request req = {};
        dobot_v4_bringup::GetErrorID::Response res = {};
        bool ok = getErrorID(req, res);
        if (ok) {
            for (const auto& i : res.error_id) {
                ROS_ERROR("Robot alarm, error id %d", i);
            }
        } else {
            ROS_ERROR("Robot alarm");
        }
    }
    last_robot_mode_ = robot_mode;
}
