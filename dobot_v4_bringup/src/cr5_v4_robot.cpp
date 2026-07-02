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

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_v4_bringup/cr5_v4_robot.h>
#include <dobot_v4_bringup/parseTool.h>

// 静态常量定义
const double CRRobot::SERVOJ_DURATION = 0.03;  // 33Hz插补频率

CRRobot::CRRobot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
    , stop_requested_(false)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

CRRobot::~CRRobot()
{
    ROS_INFO("~CRRobot");
    backend_task_.stop();
    stop_thread_ = true;
    if (threadPubFeedBackInfo.joinable()) {
        threadPubFeedBackInfo.join();
    }
}

void CRRobot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");
    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.02);  // 50Hz
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
    std::string serviceStartPath = serviceProjectName + serviceRobotName + "srv/StartPath";
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
    std::string serviceDOGroup = serviceProjectName + serviceRobotName + "srv/DOGroup";
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
    server_tbl_.push_back(control_nh_.advertiseService(serviceEnableSafeSkin, &CRRobot::enableSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeSkin, &CRRobot::setSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetStartPose, &CRRobot::getStartPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartPath, &CRRobot::startPath, this));
    server_tbl_.push_back(control_nh_.advertiseService(servicePositiveKin, &CRRobot::positiveKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceInverseKin, &CRRobot::inverseKin, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAngle, &CRRobot::getAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetPose, &CRRobot::getPose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCollisionLevel, &CRRobot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceEmergencyStop, &CRRobot::emergencyStop, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusRTUCreate, &CRRobot::modbusRTUCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusCreate, &CRRobot::modbusCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceModbusClose, &CRRobot::modbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInBits, &CRRobot::getInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInRegs, &CRRobot::getInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetCoils, &CRRobot::getCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetCoils, &CRRobot::setCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetHoldRegs, &CRRobot::getHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetHoldRegs, &CRRobot::setHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetErrorID, &CRRobot::getErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDI, &CRRobot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolDI, &CRRobot::toolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceAI, &CRRobot::AI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceToolAI, &CRRobot::toolAI, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDIGroup, &CRRobot::DIGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDOGroup, &CRRobot::doGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceBrakeControl, &CRRobot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStartDrag, &CRRobot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopDrag, &CRRobot::stopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceDragSensivity, &CRRobot::dragSensivity, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDO, &CRRobot::getDO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetAO, &CRRobot::getAO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetDOGroup, &CRRobot::getDOGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool485, &CRRobot::setTool485, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetSafeWallEnable, &CRRobot::setSafeWallEnable, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolPower, &CRRobot::setToolPower, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetToolMode, &CRRobot::setToolMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetBackDistance, &CRRobot::setBackDistance, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceSetPostCollisionMode, &CRRobot::setPostCollisionMode, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetUser, &CRRobot::setUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetTool, &CRRobot::setTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcUser, &CRRobot::calcUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCalcTool, &CRRobot::calcTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputBool, &CRRobot::getInputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputInt, &CRRobot::getInputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetInputFloat, &CRRobot::getInputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputBool, &CRRobot::getOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputInt, &CRRobot::getOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceGetOutputFloat, &CRRobot::getOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputBool, &CRRobot::setOutputBool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputInt, &CRRobot::setOutputInt, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceSetOutputFloat, &CRRobot::setOutputFloat, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJ, &CRRobot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovL, &CRRobot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovLIO, &CRRobot::movLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMovJIO, &CRRobot::movJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceArc, &CRRobot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceCircle, &CRRobot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceMoveJog, &CRRobot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceStopMoveJog, &CRRobot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJTool, &CRRobot::relMovJTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLTool, &CRRobot::relMovLTool, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovJUser, &CRRobot::relMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelMovLUser, &CRRobot::relMovLUser, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceRelJointMovJ, &CRRobot::relJointMovJ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService(serviceGetCurrentCommandId, &CRRobot::getCurrentCommandId, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoJ, &CRRobot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService(serviceServoP, &CRRobot::servoP, this));

    registerGoalCallback(boost::bind(&CRRobot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CRRobot::cancelHandle, this, _1));
    backend_task_ = control_nh_.createTimer(ros::Duration(1.5), &CRRobot::backendTask, this);

    pubFeedInfo = control_nh_.advertise<std_msgs::String>(topicFeedInfo, 1000);
    threadPubFeedBackInfo = std::thread(&CRRobot::pubFeedBackInfo, this);
    start();
}

void CRRobot::pubFeedBackInfo()
{
    RealTimeData realTimeData;
    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok() && !stop_thread_) {
        commander_->getRealData(realTimeData);
        nlohmann::json root;
        root["len"] = realTimeData.len;
        root["digital_input_bits"] = realTimeData.digital_input_bits;
        root["digital_outputs"] = realTimeData.digital_outputs;
        root["robot_mode"] = realTimeData.robot_mode;
        root["controller_timer"] = realTimeData.controller_timer;
        root["run_time"] = realTimeData.run_time;
        root["test_value"] = realTimeData.test_value;
        root["safety_mode"] = realTimeData.safety_mode;
        root["speed_scaling"] = realTimeData.speed_scaling;
        root["linear_momentum_norm"] = realTimeData.linear_momentum_norm;
        root["v_main"] = realTimeData.v_main;
        root["v_robot"] = realTimeData.v_robot;
        root["i_robot"] = realTimeData.i_robot;
        root["program_state"] = realTimeData.program_state;
        root["safety_status"] = realTimeData.safety_status;

        std::vector<double> vecTransit;    // vector 中转存取数组类型
        for (int i = 0; i < 3; i++) {
            vecTransit.push_back(realTimeData.tool_accelerometer_values[i]);
        }
        root["tool_accelerometer_values"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 3; i++) {
            vecTransit.push_back(realTimeData.elbow_position[i]);
        }
        root["elbow_position"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 3; i++) {
            vecTransit.push_back(realTimeData.elbow_velocity[i]);
        }
        root["elbow_velocity"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.q_target[i]);
        }
        root["q_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.qd_target[i]);
        }
        root["qd_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.qdd_target[i]);
        }
        root["qdd_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.i_target[i]);
        }
        root["i_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.m_target[i]);
        }
        root["m_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.q_actual[i]);
        }
        root["q_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.qd_actual[i]);
        }
        root["qd_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.i_actual[i]);
        }
        root["i_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.i_control[i]);
        }
        root["i_control"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.tool_vector_actual[i]);
        }
        root["tool_vector_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.TCP_speed_actual[i]);
        }
        root["TCP_speed_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.TCP_force[i]);
        }
        root["TCP_force"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.Tool_vector_target[i]);
        }
        root["tool_vector_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.TCP_speed_target[i]);
        }
        root["TCP_speed_target"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.motor_temperatures[i]);
        }
        root["motor_temperatures"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.joint_modes[i]);
        }
        root["joint_modes"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.v_actual[i]);
        }
        root["v_actual"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData.handtype[i]);
        }
        root["handtype"] = vecTransit;
        vecTransit.clear();

        root["userCoordinate"] = realTimeData.userCoordinate;
        root["toolCoordinate"] = realTimeData.toolCoordinate;
        root["isRunQueuedCmd"] = realTimeData.isRunQueuedCmd;
        root["isPauseCmdFlag"] = realTimeData.isPauseCmdFlag;
        root["velocityRatio"] = realTimeData.velocityRatio;
        root["accelerationRatio"] = realTimeData.accelerationRatio;
        root["jerkRatio"] = realTimeData.jerkRatio;
        root["xyzVelocityRatio"] = realTimeData.xyzVelocityRatio;
        root["rVelocityRatio"] = realTimeData.rVelocityRatio;
        root["xyzAccelerationRatio"] = realTimeData.xyzAccelerationRatio;
        root["rAccelerationRatio"] = realTimeData.rAccelerationRatio;
        root["xyzJerkRatio"] = realTimeData.xyzJerkRatio;
        root["rJerkRatio"] = realTimeData.rJerkRatio;
        root["BrakeStatus"] = realTimeData.BrakeStatus;
        root["EnableStatus"] = realTimeData.EnableStatus;
        root["DragStatus"] = realTimeData.DragStatus;
        root["RunningStatus"] = realTimeData.RunningStatus;
        root["ErrorStatus"] = realTimeData.ErrorStatus;
        root["JogStatus"] = realTimeData.JogStatus;
        root["RobotType"] = realTimeData.RobotType;
        root["DragButtonSignal"] = realTimeData.DragButtonSignal;
        root["EnableButtonSignal"] = realTimeData.EnableButtonSignal;
        root["RecordButtonSignal"] = realTimeData.RecordButtonSignal;
        root["ReappearButtonSignal"] = realTimeData.ReappearButtonSignal;
        root["JawButtonSignal"] = realTimeData.JawButtonSignal;
        root["SixForceOnline"] = realTimeData.SixForceOnline;
        root["CollisionStates"] = realTimeData.CollisionStates;
        root["ArmApproachState"] = realTimeData.ArmApproachState;
        root["J4ApproachState"] = realTimeData.J4ApproachState;
        root["J5ApproachState"] = realTimeData.J5ApproachState;
        root["J6ApproachState"] = realTimeData.J6ApproachState;
        root["vibrationDisZ"] = realTimeData.vibrationDisZ;
        root["currentCommandId"] = realTimeData.currentCommandId;

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.m_actual[i]);
        }
        root["m_actual"] = vecTransit;
        vecTransit.clear();

        root["load"] = realTimeData.load;
        root["centerX"] = realTimeData.centerX;
        root["centerY"] = realTimeData.centerY;
        root["centerZ"] = realTimeData.centerZ;

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.user[i]);
        }
        root["user"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.tool[i]);
        }
        root["tool"] = vecTransit;
        vecTransit.clear();

        root["TraceIndex"] = realTimeData.TraceIndex;    // 1296 ~ 1303 轨迹复现索引 （未实现）

        for (int i = 0; i < 6; i++) {
            vecTransit.push_back(realTimeData.SixForceValue[i]);
        }
        root["SixForceValue"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData.TargetQuaternion[i]);
        }
        root["TargetQuaternion"] = vecTransit;
        vecTransit.clear();

        for (int i = 0; i < 4; i++) {
            vecTransit.push_back(realTimeData.ActualQuaternion[i]);
        }
        root["ActualQuaternion"] = vecTransit;
        vecTransit.clear();
        root["AutoManualMode"] = realTimeData.AutoManualMode;    // 1416 ~ 1417 手自动模式 0: 未开启 1: manual 2:auto

        std::string feedBackVecStr = root.dump();

        std_msgs::String msgFeedInfo;
        msgFeedInfo.data = feedBackVecStr;
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
    
    // 防止除以零：如果区间时间为零，返回起点
    if (T <= 0.0) {
        for (int i = 0; i < P0.positions.size(); i++) {
            interp_traj.push_back(P0.positions[i] * 180.0 / M_PI);
        }
        return interp_traj;
    }
    
    // 时间索引保护：确保 t 在 [0, T] 范围内
    if (t < 0.0) {
        t = 0.0;
    } else if (t > T) {
        t = T;
    }
    
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
    size_t num_points = goal->trajectory.points.size();
    
    // 检查是否收到停止请求
    if (stop_requested_) {
        ROS_INFO("Trajectory execution stopped");
        timer_.stop();
        movj_timer_.stop();
        handle.setAborted();
        stop_requested_ = false;
        return;
    }
    
    // 检查是否所有点已下发（index_从1开始，发完最后一个点后index_==num_points）
    if (index_ >= num_points) {
        // 首次进入补时阶段，记录起始时间
        if (completion_check_start_.isZero()) {
            completion_check_start_ = ros::Time::now();
        }
        
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        bool reached = true;
        for (int i = 0; i < 6; i++) {
            if (fabs(current_joints[i] - goal_[i]) > OFFSET_VAL) {
                reached = false;
                break;
            }
        }
        if (reached) {
            ROS_INFO("Trajectory execution completed");
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
            completion_check_start_ = ros::Time(0);  // 重置
        } else {
            // 断线检测：直接 abort，不等超时
            if (!isConnected()) {
                ROS_WARN("Robot disconnected during trajectory completion, aborting");
                timer_.stop();
                movj_timer_.stop();
                handle.setAborted();
                completion_check_start_ = ros::Time(0);
                return;
            }
            // 超时检测：最后一段 ServoJ 的 t + 5s 兜底
            // 最后一段 t 就是机器人实际需要的时间，不等整条轨迹时长
            double last_segment_t;
            if (num_points >= 2) {
                last_segment_t = goal->trajectory.points[num_points - 1].time_from_start.toSec() -
                                 goal->trajectory.points[num_points - 2].time_from_start.toSec();
            } else {
                last_segment_t = trajectory_duration_;  // 单点轨迹用 trajectory_duration_
            }
            last_segment_t = std::max(0.05, last_segment_t);  // 最小 50ms 对齐 dt 裁剪
            double timeout = last_segment_t + 5.0;
            double elapsed = (ros::Time::now() - completion_check_start_).toSec();
            if (elapsed > timeout) {
                ROS_WARN("Trajectory completion timeout after %.1fs (segment_t=%.1fs)", elapsed, last_segment_t);
                timer_.stop();
                movj_timer_.stop();
                handle.setAborted();
                completion_check_start_ = ros::Time(0);  // 重置
            }
        }
        return;
    }
    
    // 记录轨迹开始时间
    if (trajectory_start_.isZero()) {
        trajectory_start_ = ros::Time::now();
        ROS_INFO("Trajectory started at %.3fs", trajectory_start_.toSec());
    }
    
    // 仅有一个点：直接下发目标位置
    if (num_points <= 1) {
        char cmd[150];
        snprintf(cmd, sizeof(cmd), "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f,aheadtime=50.0)", 
                goal->trajectory.points[0].positions[0] * 180.0 / M_PI,
                goal->trajectory.points[0].positions[1] * 180.0 / M_PI,
                goal->trajectory.points[0].positions[2] * 180.0 / M_PI,
                goal->trajectory.points[0].positions[3] * 180.0 / M_PI,
                goal->trajectory.points[0].positions[4] * 180.0 / M_PI,
                goal->trajectory.points[0].positions[5] * 180.0 / M_PI,
                trajectory_duration_);
        int32_t err_id;
        commander_->motionDoCmd(cmd, err_id);
        index_ = num_points;
        return;
    }
    
    // index_ 从1开始，始终瞄准 points[index_]（下一个目标点）
    // 在 points[index_-1] 的预定时刻发 points[index_]，让机器人无缝连续运动
    ros::Time now = ros::Time::now();
    ros::Time send_time = trajectory_start_ + goal->trajectory.points[index_ - 1].time_from_start;
    if (now < send_time) {
        return;
    }
    
    // t = 相邻路点时间差（segment duration），跟随 MoveIt 规划的时间线
    // 注意：t 不能太小，否则控制器无法完成平滑规划导致抖动。
    // 参考 ROS2 版本，设置 t 范围为 [0.05, 3600.0]
    double t = goal->trajectory.points[index_].time_from_start.toSec() - 
               goal->trajectory.points[index_ - 1].time_from_start.toSec();
    t = std::max(0.05, std::min(t, 3600.0));  // 范围 [0.05, 3600]
    
    // 调试输出（每个新段开始输出一次）
    if (index_ <= 2 || index_ % 5 == 0) {
        ROS_INFO("Pt %u/%zu: target=pts[%u], time_from_start=%.3fs, t=%.4fs",
                 index_, num_points, index_,
                 goal->trajectory.points[index_].time_from_start.toSec(), t);
    }
    
    // 发送 ServoJ：目标=points[index_]，dobot controller 会将其与上一个 servoj 平滑衔接
    // aheadtime=50.0 是提前量（无单位，范围[20,100]），默认值，提供较好的平滑性
    char cmd[150];
    snprintf(cmd, sizeof(cmd), "servoj(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=%0.3f,aheadtime=50.0)", 
            goal->trajectory.points[index_].positions[0] * 180.0 / M_PI,
            goal->trajectory.points[index_].positions[1] * 180.0 / M_PI,
            goal->trajectory.points[index_].positions[2] * 180.0 / M_PI,
            goal->trajectory.points[index_].positions[3] * 180.0 / M_PI,
            goal->trajectory.points[index_].positions[4] * 180.0 / M_PI,
            goal->trajectory.points[index_].positions[5] * 180.0 / M_PI,
            t);
    int32_t err_id;
    commander_->motionDoCmd(cmd, err_id);
    
    index_++;
}

void CRRobot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 1;  // 从1开始，跳过P0（机器人已在P0），始终瞄准 next point
    trajectory_start_ = ros::Time(0);  // 重置轨迹开始时间
    completion_check_start_ = ros::Time(0);  // 重置完成检测起始时间
    
    // 输出轨迹点数量调试信息
    const auto& traj = handle.getGoal()->trajectory;
    ROS_INFO("Received trajectory with %zu points", traj.points.size());
    if (traj.points.size() < 5) {
        ROS_WARN("Warning: trajectory has only %zu points, may cause stuttering", traj.points.size());
        for (size_t i = 0; i < traj.points.size(); i++) {
            ROS_INFO("  Point %zu: time_from_start=%.3fs", i, traj.points[i].time_from_start.toSec());
        }
    }
    
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

bool CRRobot::enableSafeSkin(dobot_v4_bringup::EnableSafeSkin::Request& request,
                             dobot_v4_bringup::EnableSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserEnableSafeSkinRequest2String(request), response.res);
}

bool CRRobot::setSafeSkin(dobot_v4_bringup::SetSafeSkin::Request& request,
                          dobot_v4_bringup::SetSafeSkin::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeSkinRequest2String(request), response.res);
}

bool CRRobot::getStartPose(dobot_v4_bringup::GetStartPose::Request& request,
                           dobot_v4_bringup::GetStartPose::Response& response)
{
    return commander_->callRosService(parseTool::parserGetStartPoseRequest2String(request), response.res);
}

bool CRRobot::startPath(dobot_v4_bringup::StartPath::Request& request, dobot_v4_bringup::StartPath::Response& response)
{
    return commander_->callRosService(parseTool::parserStartPathRequest2String(request), response.res);
}

bool CRRobot::positiveKin(dobot_v4_bringup::PositiveKin::Request& request,
                          dobot_v4_bringup::PositiveKin::Response& response)
{
    return commander_->callRosService(parseTool::parserPositiveKinRequest2String(request), response.res);
}

bool CRRobot::inverseKin(dobot_v4_bringup::InverseKin::Request& request,
                         dobot_v4_bringup::InverseKin::Response& response)
{
    return commander_->callRosService(parseTool::parserInverseKinRequest2String(request), response.res);
}

bool CRRobot::getAngle(dobot_v4_bringup::GetAngle::Request& request, dobot_v4_bringup::GetAngle::Response& response)
{
    return commander_->callRosService_f(parseTool::parserGetAngleRequest2String(request), response.res, response.robot_return);
}

bool CRRobot::getPose(dobot_v4_bringup::GetPose::Request& request, dobot_v4_bringup::GetPose::Response& response)
{
    return commander_->callRosService_f(parseTool::parserGetPoseRequest2String(request), response.res, response.robot_return);
}

bool CRRobot::emergencyStop(dobot_v4_bringup::EmergencyStop::Request& request,
                            dobot_v4_bringup::EmergencyStop::Response& response)
{
    return commander_->callRosService(parseTool::parserEmergencyStopRequest2String(request), response.res);
}

bool CRRobot::modbusRTUCreate(dobot_v4_bringup::ModbusRTUCreate::Request& request,
                              dobot_v4_bringup::ModbusRTUCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusRTUCreateRequest2String(request), response.res);
}

bool CRRobot::modbusCreate(dobot_v4_bringup::ModbusCreate::Request& request,
                           dobot_v4_bringup::ModbusCreate::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCreateRequest2String(request), response.res);
}

bool CRRobot::modbusClose(dobot_v4_bringup::ModbusClose::Request& request,
                          dobot_v4_bringup::ModbusClose::Response& response)
{
    return commander_->callRosService(parseTool::parserModbusCloseRequest2String(request), response.res);
}

bool CRRobot::getInBits(dobot_v4_bringup::GetInBits::Request& request, dobot_v4_bringup::GetInBits::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInBitsRequest2String(request), response.res);
}

bool CRRobot::getInRegs(dobot_v4_bringup::GetInRegs::Request& request, dobot_v4_bringup::GetInRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInRegsRequest2String(request), response.res);
}

bool CRRobot::getCoils(dobot_v4_bringup::GetCoils::Request& request, dobot_v4_bringup::GetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCoilsRequest2String(request), response.res);
}

bool CRRobot::setCoils(dobot_v4_bringup::SetCoils::Request& request, dobot_v4_bringup::SetCoils::Response& response)
{
    return commander_->callRosService(parseTool::parserSetCoilsRequest2String(request), response.res);
}

bool CRRobot::getHoldRegs(dobot_v4_bringup::GetHoldRegs::Request& request,
                          dobot_v4_bringup::GetHoldRegs::Response& response)
{
    return commander_->callRosService(parseTool::parserGetHoldRegsRequest2String(request), response.res);
}

bool CRRobot::setHoldRegs(dobot_v4_bringup::SetHoldRegs::Request& request,
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

bool CRRobot::toolDI(dobot_v4_bringup::ToolDI::Request& request, dobot_v4_bringup::ToolDI::Response& response)
{
    return commander_->callRosService(parseTool::parserToolDIRequest2String(request), response.res);
}

bool CRRobot::AI(dobot_v4_bringup::AI::Request& request, dobot_v4_bringup::AI::Response& response)
{
    return commander_->callRosService(parseTool::parserAIRequest2String(request), response.res);
}

bool CRRobot::toolAI(dobot_v4_bringup::ToolAI::Request& request, dobot_v4_bringup::ToolAI::Response& response)
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

bool CRRobot::stopDrag(dobot_v4_bringup::StopDrag::Request& request, dobot_v4_bringup::StopDrag::Response& response)
{
    return commander_->callRosService(parseTool::parserStopDragRequest2String(request), response.res);
}

bool CRRobot::dragSensivity(dobot_v4_bringup::DragSensivity::Request& request,
                            dobot_v4_bringup::DragSensivity::Response& response)
{
    return commander_->callRosService(parseTool::parserDragSensivityRequest2String(request), response.res);
}

bool CRRobot::getDO(dobot_v4_bringup::GetDO::Request& request, dobot_v4_bringup::GetDO::Response& response)
{
    return commander_->callRosService_f(parseTool::parserGetDORequest2String(request), response.res, response.robot_return);
}

bool CRRobot::getAO(dobot_v4_bringup::GetAO::Request& request, dobot_v4_bringup::GetAO::Response& response)
{
    return commander_->callRosService_f(parseTool::parserGetAORequest2String(request), response.res, response.robot_return);
}

bool CRRobot::getDOGroup(dobot_v4_bringup::GetDOGroup::Request& request,
                         dobot_v4_bringup::GetDOGroup::Response& response)
{
    return commander_->callRosService_f(parseTool::parserGetDOGroupRequest2String(request), response.res, response.robot_return);
}

bool CRRobot::setTool485(dobot_v4_bringup::SetTool485::Request& request,
                         dobot_v4_bringup::SetTool485::Response& response)
{
    return commander_->callRosService(parseTool::parserSetTool485Request2String(request), response.res);
}

bool CRRobot::setSafeWallEnable(dobot_v4_bringup::SetSafeWallEnable::Request& request,
                                dobot_v4_bringup::SetSafeWallEnable::Response& response)
{
    return commander_->callRosService(parseTool::parserSetSafeWallEnableRequest2String(request), response.res);
}

bool CRRobot::setToolPower(dobot_v4_bringup::SetToolPower::Request& request,
                           dobot_v4_bringup::SetToolPower::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolPowerRequest2String(request), response.res);
}

bool CRRobot::setToolMode(dobot_v4_bringup::SetToolMode::Request& request,
                          dobot_v4_bringup::SetToolMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolModeRequest2String(request), response.res);
}

bool CRRobot::setBackDistance(dobot_v4_bringup::SetBackDistance::Request& request,
                              dobot_v4_bringup::SetBackDistance::Response& response)
{
    return commander_->callRosService(parseTool::parserSetBackDistanceRequest2String(request), response.res);
}

bool CRRobot::setPostCollisionMode(dobot_v4_bringup::SetPostCollisionMode::Request& request,
                                   dobot_v4_bringup::SetPostCollisionMode::Response& response)
{
    return commander_->callRosService(parseTool::parserSetPostCollisionModeRequest2String(request), response.res);
}

bool CRRobot::setUser(dobot_v4_bringup::SetUser::Request& request, dobot_v4_bringup::SetUser::Response& response)
{
    return commander_->callRosService(parseTool::parserSetUserRequest2String(request), response.res);
}

bool CRRobot::setTool(dobot_v4_bringup::SetTool::Request& request, dobot_v4_bringup::SetTool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetToolRequest2String(request), response.res);
}

bool CRRobot::calcUser(dobot_v4_bringup::CalcUser::Request& request, dobot_v4_bringup::CalcUser::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcUserRequest2String(request), response.res);
}

bool CRRobot::calcTool(dobot_v4_bringup::CalcTool::Request& request, dobot_v4_bringup::CalcTool::Response& response)
{
    return commander_->callRosService(parseTool::parserCalcToolRequest2String(request), response.res);
}

bool CRRobot::getInputBool(dobot_v4_bringup::GetInputBool::Request& request,
                           dobot_v4_bringup::GetInputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputboolRequest2String(request), response.res);
}

bool CRRobot::getInputInt(dobot_v4_bringup::GetInputInt::Request& request,
                          dobot_v4_bringup::GetInputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputIntRequest2String(request), response.res);
}

bool CRRobot::getInputFloat(dobot_v4_bringup::GetInputFloat::Request& request,
                            dobot_v4_bringup::GetInputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetInputFloatRequest2String(request), response.res);
}

bool CRRobot::getOutputBool(dobot_v4_bringup::GetOutputBool::Request& request,
                            dobot_v4_bringup::GetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputboolRequest2String(request), response.res);
}

bool CRRobot::getOutputInt(dobot_v4_bringup::GetOutputInt::Request& request,
                           dobot_v4_bringup::GetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputIntRequest2String(request), response.res);
}

bool CRRobot::getOutputFloat(dobot_v4_bringup::GetOutputFloat::Request& request,
                             dobot_v4_bringup::GetOutputFloat::Response& response)
{
    return commander_->callRosService(parseTool::parserGetOutputFloatRequest2String(request), response.res);
}

bool CRRobot::setOutputBool(dobot_v4_bringup::SetOutputBool::Request& request,
                            dobot_v4_bringup::SetOutputBool::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputboolRequest2String(request), response.res);
}

bool CRRobot::setOutputInt(dobot_v4_bringup::SetOutputInt::Request& request,
                           dobot_v4_bringup::SetOutputInt::Response& response)
{
    return commander_->callRosService(parseTool::parserSetOutputIntRequest2String(request), response.res);
}

bool CRRobot::setOutputFloat(dobot_v4_bringup::SetOutputFloat::Request& request,
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

bool CRRobot::movLIO(dobot_v4_bringup::MovLIO::Request& request, dobot_v4_bringup::MovLIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovLIORequest2String(request), response.res);
}

bool CRRobot::movJIO(dobot_v4_bringup::MovJIO::Request& request, dobot_v4_bringup::MovJIO::Response& response)
{
    return commander_->callRosService(parseTool::parserMovJIORequest2String(request), response.res);
}

bool CRRobot::arc(dobot_v4_bringup::Arc::Request& request, dobot_v4_bringup::Arc::Response& response)
{
    return commander_->callRosService(parseTool::parserArcRequest2String(request), response.res);
}

bool CRRobot::circle(dobot_v4_bringup::Circle::Request& request, dobot_v4_bringup::Circle::Response& response)
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

bool CRRobot::relMovJTool(dobot_v4_bringup::RelMovJTool::Request& request,
                          dobot_v4_bringup::RelMovJTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJToolRequest2String(request), response.res);
}

bool CRRobot::relMovLTool(dobot_v4_bringup::RelMovLTool::Request& request,
                          dobot_v4_bringup::RelMovLTool::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLToolRequest2String(request), response.res);
}

bool CRRobot::relMovJUser(dobot_v4_bringup::RelMovJUser::Request& request,
                          dobot_v4_bringup::RelMovJUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovJUserRequest2String(request), response.res);
}

bool CRRobot::relMovLUser(dobot_v4_bringup::RelMovLUser::Request& request,
                          dobot_v4_bringup::RelMovLUser::Response& response)
{
    return commander_->callRosService(parseTool::parserRelMovLUserRequest2String(request), response.res);
}

bool CRRobot::relJointMovJ(dobot_v4_bringup::RelJointMovJ::Request& request,
                           dobot_v4_bringup::RelJointMovJ::Response& response)
{
    return commander_->callRosService(parseTool::parserrelJointMovJRequest2String(request), response.res);
}

bool CRRobot::getCurrentCommandId(dobot_v4_bringup::GetCurrentCommandId::Request& request,
                                  dobot_v4_bringup::GetCurrentCommandId::Response& response)
{
    return commander_->callRosService(parseTool::parserGetCurrentCommandIdRequest2String(request), response.res);
}

bool CRRobot::servoJ(dobot_v4_bringup::ServoJ::Request& request, dobot_v4_bringup::ServoJ::Response& response)
{
    return commander_->callRosService(parseTool::parserServoJRequest2String(request), response.res);
}

bool CRRobot::servoP(dobot_v4_bringup::ServoP::Request& request, dobot_v4_bringup::ServoP::Response& response)
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
        bool ok = GetErrorID(req, res);
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
