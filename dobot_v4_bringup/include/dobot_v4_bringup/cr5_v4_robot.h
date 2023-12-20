/**
 ***********************************************************************************************************************
 *
 * @author YangXiBo
 * @date   2023/08/18
 *
 *
 ***********************************************************************************************************************
 */

#include <string>
#include <memory>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <dobot_v4_bringup/commander.h>
#include <dobot_v4_bringup/parseTool.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <dobot_v4_bringup/EnableRobot.h>
#include <dobot_v4_bringup/DisableRobot.h>
#include <dobot_v4_bringup/ClearError.h>
#include <dobot_v4_bringup/SpeedFactor.h>
#include <dobot_v4_bringup/User.h>
#include <dobot_v4_bringup/Tool.h>
#include <dobot_v4_bringup/RobotMode.h>
#include <dobot_v4_bringup/SetPayload.h>
#include <dobot_v4_bringup/DO.h>
#include <dobot_v4_bringup/DOInstant.h>
#include <dobot_v4_bringup/ToolDO.h>
#include <dobot_v4_bringup/ToolDOInstant.h>
#include <dobot_v4_bringup/AO.h>
#include <dobot_v4_bringup/AOInstant.h>
#include <dobot_v4_bringup/AccJ.h>
#include <dobot_v4_bringup/AccL.h>
#include <dobot_v4_bringup/VelJ.h>
#include <dobot_v4_bringup/VelL.h>
#include <dobot_v4_bringup/CP.h>
#include <dobot_v4_bringup/PowerOn.h>
#include <dobot_v4_bringup/RunScript.h>
#include <dobot_v4_bringup/Stop.h>
#include <dobot_v4_bringup/Pause.h>
#include <dobot_v4_bringup/Continue.h>
#include <dobot_v4_bringup/SetCollisionLevel.h>
#include <dobot_v4_bringup/EnableSafeSkin.h>
#include <dobot_v4_bringup/SetSafeSkin.h>
#include <dobot_v4_bringup/StartPath.h>
#include <dobot_v4_bringup/GetStartPose.h>
#include <dobot_v4_bringup/PositiveKin.h>
#include <dobot_v4_bringup/GetPose.h>
#include <dobot_v4_bringup/InverseKin.h>
#include <dobot_v4_bringup/GetAngle.h>
#include <dobot_v4_bringup/EmergencyStop.h>
#include <dobot_v4_bringup/ModbusRTUCreate.h>
#include <dobot_v4_bringup/ModbusCreate.h>
#include <dobot_v4_bringup/ModbusClose.h>
#include <dobot_v4_bringup/GetInBits.h>
#include <dobot_v4_bringup/GetInRegs.h>
#include <dobot_v4_bringup/GetCoils.h>
#include <dobot_v4_bringup/SetCoils.h>
#include <dobot_v4_bringup/GetHoldRegs.h>
#include <dobot_v4_bringup/SetHoldRegs.h>
#include <dobot_v4_bringup/GetErrorID.h>
#include <dobot_v4_bringup/DI.h>
#include <dobot_v4_bringup/ToolDI.h>
#include <dobot_v4_bringup/AI.h>
#include <dobot_v4_bringup/ToolAI.h>
#include <dobot_v4_bringup/DIGroup.h>
#include <dobot_v4_bringup/DOGroup.h>
#include <dobot_v4_bringup/BrakeControl.h>
#include <dobot_v4_bringup/StartDrag.h>
#include <dobot_v4_bringup/StopDrag.h>
#include <dobot_v4_bringup/DragSensivity.h>
#include <dobot_v4_bringup/GetDO.h>
#include <dobot_v4_bringup/GetAO.h>
#include <dobot_v4_bringup/GetDOGroup.h>
#include <dobot_v4_bringup/SetTool485.h>
#include <dobot_v4_bringup/SetSafeWallEnable.h>
#include <dobot_v4_bringup/SetToolPower.h>
#include <dobot_v4_bringup/SetToolMode.h>
#include <dobot_v4_bringup/SetBackDistance.h>
#include <dobot_v4_bringup/SetPostCollisionMode.h>
#include <dobot_v4_bringup/SetUser.h>
#include <dobot_v4_bringup/SetTool.h>
#include <dobot_v4_bringup/CalcUser.h>
#include <dobot_v4_bringup/CalcTool.h>
#include <dobot_v4_bringup/GetInputBool.h>
#include <dobot_v4_bringup/GetInputInt.h>
#include <dobot_v4_bringup/GetInputFloat.h>
#include <dobot_v4_bringup/GetOutputBool.h>
#include <dobot_v4_bringup/GetOutputInt.h>
#include <dobot_v4_bringup/GetOutputFloat.h>
#include <dobot_v4_bringup/SetOutputBool.h>
#include <dobot_v4_bringup/SetOutputInt.h>
#include <dobot_v4_bringup/SetOutputFloat.h>
#include <dobot_v4_bringup/MovLIO.h>
#include <dobot_v4_bringup/MovJIO.h>
#include <dobot_v4_bringup/Arc.h>
#include <dobot_v4_bringup/Circle.h>
#include <dobot_v4_bringup/MoveJog.h>
#include <dobot_v4_bringup/StopMoveJog.h>
#include <dobot_v4_bringup/RelMovJTool.h>
#include <dobot_v4_bringup/RelMovLTool.h>
#include <dobot_v4_bringup/RelMovJUser.h>
#include <dobot_v4_bringup/RelMovLUser.h>
#include <dobot_v4_bringup/MovJ.h>
#include <dobot_v4_bringup/MovL.h>
#include <dobot_v4_bringup/RelJointMovJ.h>
#include <dobot_v4_bringup/RobotStatus.h>
#include <dobot_v4_bringup/GetCurrentCommandId.h>
#include <dobot_v4_bringup/ServoJ.h>
#include <dobot_v4_bringup/ServoP.h>
#include <dobot_v4_bringup/TcpDashboard.h>

using namespace actionlib;
using namespace control_msgs;

/**
 * CRRobot
 */
class CRRobot : protected ActionServer<FollowJointTrajectoryAction>
{
private:
    double goal_[6];
    uint32_t index_;
    ros::Timer timer_;
    ros::Timer movj_timer_;
    ros::Timer backend_task_;
    double trajectory_duration_;
    ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<ros::ServiceServer> server_tbl_;
    std::stringstream arg_buffer;
    uint16_t last_robot_mode_;
    std::thread threadPubFeedBackInfo;
    ros::Publisher pubFeedInfo;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    CRRobot(ros::NodeHandle& nh, std::string name);

    /**
     * CRRobot
     */
    ~CRRobot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    /**
     * TCP
     */
    bool enableRobot(dobot_v4_bringup::EnableRobot::Request& request,
                     dobot_v4_bringup::EnableRobot::Response& response);
    bool disableRobot(dobot_v4_bringup::DisableRobot::Request& request,
                      dobot_v4_bringup::DisableRobot::Response& response);
    bool clearError(dobot_v4_bringup::ClearError::Request& request, dobot_v4_bringup::ClearError::Response& response);
    bool speedFactor(dobot_v4_bringup::SpeedFactor::Request& request,
                     dobot_v4_bringup::SpeedFactor::Response& response);
    bool user(dobot_v4_bringup::User::Request& request, dobot_v4_bringup::User::Response& response);
    bool tool(dobot_v4_bringup::Tool::Request& request, dobot_v4_bringup::Tool::Response& response);
    bool robotMode(dobot_v4_bringup::RobotMode::Request& request, dobot_v4_bringup::RobotMode::Response& response);
    bool setPayload(dobot_v4_bringup::SetPayload::Request& request, dobot_v4_bringup::SetPayload::Response& response);
    bool DO(dobot_v4_bringup::DO::Request& request, dobot_v4_bringup::DO::Response& response);
    bool DOInstant(dobot_v4_bringup::DOInstant::Request& request, dobot_v4_bringup::DOInstant::Response& response);
    bool toolDO(dobot_v4_bringup::ToolDO::Request& request, dobot_v4_bringup::ToolDO::Response& response);
    bool toolDOInstant(dobot_v4_bringup::ToolDOInstant::Request& request,
                       dobot_v4_bringup::ToolDOInstant::Response& response);
    bool AO(dobot_v4_bringup::AO::Request& request, dobot_v4_bringup::AO::Response& response);
    bool AOInstant(dobot_v4_bringup::AOInstant::Request& request, dobot_v4_bringup::AOInstant::Response& response);
    bool accJ(dobot_v4_bringup::AccJ::Request& request, dobot_v4_bringup::AccJ::Response& response);
    bool accL(dobot_v4_bringup::AccL::Request& request, dobot_v4_bringup::AccL::Response& response);
    bool velJ(dobot_v4_bringup::VelJ::Request& request, dobot_v4_bringup::VelJ::Response& response);
    bool velL(dobot_v4_bringup::VelL::Request& request, dobot_v4_bringup::VelL::Response& response);
    bool cp(dobot_v4_bringup::CP::Request& request, dobot_v4_bringup::CP::Response& response);
    bool powerOn(dobot_v4_bringup::PowerOn::Request& request, dobot_v4_bringup::PowerOn::Response& response);
    bool runScript(dobot_v4_bringup::RunScript::Request& request, dobot_v4_bringup::RunScript::Response& response);
    bool stop(dobot_v4_bringup::Stop::Request& request, dobot_v4_bringup::Stop::Response& response);
    bool pause(dobot_v4_bringup::Pause::Request& request, dobot_v4_bringup::Pause::Response& response);
    bool Continue(dobot_v4_bringup::Continue::Request& request, dobot_v4_bringup::Continue::Response& response);
    bool EnableSafeSkin(dobot_v4_bringup::EnableSafeSkin::Request& request,
                        dobot_v4_bringup::EnableSafeSkin::Response& response);
    bool SetSafeSkin(dobot_v4_bringup::SetSafeSkin::Request& request,
                     dobot_v4_bringup::SetSafeSkin::Response& response);
    bool GetStartPose(dobot_v4_bringup::GetStartPose::Request& request,
                      dobot_v4_bringup::GetStartPose::Response& response);
    bool StartPath(dobot_v4_bringup::StartPath::Request& request, dobot_v4_bringup::StartPath::Response& response);
    bool PositiveKin(dobot_v4_bringup::PositiveKin::Request& request,
                     dobot_v4_bringup::PositiveKin::Response& response);
    bool InverseKin(dobot_v4_bringup::InverseKin::Request& request, dobot_v4_bringup::InverseKin::Response& response);
    bool GetAngle(dobot_v4_bringup::GetAngle::Request& request, dobot_v4_bringup::GetAngle::Response& response);
    bool GetPose(dobot_v4_bringup::GetPose::Request& request, dobot_v4_bringup::GetPose::Response& response);
    bool EmergencyStop(dobot_v4_bringup::EmergencyStop::Request& request,
                       dobot_v4_bringup::EmergencyStop::Response& response);
    bool setCollisionLevel(dobot_v4_bringup::SetCollisionLevel::Request& request,
                           dobot_v4_bringup::SetCollisionLevel::Response& response);
    bool ModbusRTUCreate(dobot_v4_bringup::ModbusRTUCreate::Request& request,
                         dobot_v4_bringup::ModbusRTUCreate::Response& response);
    bool ModbusCreate(dobot_v4_bringup::ModbusCreate::Request& request,
                      dobot_v4_bringup::ModbusCreate::Response& response);
    bool ModbusClose(dobot_v4_bringup::ModbusClose::Request& request,
                     dobot_v4_bringup::ModbusClose::Response& response);
    bool GetInBits(dobot_v4_bringup::GetInBits::Request& request, dobot_v4_bringup::GetInBits::Response& response);
    bool GetInRegs(dobot_v4_bringup::GetInRegs::Request& request, dobot_v4_bringup::GetInRegs::Response& response);
    bool GetCoils(dobot_v4_bringup::GetCoils::Request& request, dobot_v4_bringup::GetCoils::Response& response);
    bool SetCoils(dobot_v4_bringup::SetCoils::Request& request, dobot_v4_bringup::SetCoils::Response& response);
    bool GetHoldRegs(dobot_v4_bringup::GetHoldRegs::Request& request,
                     dobot_v4_bringup::GetHoldRegs::Response& response);
    bool SetHoldRegs(dobot_v4_bringup::SetHoldRegs::Request& request,
                     dobot_v4_bringup::SetHoldRegs::Response& response);
    bool GetErrorID(dobot_v4_bringup::GetErrorID::Request& request, dobot_v4_bringup::GetErrorID::Response& response);
    bool DI(dobot_v4_bringup::DI::Request& request, dobot_v4_bringup::DI::Response& response);
    bool ToolDI(dobot_v4_bringup::ToolDI::Request& request, dobot_v4_bringup::ToolDI::Response& response);
    bool AI(dobot_v4_bringup::AI::Request& request, dobot_v4_bringup::AI::Response& response);
    bool ToolAI(dobot_v4_bringup::ToolAI::Request& request, dobot_v4_bringup::ToolAI::Response& response);
    bool DIGroup(dobot_v4_bringup::DIGroup::Request& request, dobot_v4_bringup::DIGroup::Response& response);
    bool doGroup(dobot_v4_bringup::DOGroup::Request& request, dobot_v4_bringup::DOGroup::Response& response);
    bool brakeControl(dobot_v4_bringup::BrakeControl::Request& request,
                      dobot_v4_bringup::BrakeControl::Response& response);
    bool startDrag(dobot_v4_bringup::StartDrag::Request& request, dobot_v4_bringup::StartDrag::Response& response);
    bool StopDrag(dobot_v4_bringup::StopDrag::Request& request, dobot_v4_bringup::StopDrag::Response& response);
    bool DragSensivity(dobot_v4_bringup::DragSensivity::Request& request,
                       dobot_v4_bringup::DragSensivity::Response& response);
    bool GetDO(dobot_v4_bringup::GetDO::Request& request, dobot_v4_bringup::GetDO::Response& response);
    bool GetAO(dobot_v4_bringup::GetAO::Request& request, dobot_v4_bringup::GetAO::Response& response);
    bool GetDOGroup(dobot_v4_bringup::GetDOGroup::Request& request, dobot_v4_bringup::GetDOGroup::Response& response);
    bool SetTool485(dobot_v4_bringup::SetTool485::Request& request, dobot_v4_bringup::SetTool485::Response& response);
    bool SetSafeWallEnable(dobot_v4_bringup::SetSafeWallEnable::Request& request,
                           dobot_v4_bringup::SetSafeWallEnable::Response& response);
    bool SetToolPower(dobot_v4_bringup::SetToolPower::Request& request,
                      dobot_v4_bringup::SetToolPower::Response& response);
    bool SetToolMode(dobot_v4_bringup::SetToolMode::Request& request,
                     dobot_v4_bringup::SetToolMode::Response& response);
    bool SetBackDistance(dobot_v4_bringup::SetBackDistance::Request& request,
                         dobot_v4_bringup::SetBackDistance::Response& response);
    bool SetPostCollisionMode(dobot_v4_bringup::SetPostCollisionMode::Request& request,
                              dobot_v4_bringup::SetPostCollisionMode::Response& response);
    bool SetUser(dobot_v4_bringup::SetUser::Request& request, dobot_v4_bringup::SetUser::Response& response);
    bool SetTool(dobot_v4_bringup::SetTool::Request& request, dobot_v4_bringup::SetTool::Response& response);
    bool CalcUser(dobot_v4_bringup::CalcUser::Request& request, dobot_v4_bringup::CalcUser::Response& response);
    bool CalcTool(dobot_v4_bringup::CalcTool::Request& request, dobot_v4_bringup::CalcTool::Response& response);
    bool GetInputBool(dobot_v4_bringup::GetInputBool::Request& request,
                      dobot_v4_bringup::GetInputBool::Response& response);
    bool GetInputInt(dobot_v4_bringup::GetInputInt::Request& request,
                     dobot_v4_bringup::GetInputInt::Response& response);
    bool GetInputFloat(dobot_v4_bringup::GetInputFloat::Request& request,
                       dobot_v4_bringup::GetInputFloat::Response& response);
    bool GetOutputBool(dobot_v4_bringup::GetOutputBool::Request& request,
                       dobot_v4_bringup::GetOutputBool::Response& response);
    bool GetOutputInt(dobot_v4_bringup::GetOutputInt::Request& request,
                      dobot_v4_bringup::GetOutputInt::Response& response);
    bool GetOutputFloat(dobot_v4_bringup::GetOutputFloat::Request& request,
                        dobot_v4_bringup::GetOutputFloat::Response& response);
    bool SetOutputBool(dobot_v4_bringup::SetOutputBool::Request& request,
                       dobot_v4_bringup::SetOutputBool::Response& response);
    bool SetOutputInt(dobot_v4_bringup::SetOutputInt::Request& request,
                      dobot_v4_bringup::SetOutputInt::Response& response);
    bool SetOutputFloat(dobot_v4_bringup::SetOutputFloat::Request& request,
                        dobot_v4_bringup::SetOutputFloat::Response& response);
    bool movJ(dobot_v4_bringup::MovJ::Request& request, dobot_v4_bringup::MovJ::Response& response);
    bool movL(dobot_v4_bringup::MovL::Request& request, dobot_v4_bringup::MovL::Response& response);
    bool MovLIO(dobot_v4_bringup::MovLIO::Request& request, dobot_v4_bringup::MovLIO::Response& response);
    bool MovJIO(dobot_v4_bringup::MovJIO::Request& request, dobot_v4_bringup::MovJIO::Response& response);
    bool Arc(dobot_v4_bringup::Arc::Request& request, dobot_v4_bringup::Arc::Response& response);
    bool Circle(dobot_v4_bringup::Circle::Request& request, dobot_v4_bringup::Circle::Response& response);
    bool moveJog(dobot_v4_bringup::MoveJog::Request& request, dobot_v4_bringup::MoveJog::Response& response);
    bool stopmoveJog(dobot_v4_bringup::StopMoveJog::Request& request,
                     dobot_v4_bringup::StopMoveJog::Response& response);
    bool RelMovJTool(dobot_v4_bringup::RelMovJTool::Request& request,
                     dobot_v4_bringup::RelMovJTool::Response& response);
    bool RelMovLTool(dobot_v4_bringup::RelMovLTool::Request& request,
                     dobot_v4_bringup::RelMovLTool::Response& response);
    bool RelMovJUser(dobot_v4_bringup::RelMovJUser::Request& request,
                     dobot_v4_bringup::RelMovJUser::Response& response);
    bool RelMovLUser(dobot_v4_bringup::RelMovLUser::Request& request,
                     dobot_v4_bringup::RelMovLUser::Response& response);
    bool relJointMovJ(dobot_v4_bringup::RelJointMovJ::Request& request,
                      dobot_v4_bringup::RelJointMovJ::Response& response);
    bool GetCurrentCommandId(dobot_v4_bringup::GetCurrentCommandId::Request& request,
                             dobot_v4_bringup::GetCurrentCommandId::Response& response);
    bool ServoJ(dobot_v4_bringup::ServoJ::Request& request, dobot_v4_bringup::ServoJ::Response& response);
    bool ServoP(dobot_v4_bringup::ServoP::Request& request, dobot_v4_bringup::ServoP::Response& response);
    bool getErrorID(dobot_v4_bringup::GetErrorID::Request& request, dobot_v4_bringup::GetErrorID::Response& response);
    bool tcpDashboard(dobot_v4_bringup::TcpDashboard::Request& request,
                      dobot_v4_bringup::TcpDashboard::Response& response);
    std::vector<double> sample_traj(const trajectory_msgs::JointTrajectoryPoint& P0,
                                    const trajectory_msgs::JointTrajectoryPoint& P1, const double& time_index);

private:
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    std::string parseString(const std::string& str);
    inline void Info(const std::string&);
    void backendTask(const ros::TimerEvent& e);
    void pubFeedBackInfo();
};
