/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/09
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <regex>
#include <algorithm>
#include <dobot_bringup/tcp_socket.h>

#pragma pack(push, 1)
struct RealTimeData
{
    uint16_t len;                   // 0000 ~ 0001  字符长度
    uint16_t Reserve[3];            // 0002 ~ 0007  占位符
    uint64_t digital_input_bits;    // 0008 ~ 0015  DI
    uint64_t digital_outputs;       // 0016 ~ 0023  DO
    uint64_t robot_mode;            // 0024 ~ 0031  机器人模式
    uint64_t controller_timer;      // 0032 ~ 0039
    uint64_t run_time;              // 0040 ~ 0047
    uint64_t test_value;            // 0048 ~ 0055  内存结构测试标准值  0x0123456789ABCDEF
    double safety_mode;             // 0056 ~ 0063
    double speed_scaling;           // 0064 ~ 0071
    double linear_momentum_norm;    // 0072 ~ 0079
    double v_main;                  // 0080 ~ 0087
    double v_robot;                 // 0088 ~ 0095
    double i_robot;                         // 0096 ~ 0103
    double program_state;                   // 0104 ~ 0111
    double safety_status;                   // 0112 ~ 0119
    double tool_accelerometer_values[3];    // 0120 ~ 0143
    double elbow_position[3];    // 0144 ~ 0167
    double elbow_velocity[3];    // 0168 ~ 0191
    double q_target[6];              // 0192 ~ 0239
    double qd_target[6];             // 0240 ~ 0287
    double qdd_target[6];            // 0288 ~ 0335
    double i_target[6];              // 0336 ~ 0383
    double m_target[6];              // 0384 ~ 0431
    double q_actual[6];              // 0432 ~ 0479
    double qd_actual[6];             // 0480 ~ 0527
    double i_actual[6];              // 0528 ~ 0575
    double i_control[6];             // 0576 ~ 0623
    double tool_vector_actual[6];    // 0624 ~ 0671
    double TCP_speed_actual[6];      // 0672 ~ 0719
    double TCP_force[6];             // 0720 ~ 0767
    double Tool_vector_target[6];    // 0768 ~ 0815
    double TCP_speed_target[6];      // 0816 ~ 0863
    double motor_temperatures[6];    // 0864 ~ 0911
    double joint_modes[6];           // 0912 ~ 0959
    double v_actual[6];              // 960  ~ 1007
    int8_t handtype[4];              // 1008,1009,1010,1011 R、D、N、cfg
    int8_t userCoordinate;           // 1012
    int8_t toolCoordinate;           // 1013
    int8_t isRunQueuedCmd;           // 1014
    int8_t isPauseCmdFlag;           // 1015
    int8_t velocityRatio;            // 1016
    int8_t accelerationRatio;        // 1017
    int8_t jerkRatio;                // 1018
    int8_t xyzVelocityRatio;         // 1019
    int8_t rVelocityRatio;           // 1020
    int8_t xyzAccelerationRatio;     // 1021
    int8_t rAccelerationRatio;       // 1022
    int8_t xyzJerkRatio;             // 1023
    int8_t rJerkRatio;               // 1024
    int8_t BrakeStatus;              // 1025 机器人抱闸状态
    int8_t EnableStatus;             // 1026 机器人使能状态
    int8_t DragStatus;               // 1027 机器人拖拽状态
    int8_t RunningStatus;            // 1028 机器人运行状态
    int8_t ErrorStatus;              // 1029 机器人报警状态
    int8_t JogStatus;                // 1030 机器人点动状态
    int8_t RobotType;                // 1031 M1机型手系
    int8_t DragButtonSignal;         // 1032 按钮板拖拽信号
    int8_t EnableButtonSignal;       // 1033 按钮板使能信号
    int8_t RecordButtonSignal;       // 1034 按钮板录制信号
    int8_t ReappearButtonSignal;     // 1035 按钮板复现信号
    int8_t JawButtonSignal;          // 1036 按钮板夹爪控制信号
    int8_t SixForceOnline;           // 1037 六维力在线状态（未实现）
    int8_t CollisionStates;          // 1038 碰撞状态
    int8_t ArmApproachState;         // 1039 小臂接近暂停状态
    int8_t J4ApproachState;          // 1040 J4接近暂停状态
    int8_t J5ApproachState;          // 1041 J5接近暂停状态
    int8_t J6ApproachState;          // 1042 J6接近暂停状态
    int8_t Reserve2[61];             // 1043 ~ 1103   预留
    double vibrationDisZ;            // 1104 ~ 1111 加速度计测量Z轴抖动位移
    uint64_t currentCommandId;       // 1112 ~ 1119 当前运动队列id
    double m_actual[6];              // 1120 ~ 1167 实际扭矩
    double load;                     // 1168 ~ 1175
    double centerX;                  // 1176 ~ 1183
    double centerY;                  // 1184 ~ 1191
    double centerZ;                  // 1192 ~ 1199
    double user[6];                  // 1200 ~ 1247
    double tool[6];                  // 1248 ~ 1295
    double TraceIndex;               // 1296 ~ 1303
    double SixForceValue[6];         // 1304 ~ 1351
    double TargetQuaternion[4];      // 1352 ~ 1383
    double ActualQuaternion[4];      // 1384 ~ 1415
    uint16_t AutoManualMode;         // 1416 ~ 1417 手自动模式 0: 未开启 1: manual 2:auto
    int8_t Reserve3[22];             // 1418 ~ 1439
};
#pragma pack(pop)

static constexpr uint64_t EXPECTED_TEST_VALUE = 0x0123456789ABCDEF;
static constexpr size_t FRAME_LENGTH = 1440;
static constexpr size_t TEST_VALUE_OFFSET = 48;
static constexpr size_t BUFFER_SIZE = 5760;

class FrameBuffer {
private:
    uint8_t buffer_[BUFFER_SIZE];
    size_t head_ = 0;
    size_t tail_ = 0;

    size_t available() const {
        return (head_ >= tail_) ? (head_ - tail_) : (BUFFER_SIZE - tail_ + head_);
    }

public:
    void push(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            buffer_[head_] = data[i];
            head_ = (head_ + 1) % BUFFER_SIZE;
            if (head_ == tail_) {
                tail_ = (tail_ + 1) % BUFFER_SIZE;
            }
        }
    }

    bool extractFrame(RealTimeData& frame) {
        size_t avail = available();
        if (avail < FRAME_LENGTH) return false;

        size_t search_pos = tail_;
        for (size_t i = 0; i <= avail - FRAME_LENGTH; i++) {
            search_pos = (tail_ + i) % BUFFER_SIZE;

            uint64_t test_value_at_pos;
            size_t copy_len = sizeof(test_value_at_pos);

            if (search_pos + TEST_VALUE_OFFSET + copy_len <= BUFFER_SIZE) {
                memcpy(&test_value_at_pos, buffer_ + search_pos + TEST_VALUE_OFFSET, copy_len);
            } else {
                size_t remaining = BUFFER_SIZE - (search_pos + TEST_VALUE_OFFSET);
                memcpy(&test_value_at_pos, buffer_ + search_pos + TEST_VALUE_OFFSET, remaining);
                memcpy((uint8_t*)&test_value_at_pos + remaining, buffer_, copy_len - remaining);
            }

            if (test_value_at_pos == EXPECTED_TEST_VALUE) {
                if (search_pos + FRAME_LENGTH <= BUFFER_SIZE) {
                    memcpy(&frame, buffer_ + search_pos, FRAME_LENGTH);
                } else {
                    size_t remaining = BUFFER_SIZE - search_pos;
                    memcpy(&frame, buffer_ + search_pos, remaining);
                    memcpy((uint8_t*)&frame + remaining, buffer_, FRAME_LENGTH - remaining);
                }

                tail_ = (search_pos + FRAME_LENGTH) % BUFFER_SIZE;
                return true;
            }
        }

        tail_ = (tail_ + 1) % BUFFER_SIZE;
        return false;
    }

    void clear() {
        head_ = tail_ = 0;
    }

    size_t getAvailable() const {
        return available();
    }
};

class CR5Commander
{
protected:
    static constexpr double PI = 3.1415926;

private:
    mutable std::mutex mutex_;
    double current_joint_[6];
    double tool_vector_[6];
    RealTimeData real_time_data_;
    std::atomic<bool> is_running_;
    std::unique_ptr<std::thread> thread_;
    std::shared_ptr<TcpClient> motion_cmd_tcp_;
    std::shared_ptr<TcpClient> real_time_tcp_;
    std::shared_ptr<TcpClient> dash_board_tcp_;

    FrameBuffer frame_buffer_;
    int invalid_frame_count_ = 0;
    static constexpr int MAX_INVALID_FRAMES = 50;

public:
    explicit CR5Commander(const std::string& ip)
        : is_running_(false)
    {
        memset(current_joint_, 0, sizeof(current_joint_));
        memset(tool_vector_, 0, sizeof(tool_vector_));
        memset(&real_time_data_, 0, sizeof(real_time_data_));

        motion_cmd_tcp_ = std::make_shared<TcpClient>(ip, 30003);
        real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
        dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
    }

    ~CR5Commander()
    {
        is_running_ = false;
        thread_->join();
    }

    void getCurrentJointStatus(double* joint)
    {
        mutex_.lock();
        memcpy(joint, current_joint_, sizeof(current_joint_));
        mutex_.unlock();
    }

    void getToolVectorActual(double* val)
    {
        mutex_.lock();
        memcpy(val, tool_vector_, sizeof(tool_vector_));
        mutex_.unlock();
    }

    void recvTask()
    {
        uint8_t raw_buffer[4096];
        uint32_t has_read;
        int reconnect_attempts = 0;
        const int max_reconnect_attempts = 10;

        while (is_running_) {
            if (real_time_tcp_->isConnect()) {
                try {
                    if (real_time_tcp_->tcpRecv(raw_buffer, sizeof(raw_buffer), has_read, 5000)) {
                        frame_buffer_.push(raw_buffer, has_read);

                        RealTimeData frame;
                        bool found_frame = false;
                        while (frame_buffer_.extractFrame(frame)) {
                            if (frame.len == 1440) {
                                mutex_.lock();
                                real_time_data_ = frame;
                                for (uint32_t i = 0; i < 6; i++)
                                    current_joint_[i] = deg2Rad(real_time_data_.q_actual[i]);
                                memcpy(tool_vector_, real_time_data_.tool_vector_actual, sizeof(tool_vector_));
                                mutex_.unlock();

                                found_frame = true;
                                invalid_frame_count_ = 0;
                                reconnect_attempts = 0;  // 重置重连计数器
                            }
                        }

                        if (!found_frame && frame_buffer_.getAvailable() >= FRAME_LENGTH * 4) {
                            invalid_frame_count_++;
                            if (invalid_frame_count_ >= MAX_INVALID_FRAMES) {
                                ROS_ERROR("Too many invalid frames (%d), disconnecting and reconnecting...", invalid_frame_count_);
                                
                                // 主动断开并重连
                                real_time_tcp_->disConnect();
                                frame_buffer_.clear();
                                invalid_frame_count_ = 0;
                                reconnect_attempts++;
                                
                                // 指数退避：100ms * (2^min(attempts, 6))
                                int delay_ms = 100 * (1 << std::min(reconnect_attempts, 6));
                                ROS_INFO("Waiting %d ms before reconnect...", delay_ms);
                                usleep(delay_ms * 1000);
                            }
                        }
                    }
                } catch (const TcpClientException& err) {
                    real_time_tcp_->disConnect();
                    frame_buffer_.clear();
                    invalid_frame_count_ = 0;
                    reconnect_attempts++;
                    ROS_ERROR("real time tcp recv error : %s", err.what());
                    
                    // 当30004端口断开时，主动断开29999和30003端口，确保所有端口同步重连
                    if (dash_board_tcp_->isConnect()) {
                        ROS_INFO("Real-time port disconnected, also disconnecting dashboard port 29999...");
                        dash_board_tcp_->disConnect();
                    }
                    if (motion_cmd_tcp_->isConnect()) {
                        ROS_INFO("Real-time port disconnected, also disconnecting motion command port 30003...");
                        motion_cmd_tcp_->disConnect();
                    }
                    
                    // 指数退避延迟
                    int delay_ms = 100 * (1 << std::min(reconnect_attempts, 6));
                    ROS_INFO("Waiting %d ms before reconnect attempt %d...", delay_ms, reconnect_attempts);
                    usleep(delay_ms * 1000);
                }
            } else {
                frame_buffer_.clear();
                invalid_frame_count_ = 0;
                try {
                    real_time_tcp_->connect();
                    reconnect_attempts = 0;  // 连接成功后重置计数器
                } catch (const TcpClientException& err) {
                    reconnect_attempts++;
                    ROS_ERROR("move cmd tcp connect error : %s", err.what());
                    
                    // 当30004连接失败时，也断开29999和30003端口
                    if (dash_board_tcp_->isConnect()) {
                        ROS_INFO("Real-time port connect failed, also disconnecting dashboard port 29999...");
                        dash_board_tcp_->disConnect();
                    }
                    if (motion_cmd_tcp_->isConnect()) {
                        ROS_INFO("Real-time port connect failed, also disconnecting motion command port 30003...");
                        motion_cmd_tcp_->disConnect();
                    }
                    
                    // 指数退避：1s, 2s, 4s, 8s, 16s, 32s (最大)
                    int delay_sec = 1 << std::min(reconnect_attempts, 5);
                    if (reconnect_attempts > max_reconnect_attempts) {
                        ROS_WARN("Max reconnection attempts reached, waiting 30s...");
                        sleep(30);
                        reconnect_attempts = 0;  // 重置计数器，允许继续尝试
                    } else {
                        ROS_INFO("Waiting %d seconds before reconnect attempt %d...", delay_sec, reconnect_attempts);
                        sleep(delay_sec);
                    }
                }
            }

            if (!dash_board_tcp_->isConnect()) {
                static int dash_reconnect_attempts = 0;
                try {
                    dash_board_tcp_->connect();
                    dash_reconnect_attempts = 0;  // 连接成功，重置计数器
                    // 等待连接稳定
                    usleep(500000);  // 500ms
                } catch (const TcpClientException& err) {
                    dash_reconnect_attempts++;
                    ROS_ERROR("dash tcp connect error : %s", err.what());
                    // 使用指数退避，但最多等待5秒
                    int delay_sec = 1 << std::min(dash_reconnect_attempts, 2);
                    sleep(delay_sec);
                    if (dash_reconnect_attempts > 20) {
                        dash_reconnect_attempts = 0;  // 重置，防止溢出
                    }
                }
            }

            if (!motion_cmd_tcp_->isConnect()) {
                static int motion_reconnect_attempts = 0;
                try {
                    motion_cmd_tcp_->connect();
                    motion_reconnect_attempts = 0;  // 连接成功，重置计数器
                    // 等待连接稳定
                    usleep(500000);  // 500ms
                } catch (const TcpClientException& err) {
                    motion_reconnect_attempts++;
                    ROS_ERROR("motion cmd tcp connect error : %s", err.what());
                    // 使用指数退避，但最多等待5秒
                    int delay_sec = 1 << std::min(motion_reconnect_attempts, 2);
                    sleep(delay_sec);
                    if (motion_reconnect_attempts > 20) {
                        motion_reconnect_attempts = 0;  // 重置，防止溢出
                    }
                }
            }
        }
    }

    void init()
    {
        try {
            is_running_ = true;
            thread_ = std::unique_ptr<std::thread>(new std::thread(&CR5Commander::recvTask, this));
        } catch (const TcpClientException& err) {
            ROS_ERROR("Commander : %s", err.what());
        }
    }

    bool isEnable() const
    {
        mutex_.lock();
        bool result = real_time_data_.robot_mode == 5;
        mutex_.unlock();
        return result;
    }

    bool isConnected() const
    {
        return dash_board_tcp_->isConnect() && motion_cmd_tcp_->isConnect();
    }

    void getRealData(RealTimeData& data) const
    {
        mutex_.lock();
        data = real_time_data_;
        mutex_.unlock();
    }

    uint16_t getRobotMode() const
    {
        mutex_.lock();
        uint16_t result = real_time_data_.robot_mode;
        mutex_.unlock();
        return result;
    }

    void dashboardDoCmd(const char* cmd, int32_t& err_id)
    {
        std::vector<std::string> result;
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result);
    }

    void dashboardDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result)
    {
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result);
    }

    void motionDoCmd(const char* cmd, int32_t& err_id)
    {
        std::vector<std::string> result;
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result);
    }

    void motionDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result)
    {
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result);
    }

    static void parseString(const std::string& str, const std::string& send_cmd, int32_t& err,
                            std::vector<std::string>& result)
    {
        std::regex pattern("-?\\d+");
        std::smatch matches;
        std::string::const_iterator searchStart(str.cbegin());
        
        while (std::regex_search(searchStart, str.cend(), matches, pattern)) {
            for (auto& match : matches) {
                result.push_back(match.str());
            }
            searchStart = matches.suffix().first;
        }
        
        if (result.size() >= 1) {
            err = stoi(result[0]);
        } else {
            err = -1;
        }
    }

    void dashSendCmd(const char* cmd, uint32_t len)
    {
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    bool dashRecvCmd(char* cmd, uint32_t len, uint32_t timeout)
    {
        uint32_t has_read;
        dash_board_tcp_->tcpRecv(cmd, len, has_read, timeout);
        return has_read != 0;
    }

    void realSendCmd(const char* cmd, uint32_t len)
    {
        real_time_tcp_->tcpSend(cmd, strlen(cmd));
    }

private:
    static inline double deg2Rad(double deg)
    {
        return deg * PI / 180.0;
    }
    static void tcpDoCmd(std::shared_ptr<TcpClient>& tcp, const char* cmd, int32_t& err_id,
                         std::vector<std::string>& result)
    {
        result.clear();
        err_id = 0;
        
        // 检查连接状态
        if (!tcp->isConnect()) {
            ROS_WARN("tcpDoCmd : TCP not connected, attempting to reconnect...");
            try {
                tcp->connect();
                // 等待连接稳定
                usleep(200000);  // 200ms
            } catch (const TcpClientException& err) {
                ROS_ERROR("tcpDoCmd : reconnect failed : %s", err.what());
                err_id = -1;
                return;
            }
        }
        
        try {
            uint32_t has_read;
            char buf[1024];
            memset(buf, 0, sizeof(buf));

            ROS_INFO("tcp send cmd : %s", cmd);
            tcp->tcpSend(cmd, strlen(cmd));

            // 运动指令不额外等待，减少发送延迟（原usleep(100000)已移除）
            
            // 使用带分隔符的接收方法（命令以';'结尾）
            bool success = tcp->tcpRecvWithDelimiter(buf, sizeof(buf), has_read, 100);  // 100ms超时
            
            if (!success) {
                ROS_ERROR("tcpDoCmd : recv timeout");
                err_id = -1;
                return;
            }

            ROS_INFO("tcp recv cmd : %s", buf);
            parseString(buf, cmd, err_id, result);
        } catch (const TcpClientException& err) {
            ROS_ERROR("tcpDoCmd failed : %s", err.what());
            err_id = -1;
            tcp->disConnect();  // 断开以便下次重连
        } catch (const std::exception& err) {
            ROS_ERROR("tcpDoCmd exception : %s", err.what());
            err_id = -1;
        }
    }
};