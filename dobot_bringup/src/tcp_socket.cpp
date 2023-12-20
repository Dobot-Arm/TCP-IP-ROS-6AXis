/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/10
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <cerrno>
#include <cstring>
#include <utility>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <dobot_bringup/tcp_socket.h>

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)
{
}

TcpClient::~TcpClient()
{
    close();
}

void TcpClient::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

void TcpClient::connect()
{
    if (fd_ < 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};

    memset(&addr, 0, sizeof(addr));
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);

    if (::connect(fd_, (sockaddr*)&addr, sizeof(addr)) < 0)
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(errno));
    is_connected_ = true;

    ROS_INFO("%s : connect successfully", toString().c_str());
}

void TcpClient::disConnect()
{
    if (is_connected_)
    {
        fd_ = -1;
        is_connected_ = false;
        ::close(fd_);
    }
}

bool TcpClient::isConnect() const
{
    return is_connected_;
}

void TcpClient::tcpSend(const void* buf, uint32_t len)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    ROS_INFO("send : %s", (const char*)buf);
    const auto* tmp = (const uint8_t*)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, MSG_NOSIGNAL);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void* buf, uint32_t len, uint32_t& has_read, uint32_t timeout)
{
    uint8_t* tmp = (uint8_t*)buf;    // NOLINT(modernize-use-auto)

    fd_set read_fds;
    timeval tv = { 0, 0 };
    timeval* tp;
    if (timeout)
    {
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        tp = &tv;
    }
    else
    {
        tp = nullptr;
    }
    has_read = 0;
    while (len)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        int err = ::select(fd_ + 1, &read_fds, nullptr, nullptr, tp);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        else if (err == 0)
        {
            return false;
        }

        err = (int)::read(fd_, tmp, len);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        else if (err == 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }
        len -= err;
        tmp += err;
        has_read += err;
    }

    return true;
}

std::string TcpClient::toString()
{
    return ip_ + ":" + std::to_string(port_);
}
