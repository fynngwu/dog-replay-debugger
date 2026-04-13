#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "motor_io.hpp"

namespace dog {

class TcpServer {
public:
    TcpServer(MotorIO& motor_io, int cmd_port, int state_port);
    ~TcpServer();

    bool Start(std::string& err);
    void Stop();

private:
    bool SetupServerSocket(int port, int& out_fd, std::string& err);
    bool SendAll(int fd, const std::string& data);
    void CommandLoop();
    void StateLoop();
    void HandleCommandClient(int client_fd);
    std::string ProcessCommand(const std::string& line);

    MotorIO& motor_io_;
    int cmd_port_;
    int state_port_;
    int cmd_server_fd_ = -1;
    int state_server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread command_thread_;
    std::thread state_thread_;
};

}  // namespace dog
