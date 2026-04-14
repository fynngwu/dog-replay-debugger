#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace dog {

class StateMachine;

class TcpServer {
public:
    TcpServer(StateMachine& sm, int port);
    ~TcpServer();

    bool Start(std::string& err);
    void Stop();

private:
    bool SetupServerSocket(int port, int& out_fd, std::string& err);
    bool SendAll(int fd, const std::string& data);
    void AcceptLoop();
    void HandleClient(int client_fd);
    std::string ProcessCommand(const std::string& line);

    StateMachine& sm_;
    int port_;
    int server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace dog
