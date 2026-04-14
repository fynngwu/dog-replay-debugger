#include "tcp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "protocol.hpp"
#include "state_machine.hpp"

namespace dog {

TcpServer::TcpServer(StateMachine& sm, int port) : sm_(sm), port_(port) {}

TcpServer::~TcpServer() { Stop(); }

bool TcpServer::SetupServerSocket(int port, int& out_fd, std::string& err) {
    out_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (out_fd < 0) {
        err = std::string("socket failed: ") + std::strerror(errno);
        return false;
    }
    const int opt = 1;
    ::setsockopt(out_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(out_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        err = std::string("bind failed: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    if (::listen(out_fd, 8) < 0) {
        err = std::string("listen failed: ") + std::strerror(errno);
        ::close(out_fd);
        out_fd = -1;
        return false;
    }
    return true;
}

bool TcpServer::Start(std::string& err) {
    if (running_) return true;
    if (!SetupServerSocket(port_, server_fd_, err)) return false;
    running_ = true;
    thread_ = std::thread(&TcpServer::AcceptLoop, this);
    return true;
}

void TcpServer::Stop() {
    if (!running_) return;
    running_ = false;
    if (server_fd_ >= 0) {
        ::close(server_fd_);
        server_fd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
}

bool TcpServer::SendAll(int fd, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) continue;
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

void TcpServer::AcceptLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        const int client_fd =
            ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        HandleClient(client_fd);
        ::close(client_fd);
    }
}

namespace {

std::string Trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

}  // namespace

void TcpServer::HandleClient(int client_fd) {
    char buf[4096];
    std::string buffer;
    while (running_) {
        const ssize_t n = ::recv(client_fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        buffer.append(buf, static_cast<size_t>(n));
        while (true) {
            const size_t pos = buffer.find('\n');
            if (pos == std::string::npos) break;
            const std::string line = Trim(buffer.substr(0, pos));
            buffer.erase(0, pos + 1);
            if (line.empty()) continue;
            const std::string reply = ProcessCommand(line);
            if (!SendAll(client_fd, reply)) return;
        }
    }
}

std::string TcpServer::ProcessCommand(const std::string& line) {
    const auto cmd = ParseCommand(line);
    switch (cmd.type) {
        case Command::RequestMode: {
            if (cmd.args.size() != 1)
                return MakeErrorReply("usage: request_mode <init|execute|policy|stop>");
            Mode mode;
            const std::string& m = cmd.args[0];
            if (m == "init")          mode = Mode::INIT;
            else if (m == "execute")  mode = Mode::EXECUTE;
            else if (m == "policy")   mode = Mode::POLICY;
            else if (m == "stop")     mode = Mode::STOP;
            else return MakeErrorReply("unknown mode: " + m + ", use init/execute/policy/stop");

            std::string err = sm_.RequestMode(mode);
            if (!err.empty()) return MakeErrorReply(err);
            return MakeOkReply("mode set to " + m);
        }
        case Command::Target: {
            if (static_cast<int>(cmd.args.size()) != StateMachine::NUM_JOINTS) {
                return MakeErrorReply("target requires " +
                    std::to_string(StateMachine::NUM_JOINTS) + " float values");
            }
            std::array<float, StateMachine::NUM_JOINTS> joints;
            try {
                for (int i = 0; i < StateMachine::NUM_JOINTS; ++i)
                    joints[i] = std::stof(cmd.args[i]);
            } catch (const std::exception& e) {
                return MakeErrorReply(std::string("float parse failed: ") + e.what());
            }

            std::string err = sm_.RequestMode(Mode::EXECUTE);
            if (!err.empty()) return MakeErrorReply(err);

            err = sm_.EnqueueTarget(joints);
            if (!err.empty()) return MakeErrorReply(err);
            return MakeOkReply("target queued");
        }
        case Command::GetMode: {
            std::string mode_str;
            switch (sm_.GetCurrentMode()) {
                case Mode::INIT:    mode_str = "INIT"; break;
                case Mode::EXECUTE: mode_str = "EXECUTE"; break;
                case Mode::POLICY:  mode_str = "POLICY"; break;
                case Mode::STOP:    mode_str = "STOP"; break;
            }
            return MakeOkReply("current mode: " + mode_str);
        }
        case Command::GetJoints: {
            auto js = sm_.GetJointStates();
            return MakeOkData(SerializeJoints(js.position.data(), js.velocity.data(),
                                              StateMachine::NUM_JOINTS));
        }
        case Command::GetImu: {
            auto imu = sm_.GetIMUData();
            return MakeOkData(SerializeIMU(imu.angular_velocity.data(),
                                             imu.projected_gravity.data()));
        }
        case Command::GetAll: {
            auto js = sm_.GetJointStates();
            auto imu = sm_.GetIMUData();
            std::string mode_str;
            switch (sm_.GetCurrentMode()) {
                case Mode::INIT:    mode_str = "INIT"; break;
                case Mode::EXECUTE: mode_str = "EXECUTE"; break;
                case Mode::POLICY:  mode_str = "POLICY"; break;
                case Mode::STOP:    mode_str = "STOP"; break;
            }
            return MakeOkData(SerializeAll(mode_str, js.position.data(), js.velocity.data(),
                                           StateMachine::NUM_JOINTS, imu.angular_velocity.data(),
                                           imu.projected_gravity.data()));
        }
        case Command::Unknown:
            return MakeErrorReply("unknown command, use request_mode/target/get_mode/get_joints/get_imu/get_all");
            break;
    }
    return MakeErrorReply("internal error");
}

}  // namespace dog
