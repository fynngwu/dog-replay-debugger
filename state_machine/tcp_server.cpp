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
#include <vector>

#include "state_machine.hpp"

namespace dog {

static std::string Trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

static std::vector<std::string> SplitWS(const std::string& s) {
    std::stringstream ss(s);
    std::vector<std::string> out;
    std::string token;
    while (ss >> token) out.push_back(token);
    return out;
}

static std::string OkReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"msg\":\"" << msg << "\"}\n";
    return oss.str();
}

static std::string OkData(const std::string& data) {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"data\":" << data << "}\n";
    return oss.str();
}

static std::string ErrorReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"msg\":\"" << msg << "\"}\n";
    return oss.str();
}

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
    const std::vector<std::string> tokens = SplitWS(line);
    if (tokens.empty()) return ErrorReply("empty command");

    const std::string& op = tokens[0];

    if (op == "request_mode") {
        if (tokens.size() != 2) return ErrorReply("usage: request_mode <init|execute|policy|stop>");
        Mode mode;
        const std::string& m = tokens[1];
        if (m == "init") mode = Mode::INIT;
        else if (m == "execute") mode = Mode::EXECUTE;
        else if (m == "policy") mode = Mode::POLICY;
        else if (m == "stop") mode = Mode::STOP;
        else return ErrorReply("unknown mode: " + m + ", use init/execute/policy/stop");

        std::string err = sm_.RequestMode(mode);
        if (!err.empty()) return ErrorReply(err);
        return OkReply("mode set to " + m);
    }

    if (op == "target") {
        if (tokens.size() != 1 + StateMachine::NUM_JOINTS) {
            return ErrorReply("target requires " +
                              std::to_string(StateMachine::NUM_JOINTS) + " float values");
        }
        std::array<float, StateMachine::NUM_JOINTS> joints;
        try {
            for (int i = 0; i < StateMachine::NUM_JOINTS; ++i) {
                joints[i] = std::stof(tokens[1 + i]);
            }
        } catch (const std::exception& e) {
            return ErrorReply(std::string("float parse failed: ") + e.what());
        }

        std::string err = sm_.RequestMode(Mode::EXECUTE);
        if (!err.empty()) return ErrorReply(err);

        err = sm_.EnqueueTarget(joints);
        if (!err.empty()) return ErrorReply(err);
        return OkReply("target queued");
    }

    if (op == "get_mode") {
        std::string mode_str;
        switch (sm_.GetCurrentMode()) {
            case Mode::INIT: mode_str = "INIT"; break;
            case Mode::EXECUTE: mode_str = "EXECUTE"; break;
            case Mode::POLICY: mode_str = "POLICY"; break;
            case Mode::STOP: mode_str = "STOP"; break;
        }
        return OkReply("current mode: " + mode_str);
    }

    if (op == "get_joints") {
        auto js = sm_.GetJointStates();
        std::ostringstream oss;
        oss << "{\"position\":[";
        for (int i = 0; i < StateMachine::NUM_JOINTS; ++i) {
            if (i > 0) oss << ",";
            oss << js.position[i];
        }
        oss << "],\"velocity\":[";
        for (int i = 0; i < StateMachine::NUM_JOINTS; ++i) {
            if (i > 0) oss << ",";
            oss << js.velocity[i];
        }
        oss << "]}";
        return OkData(oss.str());
    }

    if (op == "get_imu") {
        auto imu = sm_.GetIMUData();
        std::ostringstream oss;
        oss << "{\"gyro\":[";
        for (int i = 0; i < 3; ++i) {
            if (i > 0) oss << ",";
            oss << imu.angular_velocity[i];
        }
        oss << "],\"gravity\":[";
        for (int i = 0; i < 3; ++i) {
            if (i > 0) oss << ",";
            oss << imu.projected_gravity[i];
        }
        oss << "]}";
        return OkData(oss.str());
    }

    return ErrorReply("unknown command: " + op + ", use request_mode/target/get_mode/get_joints/get_imu");
}

}  // namespace dog
