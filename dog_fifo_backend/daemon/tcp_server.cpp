#include "tcp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <vector>

#include "motor_config.hpp"
#include "protocol.hpp"

namespace dog {

TcpServer::TcpServer(MotorIO& motor_io, int cmd_port, int state_port)
    : motor_io_(motor_io), cmd_port_(cmd_port), state_port_(state_port) {}

TcpServer::~TcpServer() {
    Stop();
}

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
    if (running_) {
        return true;
    }
    if (!SetupServerSocket(cmd_port_, cmd_server_fd_, err)) {
        return false;
    }
    if (!SetupServerSocket(state_port_, state_server_fd_, err)) {
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
        return false;
    }
    running_ = true;
    command_thread_ = std::thread(&TcpServer::CommandLoop, this);
    state_thread_ = std::thread(&TcpServer::StateLoop, this);
    return true;
}

void TcpServer::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    if (cmd_server_fd_ >= 0) {
        ::close(cmd_server_fd_);
        cmd_server_fd_ = -1;
    }
    if (state_server_fd_ >= 0) {
        ::close(state_server_fd_);
        state_server_fd_ = -1;
    }
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
    if (state_thread_.joinable()) {
        state_thread_.join();
    }
}

bool TcpServer::SendAll(int fd, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

void TcpServer::CommandLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        const int client_fd = ::accept(cmd_server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        HandleCommandClient(client_fd);
        ::close(client_fd);
    }
}

void TcpServer::HandleCommandClient(int client_fd) {
    char buf[4096];
    std::string buffer;
    while (running_) {
        const ssize_t n = ::recv(client_fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            break;
        }
        buffer.append(buf, static_cast<size_t>(n));
        while (true) {
            const size_t pos = buffer.find('\n');
            if (pos == std::string::npos) {
                break;
            }
            const std::string line = Trim(buffer.substr(0, pos));
            buffer.erase(0, pos + 1);
            if (line.empty()) {
                continue;
            }
            const std::string reply = ProcessCommand(line);
            if (!SendAll(client_fd, reply)) {
                return;
            }
        }
    }
}

std::string TcpServer::ProcessCommand(const std::string& line) {
    const std::vector<std::string> tokens = SplitWS(line);
    if (tokens.empty()) {
        return ErrorReply("empty command", "bad_command");
    }
    const std::string& op = tokens[0];
    try {
        if (op == "ping") {
            return OkReply("pong");
        }
        if (op == "init") {
            float duration_sec = 2.5f;  // 默认 2.5 秒
            int hz = 500;               // 默认 500 Hz (2ms 周期)
            if (tokens.size() >= 2) {
                duration_sec = std::stof(tokens[1]);
            }
            std::string err;
            if (!motor_io_.SmoothInitToOffset(duration_sec, hz, err)) {
                return ErrorReply(err, "init_failed");
            }
            std::ostringstream oss;
            oss << "{\"ok\":true,\"msg\":\"initialized to offset\",\"duration\":" << duration_sec << "}\n";
            return oss.str();
        }
        if (op == "disable") {
            std::string err;
            if (!motor_io_.DisableAllAndClear(err)) {
                return ErrorReply(err, "disable_failed");
            }
            return OkReply("disabled");
        }
        if (op == "setjoint" || op == "set_joint") {
            std::vector<float> targets;
            std::string err;
            if (!ParseFloatList(tokens, 1, kNumMotors, targets, err)) {
                return ErrorReply(err, "bad_command");
            }
            size_t queue_size = 0;
            if (!motor_io_.EnqueueJointTarget(targets, err, queue_size)) {
                if (err == "queue_full") {
                    std::ostringstream oss;
                    oss << "{\"ok\":false,\"code\":\"queue_full\",\"msg\":\"queue_full\",\"queue_size\":" << queue_size << "}\n";
                    return oss.str();
                }
                return ErrorReply(err, "enqueue_failed");
            }
            std::ostringstream oss;
            oss << "{\"ok\":true,\"msg\":\"queued\",\"queue_size\":" << queue_size << "}\n";
            return oss.str();
        }
        if (op == "setzero" || op == "set_zero") {
            if (tokens.size() != 2) {
                return ErrorReply("setzero expects: setzero <joint_index>", "bad_command");
            }
            const int joint_index = std::stoi(tokens[1]);
            std::string err;
            if (!motor_io_.SetZeroJoint(joint_index, err)) {
                return ErrorReply(err, "setzero_failed");
            }
            return OkReply(std::string("setzero sent to ") + kJointNames[joint_index]);
        }
        if (op == "set_mit_param") {
            if (tokens.size() != 5) {
                return ErrorReply("set_mit_param expects: set_mit_param <kp> <kd> <vel_limit> <torque_limit>", "bad_command");
            }
            const float kp = std::stof(tokens[1]);
            const float kd = std::stof(tokens[2]);
            const float vel_limit = std::stof(tokens[3]);
            const float torque_limit = std::stof(tokens[4]);
            std::string err;
            if (!motor_io_.SetMITConfig(kp, kd, vel_limit, torque_limit, err)) {
                return ErrorReply(err, "set_mit_param_failed");
            }
            std::ostringstream oss;
            oss << "{\"ok\":true,\"msg\":\"mit params updated\",\"kp\":" << kp
                << ",\"kd\":" << kd
                << ",\"vel_limit\":" << vel_limit
                << ",\"torque_limit\":" << torque_limit << "}\n";
            return oss.str();
        }
    } catch (const std::exception& e) {
        return ErrorReply(std::string("command parse failed: ") + e.what(), "bad_command");
    }
    return ErrorReply(std::string("unknown command: ") + op, "bad_command");
}

void TcpServer::StateLoop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        const int client_fd = ::accept(state_server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        while (running_) {
            if (!SendAll(client_fd, motor_io_.StateJsonLine())) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kStatePeriodMs));
        }
        ::close(client_fd);
    }
}

}  // namespace dog
