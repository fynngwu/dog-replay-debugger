#include <array>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

static bool SendAll(int fd, const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) continue;
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

static std::string SendAndRecv(int fd, const std::string& cmd) {
    if (!SendAll(fd, cmd.c_str(), cmd.size())) return "";
    if (!SendAll(fd, "\n", 1)) return "";

    char buf[4096];
    std::string response;
    while (true) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        response.append(buf, static_cast<size_t>(n));
        if (response.find('\n') != std::string::npos) break;
    }
    return response;
}

static void PrintUsage() {
    std::cout << "Usage:\n"
              << "  dog_client <host> <port> request_mode <init|execute|stop>\n"
              << "  dog_client <host> <port> target <j0> <j1> ... <j11>\n"
              << "  dog_client <host> <port> get_mode\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        PrintUsage();
        return 1;
    }

    const char* host = argv[1];
    int port = std::stoi(argv[2]);
    std::string op = argv[3];

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << std::endl;
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        std::cerr << "invalid address: " << host << std::endl;
        ::close(fd);
        return 1;
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "connect failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }

    std::string cmd;
    if (op == "request_mode") {
        if (argc != 5) {
            std::cerr << "usage: request_mode <init|execute|stop>" << std::endl;
            ::close(fd);
            return 1;
        }
        cmd = "request_mode " + std::string(argv[4]);
    } else if (op == "target") {
        if (argc != 16) {
            std::cerr << "target requires 12 float values" << std::endl;
            ::close(fd);
            return 1;
        }
        cmd = "target";
        for (int i = 4; i < 16; ++i) {
            cmd += " " + std::string(argv[i]);
        }
    } else if (op == "get_mode") {
        cmd = "get_mode";
    } else {
        std::cerr << "unknown command: " << op << std::endl;
        PrintUsage();
        ::close(fd);
        return 1;
    }

    std::string reply = SendAndRecv(fd, cmd);
    std::cout << reply;
    ::close(fd);
    return 0;
}
