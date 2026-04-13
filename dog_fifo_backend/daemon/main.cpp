#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "motor_io.hpp"
#include "tcp_server.hpp"

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running = false;
}

int ParsePort(const char* text, int fallback) {
    if (!text) return fallback;
    try {
        return std::stoi(text);
    } catch (...) {
        return fallback;
    }
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    int cmd_port = 47001;
    int state_port = 47002;
    if (argc >= 2) {
        cmd_port = ParsePort(argv[1], cmd_port);
    }
    if (argc >= 3) {
        state_port = ParsePort(argv[2], state_port);
    }

    dog::MotorIO motor_io;
    std::string err;
    if (!motor_io.Initialize(err)) {
        std::cerr << "[fatal] MotorIO initialize failed: " << err << std::endl;
        return 1;
    }

    if (!motor_io.StartWorker(err)) {
        std::cerr << "[fatal] MotorIO start worker failed: " << err << std::endl;
        return 1;
    }

    dog::TcpServer server(motor_io, cmd_port, state_port);
    if (!server.Start(err)) {
        std::cerr << "[fatal] TcpServer start failed: " << err << std::endl;
        return 1;
    }

    std::cout << "dog_fifo_backend started. cmd_port=" << cmd_port
              << " state_port=" << state_port << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    server.Stop();
    motor_io.StopWorker();
    return 0;
}
