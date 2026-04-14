#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "dog_driver.hpp"
#include "state_machine.hpp"
#include "tcp_server.hpp"

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running = false; }

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

    int port = 48001;
    if (argc >= 2) port = ParsePort(argv[1], port);

    std::cout << "state_machine starting: port=" << port << std::endl;

    DogDriver driver;
    dog::StateMachine sm(driver);

    std::thread sm_thread([&sm]() { sm.Run(); });

    dog::TcpServer server(sm, port);
    std::string err;
    if (!server.Start(err)) {
        std::cerr << "[fatal] TCP server start failed: " << err << std::endl;
        sm.Stop();
        sm_thread.join();
        return 1;
    }

    std::cout << "state_machine ready. port=" << port << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "shutting down..." << std::endl;
    server.Stop();
    sm.Stop();
    sm_thread.join();
    return 0;
}
