#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "dog_driver.hpp"

static const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
};

static const char kUsage[] =
R"(Usage: sm_test <command> [args...]

Commands:
  joints              Read all joint positions/velocities once
  joints --stream     Stream joints at ~10 Hz (Ctrl+C to stop)
  imu                 Read IMU data once
  imu --stream        Stream IMU data at ~10 Hz (Ctrl+C to stop)
  set_joint <idx> <rad>  Set single joint position
  enable              Enable all motors
  disable             Disable all motors
  online              Check which motors are online
  info                Print driver info (IMU status, motor status)
)";

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static void print_joints(const DogDriver::JointState& js) {
    printf("  %-10s  %12s  %12s\n", "Joint", "Pos (rad)", "Vel (rad/s)");
    printf("  %-10s  %12s  %12s\n", "----------", "------------", "------------");
    for (int i = 0; i < DogDriver::NUM_JOINTS; i++) {
        printf("  [%2d] %-6s  %+10.4f    %+10.4f\n",
               i, kJointNames[i], js.position[i], js.velocity[i]);
    }
}

static void print_imu(const DogDriver::IMUData& imu) {
    printf("  gyro  (rad/s): x=%+9.4f  y=%+9.4f  z=%+9.4f\n",
           imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
    printf("  gravity       : x=%+9.4f  y=%+9.4f  z=%+9.4f\n",
           imu.projected_gravity[0], imu.projected_gravity[1], imu.projected_gravity[2]);
}

static int cmd_joints(DogDriver& driver, bool stream) {
    if (stream) {
        std::signal(SIGINT, signal_handler);
        printf("Streaming joints at ~10 Hz (Ctrl+C to stop)...\n\n");
        int count = 0;
        while (g_running) {
            auto t0 = std::chrono::steady_clock::now();
            auto js = driver.GetJointStates();
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&time_t));

            printf("[%s] pos: ", ts);
            for (int i = 0; i < DogDriver::NUM_JOINTS; i++) {
                printf("%+.3f ", js.position[i]);
            }
            printf("\n       vel: ");
            for (int i = 0; i < DogDriver::NUM_JOINTS; i++) {
                printf("%+.2f ", js.velocity[i]);
            }
            printf("\n\n");

            count++;
            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            if (ms < 100) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100 - ms));
            }
        }
        printf("\nStopped after %d readings.\n", count);
        return 0;
    }

    // Single read
    auto js = driver.GetJointStates();
    printf("=== get_joints ===\n");
    print_joints(js);
    return 0;
}

static int cmd_imu(DogDriver& driver, bool stream) {
    if (!driver.IsIMUConnected()) {
        printf("WARNING: IMU is not connected.\n");
        return 1;
    }

    if (stream) {
        std::signal(SIGINT, signal_handler);
        printf("Streaming IMU at ~10 Hz (Ctrl+C to stop)...\n\n");
        int count = 0;
        while (g_running) {
            auto t0 = std::chrono::steady_clock::now();
            auto imu = driver.GetIMUData();
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&time_t));

            printf("[%s] ", ts);
            print_imu(imu);
            printf("\n");

            count++;
            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            if (ms < 100) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100 - ms));
            }
        }
        printf("\nStopped after %d readings.\n", count);
        return 0;
    }

    // Single read
    auto imu = driver.GetIMUData();
    printf("=== get_imu ===\n");
    print_imu(imu);
    return 0;
}

static int cmd_set_joint(DogDriver& driver, int idx, float rad) {
    if (idx < 0 || idx >= DogDriver::NUM_JOINTS) {
        printf("ERROR: joint index must be 0-11, got %d\n", idx);
        return 1;
    }
    printf("Setting [%d] %s = %.6f rad (%.2f deg)\n",
           idx, kJointNames[idx], rad, rad * 57.2958);
    int ret = driver.SetJointPosition(idx, rad);
    if (ret == 0) {
        printf("OK\n");
    } else {
        printf("ERROR: SetJointPosition returned %d\n", ret);
    }
    return ret;
}

static int cmd_online(DogDriver& driver) {
    printf("Motor online status:\n");
    for (int i = 0; i < DogDriver::NUM_JOINTS; i++) {
        bool online = driver.IsJointOnline(i);
        printf("  [%2d] %-6s  %s\n", i, kJointNames[i], online ? "ONLINE" : "OFFLINE");
    }
    return 0;
}

static int cmd_info(DogDriver& driver) {
    printf("IMU:    %s\n", driver.IsIMUConnected() ? "connected" : "NOT connected");
    printf("Motors: ");
    int online_count = 0;
    for (int i = 0; i < DogDriver::NUM_JOINTS; i++) {
        if (driver.IsJointOnline(i)) online_count++;
    }
    printf("%d/%d online\n", online_count, DogDriver::NUM_JOINTS);

    auto js = driver.GetJointStates();
    auto imu = driver.GetIMUData();

    printf("\n--- Current Joint States ---\n");
    print_joints(js);

    if (driver.IsIMUConnected()) {
        printf("\n--- Current IMU Data ---\n");
        print_imu(imu);
    }
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("%s", kUsage);
        return 1;
    }

    std::string cmd = argv[1];

    // Check for commands that don't need hardware
    if (cmd == "-h" || cmd == "--help") {
        printf("%s", kUsage);
        return 0;
    }

    printf("Initializing DogDriver (this takes a few seconds)...\n");
    DogDriver driver;
    printf("DogDriver ready.\n\n");

    if (cmd == "joints") {
        bool stream = (argc >= 3 && std::strcmp(argv[2], "--stream") == 0);
        return cmd_joints(driver, stream);
    }
    if (cmd == "imu") {
        bool stream = (argc >= 3 && std::strcmp(argv[2], "--stream") == 0);
        return cmd_imu(driver, stream);
    }
    if (cmd == "set_joint") {
        if (argc < 4) {
            printf("Usage: sm_test set_joint <idx> <rad>\n");
            return 1;
        }
        int idx = std::atoi(argv[2]);
        float rad = std::atof(argv[3]);
        return cmd_set_joint(driver, idx, rad);
    }
    if (cmd == "enable") {
        int ret = driver.EnableAll();
        printf("EnableAll: %s\n", ret == 0 ? "OK" : "FAILED");
        return ret;
    }
    if (cmd == "disable") {
        int ret = driver.DisableAll();
        printf("DisableAll: %s\n", ret == 0 ? "OK" : "FAILED");
        return ret;
    }
    if (cmd == "online") {
        return cmd_online(driver);
    }
    if (cmd == "info") {
        return cmd_info(driver);
    }

    printf("Unknown command: %s\n", cmd.c_str());
    printf("%s", kUsage);
    return 1;
}
