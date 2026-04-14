#include "observations.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>

#include "serial.h"
#include "wit_c_sdk.h"

IMUComponent* IMUComponent::instance_ = nullptr;
int IMUComponent::fd = -1;
int IMUComponent::s_iCurBaud = 9600;

static volatile char s_cDataUpdate = 0;

IMUComponent::IMUComponent(const char* dev) : dev_path(dev), running_(true) {
    instance_ = this;

    std::memset(acc, 0, sizeof(acc));
    std::memset(gyro, 0, sizeof(gyro));
    std::memset(angle, 0, sizeof(angle));
    std::memset(quaternion, 0, sizeof(quaternion));

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitDelayMsRegister(Delayms);
    WitSerialWriteRegister(SerialWriteRegister);
    WitRegisterCallBack(SensorDataUpdata);

    AutoScanSensor();

    if (ConfigureSensorOutputs() != 0) {
        std::cerr << "Configure IMU failed" << std::endl;
    }

    update_thread_ = std::thread(&IMUComponent::UpdateLoop, this);
}

IMUComponent::~IMUComponent() {
    running_ = false;
    if (update_thread_.joinable()) {
        update_thread_.join();
    }
    if (fd >= 0) {
        serial_close(fd);
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void IMUComponent::UpdateLoop() {
    while (running_) {
        Update();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void IMUComponent::Update() {
    if (fd < 0) return;

    char cBuff[1];
    while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
        WitSerialDataIn(cBuff[0]);
    }
}

std::vector<float> IMUComponent::GetObs() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<float> obs;
    obs.reserve(6);

    obs.push_back(gyro[1] * M_PI / 180.0f);
    obs.push_back(-gyro[0] * M_PI / 180.0f);
    obs.push_back(gyro[2] * M_PI / 180.0f);

    float quat0 = quaternion[0];
    float quat1 = quaternion[1];
    float quat2 = quaternion[2];
    float quat3 = quaternion[3];

    float gx = 2 * (quat1 * quat3 - quat0 * quat2);
    float gy = 2 * (quat2 * quat3 + quat0 * quat1);
    float gz = 1 - 2 * (quat1 * quat1 + quat2 * quat2);

    float norm = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (norm > 1e-6f) {
        gx /= norm;
        gy /= norm;
        gz /= norm;
    }

    obs.push_back(-gy);
    obs.push_back(gx);
    obs.push_back(-gz);

    return obs;
}

std::vector<float> DriverIMUAdapter::GetObs() const {
    auto imu = driver_.GetIMUData();
    std::vector<float> obs;
    obs.reserve(6);
    // gyro: DogDriver returns body-frame angular velocity in rad/s
    obs.push_back(imu.angular_velocity[1]);
    obs.push_back(-imu.angular_velocity[0]);
    obs.push_back(imu.angular_velocity[2]);
    // projected gravity: already a unit vector in body frame
    obs.push_back(-imu.projected_gravity[1]);
    obs.push_back(imu.projected_gravity[0]);
    obs.push_back(-imu.projected_gravity[2]);
    return obs;
}

void IMUComponent::AutoScanSensor() {
    int i, iRetry;
    char cBuff[1];

    for(i = 0; i < sizeof(c_uiBaud)/sizeof(int); i++) {
        if(fd >= 0) serial_close(fd);

        s_iCurBaud = c_uiBaud[i];
        fd = serial_open((unsigned char*)dev_path, s_iCurBaud);

        if(fd < 0) continue;

        iRetry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Delayms(200);

            while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
                WitSerialDataIn(cBuff[0]);
            }

            if(s_cDataUpdate != 0) {
                std::cout << "IMU Connected at baud " << s_iCurBaud << std::endl;
                return;
            }
            iRetry--;
        } while(iRetry);
    }
    std::cerr << "Can not find IMU sensor" << std::endl;
}

int IMUComponent::ConfigureSensorOutputs() {
    int32_t ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_Q);
    if(ret != WIT_HAL_OK) {
        return -1;
    }
    return 0;
}

void IMUComponent::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    s_cDataUpdate = 1;

    if (!instance_) return;

    std::lock_guard<std::mutex> lock(instance_->data_mutex_);

    for(int i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case AX: case AY: case AZ:
                for(int j=0; j<3; j++) instance_->acc[j] = sReg[AX+j] / 32768.0f * 16.0f;
                break;
            case GX: case GY: case GZ:
                for(int j=0; j<3; j++) instance_->gyro[j] = sReg[GX+j] / 32768.0f * 2000.0f;
                break;
            case Roll: case Pitch: case Yaw:
                for(int j=0; j<3; j++) instance_->angle[j] = sReg[Roll+j] / 32768.0f * 180.0f;
                break;
            case q0: case q1: case q2: case q3:
                for(int j=0; j<4; j++) instance_->quaternion[j] = sReg[q0+j] / 32768.0f;
                break;
        }
        uiReg++;
    }
}

void IMUComponent::Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

void IMUComponent::SerialWriteRegister(uint8_t *p_ucData, uint32_t uiLen) {
    if (fd >= 0) {
        serial_write_data(fd, p_ucData, uiLen);
    }
}

JointComponent::JointComponent(DogDriver& driver) : driver_(driver) {}

std::vector<float> JointComponent::GetObs() const {
    auto js = driver_.GetJointStates();
    std::vector<float> obs(DogDriver::NUM_JOINTS * 2);

    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        float pos = js.position[i];
        float vel = js.velocity[i];

        if (i >= 8 && i <= 11) {
            obs[i] = pos / DogDriver::KNEE_GEAR_RATIO;
            obs[DogDriver::NUM_JOINTS + i] = vel / DogDriver::KNEE_GEAR_RATIO;
        } else {
            obs[i] = pos;
            obs[DogDriver::NUM_JOINTS + i] = vel;
        }
    }
    return obs;
}

Gamepad::Gamepad(const char* dev) : fd(-1), running(true) {
    fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open gamepad device: " << dev << std::endl;
        running = false;
        return;
    }
    std::memset(axes, 0, sizeof(axes));
    read_thread = std::thread(&Gamepad::ReadLoop, this);
}

Gamepad::~Gamepad() {
    running = false;
    if (read_thread.joinable()) {
        read_thread.join();
    }
    if (fd >= 0) {
        close(fd);
    }
}

float Gamepad::GetAxis(int axis) const {
    if (axis < 0 || axis >= JS_AXIS_LIMIT) return 0.0f;
    std::lock_guard<std::mutex> lock(data_mutex);
    return axes[axis];
}

bool Gamepad::IsConnected() const {
    return fd >= 0;
}

void Gamepad::ReadLoop() {
    struct js_event event;
    while (running) {
        while (read(fd, &event, sizeof(event)) == sizeof(event)) {
            if (event.type & JS_EVENT_AXIS) {
                if (event.number < JS_AXIS_LIMIT) {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    axes[event.number] = (float)event.value / 32767.0f;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

std::vector<float> ActionComponent::GetObs() const {
    return prev_actions;
}

void ActionComponent::SetAction(const std::vector<float>& action) {
    prev_actions = action;
}

std::vector<float> CommandComponent::GetObs() const {
    if (gamepad && gamepad->IsConnected()) {
        command[0] = -gamepad->GetAxis(1) * 0.5;
        command[1] = 0.0f;
        command[2] = -gamepad->GetAxis(3) * 0.5;
    }
    return command;
}

void CommandComponent::SetCommand(const std::vector<float>& cmd) {
    command = cmd;
}

void CommandComponent::Update() {}

RoboObsFrame::RoboObsFrame() : timestamp(0) {}

void RoboObsFrame::AddComponent(std::shared_ptr<ObsComponent> component) {
    components.push_back(component);
}

std::vector<float> RoboObsFrame::GetObs() const {
    std::vector<float> obs;
    for (const auto& comp : components) {
        auto sub_obs = comp->GetObs();
        obs.insert(obs.end(), sub_obs.begin(), sub_obs.end());
    }
    return obs;
}

RoboObs::RoboObs(int history_length) : obs_dim(0), history_length(history_length) {}

void RoboObs::AddComponent(std::shared_ptr<ObsComponent> component) {
    frame.AddComponent(component);
}

void RoboObs::UpdateObs() {
    for (const auto& component : frame.components) {
        component->Update();
    }
    auto current_obs = frame.GetObs();
    obs_dim = current_obs.size();

    history.push_back(current_obs);

    while (history.size() > (size_t)history_length) {
        history.pop_front();
    }
}

std::vector<float> RoboObs::GetWholeObs() const {
    std::vector<float> whole_obs;
    int missing = history_length - history.size();

    if (missing > 0 && obs_dim > 0) {
        std::vector<float> padding(missing * obs_dim, 0.0f);
        whole_obs.insert(whole_obs.end(), padding.begin(), padding.end());
    }

    for (const auto& obs : history) {
        whole_obs.insert(whole_obs.end(), obs.begin(), obs.end());
    }
    return whole_obs;
}

std::vector<float> RoboObs::GetSingleObs() const {
    if (history.empty()) return std::vector<float>(obs_dim, 0.0f);
    return history.back();
}
