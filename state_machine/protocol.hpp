#pragma once

#include <string>
#include <vector>

namespace dog {

enum class Command { RequestMode, Target, GetMode, GetJoints, GetImu, GetAll, Unknown };

struct ParsedCommand {
    Command type = Command::Unknown;
    std::vector<std::string> args;
};

ParsedCommand ParseCommand(const std::string& line);

std::string MakeOkReply(const std::string& msg);
std::string MakeOkData(const std::string& json_data);
std::string MakeErrorReply(const std::string& msg);

std::string SerializeJoints(const float* position, const float* velocity, int count);
std::string SerializeIMU(const float* gyro, const float* gravity);
std::string SerializeAll(const std::string& mode, const float* position, const float* velocity, int joint_count, const float* gyro, const float* gravity);

}  // namespace dog
