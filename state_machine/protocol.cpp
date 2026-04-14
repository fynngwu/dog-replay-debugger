#include "protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>

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

// Escape a string for inclusion in a JSON string value.
static std::string JsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

ParsedCommand ParseCommand(const std::string& line) {
    ParsedCommand cmd;
    const auto tokens = SplitWS(Trim(line));
    if (tokens.empty()) return cmd;

    const std::string& op = tokens[0];
    if (op == "request_mode")      cmd.type = Command::RequestMode;
    else if (op == "target")       cmd.type = Command::Target;
    else if (op == "get_mode")     cmd.type = Command::GetMode;
    else if (op == "get_joints")   cmd.type = Command::GetJoints;
    else if (op == "get_imu")      cmd.type = Command::GetImu;
    else                           cmd.type = Command::Unknown;

    for (size_t i = 1; i < tokens.size(); ++i)
        cmd.args.push_back(tokens[i]);
    return cmd;
}

std::string MakeOkReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

std::string MakeOkData(const std::string& json_data) {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"data\":" << json_data << "}\n";
    return oss.str();
}

std::string MakeErrorReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

std::string SerializeJoints(const float* position, const float* velocity, int count) {
    std::ostringstream oss;
    oss << "{\"position\":[";
    for (int i = 0; i < count; ++i) {
        if (i > 0) oss << ",";
        oss << position[i];
    }
    oss << "],\"velocity\":[";
    for (int i = 0; i < count; ++i) {
        if (i > 0) oss << ",";
        oss << velocity[i];
    }
    oss << "]}";
    return oss.str();
}

std::string SerializeIMU(const float* gyro, const float* gravity) {
    std::ostringstream oss;
    oss << "{\"gyro\":[";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) oss << ",";
        oss << gyro[i];
    }
    oss << "],\"gravity\":[";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) oss << ",";
        oss << gravity[i];
    }
    oss << "]}";
    return oss.str();
}

}  // namespace dog
