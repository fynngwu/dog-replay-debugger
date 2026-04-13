#pragma once

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace dog {

inline std::string Trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

inline std::vector<std::string> SplitWS(const std::string& s) {
    std::stringstream ss(s);
    std::vector<std::string> out;
    std::string token;
    while (ss >> token) out.push_back(token);
    return out;
}

inline bool ParseFloatList(const std::vector<std::string>& tokens,
                           size_t start,
                           size_t expected,
                           std::vector<float>& out,
                           std::string& err) {
    if (tokens.size() != start + expected) {
        err = "expected " + std::to_string(expected) + " float values";
        return false;
    }
    out.clear();
    out.reserve(expected);
    try {
        for (size_t i = 0; i < expected; ++i) {
            out.push_back(std::stof(tokens[start + i]));
        }
    } catch (const std::exception& e) {
        err = std::string("float parse failed: ") + e.what();
        return false;
    }
    return true;
}

inline std::string JsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out;
}

inline std::string JsonArray(const std::vector<float>& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ',';
        oss << values[i];
    }
    oss << ']';
    return oss.str();
}

inline std::string OkReply(const std::string& msg) {
    std::ostringstream oss;
    oss << "{\"ok\":true,\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

inline std::string ErrorReply(const std::string& msg, const std::string& code) {
    std::ostringstream oss;
    oss << "{\"ok\":false,\"code\":\"" << JsonEscape(code)
        << "\",\"msg\":\"" << JsonEscape(msg) << "\"}\n";
    return oss.str();
}

}  // namespace dog
