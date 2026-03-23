/*
 * include/orientation.hpp
 */
#pragma once
#include <string>

enum class Orientation {
    X_PLUS,  // (1, 0) East
    Y_PLUS,  // (0, 1) South (assuming y increases downwards in grid maps usually, or up depending on coord sys)
    X_MINUS, // (-1, 0) West
    Y_MINUS  // (0, -1) North
};

// 辅助函数：将方向转换为字符串用于输出
inline std::string orientationToString(Orientation dir) {
    switch (dir) {
        case Orientation::X_PLUS:  return "X_PLUS";
        case Orientation::X_MINUS: return "X_MINUS";
        case Orientation::Y_PLUS:  return "Y_PLUS";
        case Orientation::Y_MINUS: return "Y_MINUS";
        default: return "UNKNOWN";
    }
}