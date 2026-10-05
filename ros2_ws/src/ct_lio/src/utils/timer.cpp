// timer.cpp - Timer 静态成员定义
#include "ct_lio_ros2/utils/timer.hpp"

namespace ct_lio {

std::map<std::string, Timer::TimerRecord> Timer::records_;

}  // namespace ct_lio
