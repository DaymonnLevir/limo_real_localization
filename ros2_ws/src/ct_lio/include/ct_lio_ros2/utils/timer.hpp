// timer.hpp - 轻量性能计时工具 (ct_lio 命名空间版)
// 来源: 原 ct-lio src/common/timer/timer.h, 仅保留 Evaluate(按函数名累积耗时)
// 设计: 按函数名累积耗时, 每函数记录条数上限 kMaxRecordsPerFunc(超限丢弃最旧, 内存有界);
//       PrintAndReset() 输出统计后清空, 由调用方周期性触发(见 lidar_odometry Run)。
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace ct_lio {

/// 统计时间工具(全局静态记录)
class Timer {
 public:
  /// 每函数最多保留的记录条数(防止长时间运行内存线性膨胀)
  static constexpr size_t kMaxRecordsPerFunc = 500;

  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage_ms) : func_name(name) {
      time_usage_in_ms.emplace_back(time_usage_ms);
    }
    std::string func_name;
    std::vector<double> time_usage_in_ms;
  };

  /// 执行 func 并记录耗时(毫秒), 按 func_name 累积
  template <class F>
  static void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::steady_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::steady_clock::now();
    double time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
    auto it = records_.find(func_name);
    if (it != records_.end()) {
      it->second.time_usage_in_ms.push_back(time_used);
      // 超上限: 丢弃最旧(整体平移保持近端统计; 低频调用无感知)
      if (it->second.time_usage_in_ms.size() > kMaxRecordsPerFunc) {
        it->second.time_usage_in_ms.erase(it->second.time_usage_in_ms.begin());
      }
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  /// 输出各函数统计(次数/均值/最大/最小)到 stderr 并清空记录
  static void PrintAndReset() {
    for (const auto& [name, rec] : records_) {
      const auto& v = rec.time_usage_in_ms;
      if (v.empty()) continue;
      double sum = 0.0, maxv = v.front(), minv = v.front();
      for (double t : v) {
        sum += t;
        maxv = std::max(maxv, t);
        minv = std::min(minv, t);
      }
      std::cerr << "[Timer] " << name << ": n=" << v.size() << " avg=" << sum / v.size()
                << "ms max=" << maxv << "ms min=" << minv << "ms" << std::endl;
    }
    records_.clear();
  }

 private:
  static std::map<std::string, TimerRecord> records_;
};

}  // namespace ct_lio
