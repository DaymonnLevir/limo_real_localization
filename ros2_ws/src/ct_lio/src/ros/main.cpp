// main.cpp - 节点入口
// 职责: init → 构造节点 → 校验启动成功 → spin → shutdown
// 启动失败(参数非法/核心 Initialize 失败)时退出码非零, 供 launch/systemd 区分
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ct_lio_ros2/ros/lio_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 构造可能抛异常(参数声明类型不匹配/资源分配失败), 捕获后打日志退出非零
  std::shared_ptr<ct_lio::LioNode> node;
  try {
    node = std::make_shared<ct_lio::LioNode>();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ct_lio_node"), "node construction failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // 启动校验: Initialize 失败(参数非法等)时节点未完成初始化, 退出非零
  if (!node->IsInitialized()) {
    RCLCPP_ERROR(rclcpp::get_logger("ct_lio_node"),
                 "LioNode failed to initialize (check params/config), exiting");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
