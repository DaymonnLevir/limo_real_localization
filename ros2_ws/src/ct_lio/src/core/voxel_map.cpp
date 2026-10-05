// voxel_map.cpp - 哈希体素地图实现(具体策略见 voxel_map.hpp)
#include "ct_lio_ros2/core/voxel_map.hpp"

namespace ct_lio::core {

void VoxelMap::InsertPoint(const Eigen::Vector3d& point) {
  Voxel v = Voxel::FromPoint(point, voxel_size_);
  auto search = map_.find(v);

  if (search != map_.end()) {
    auto& block = search->second->second;
    if (!block->IsFull()) {
      // 距离去重: 仅当新点与体素内最近点距离超过 min_distance_points 才插入
      // 搜索上界取 10 倍体素尺寸(启发式, 覆盖 3x3x3 邻域的对角长度, 保证能找到最近点)
      const double kSearchRadiusFactor = 10.0;
      double sq_min = kSearchRadiusFactor * voxel_size_ * voxel_size_;
      for (const auto& p : block->Points()) {
        double sq = (p - point).squaredNorm();
        if (sq < sq_min) sq_min = sq;
      }
      if (sq_min > min_distance_points_ * min_distance_points_) {
        block->AddPoint(point);
      }
    }
    // LRU 更新: 访问过的体素移到链表头
    cache_.splice(cache_.begin(), cache_, search->second);
  } else {
    // 新体素: 创建块并插到链表头
    auto block = std::make_shared<VoxelBlock>(max_points_per_voxel_);
    block->AddPoint(point);
    cache_.push_front({v, block});
    map_.insert({v, cache_.begin()});

    // 超过容量: 淘汰链表尾(最久未使用的体素)
    while (map_.size() > static_cast<size_t>(capacity_)) {
      const Voxel& evict = cache_.back().first;
      map_.erase(evict);
      cache_.pop_back();
    }
  }
}

VoxelMap::BlockPtr VoxelMap::GetBlock(const Voxel& v) const {
  auto search = map_.find(v);
  if (search == map_.end()) return nullptr;
  return search->second->second;
}

}  // namespace ct_lio::core
