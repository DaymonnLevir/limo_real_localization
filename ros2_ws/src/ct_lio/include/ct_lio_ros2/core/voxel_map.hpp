// voxel_map.hpp - 哈希体素地图 + LRU 淘汰
//
// 职责: 维护局部点云地图, 支持按体素访问与有限容量下的自动淘汰.
// 结构: 以 tsl::robin_map 建立 体素坐标 -> LRU 链表迭代器 的索引,
//       配合 std::list 维护访问顺序(LRU 序).
// 策略: 访问过的体素 splice 到链表头(最近使用), 超过容量时删除链表尾
//       (最久未使用).
// 参考: NV-LIO 的 voxelHashMap2 实现.
#pragma once

#include <list>
#include <memory>
#include <vector>

#include <tsl/robin_map.h>

#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

// ---------------------------------------------------------------------------
// 体素坐标与哈希
// ---------------------------------------------------------------------------
/// 体素整数坐标
struct Voxel {
  short x = 0, y = 0, z = 0;
  Voxel() = default;
  Voxel(short _x, short _y, short _z) : x(_x), y(_y), z(_z) {}
  bool operator==(const Voxel& v) const { return x == v.x && y == v.y && z == v.z; }
  static Voxel FromPoint(const Eigen::Vector3d& point, double voxel_size) {
    return {static_cast<short>(point.x() / voxel_size),
            static_cast<short>(point.y() / voxel_size),
            static_cast<short>(point.z() / voxel_size)};
  }
};

}  // namespace ct_lio::core

// voxel 哈希特化(必须在 robin_map 实例化之前)
namespace std {
template <>
struct hash<ct_lio::core::Voxel> {
  std::size_t operator()(const ct_lio::core::Voxel& v) const {
    const size_t kP1 = 73856093;
    const size_t kP2 = 19349669;
    const size_t kP3 = 83492791;
    return v.x * kP1 + v.y * kP2 + v.z * kP3;
  }
};
}  // namespace std

namespace ct_lio::core {

// ---------------------------------------------------------------------------
// 体素块
// ---------------------------------------------------------------------------
/// 体素块: 定容点列表(按最大容量分配; 已简化为纯点列表, 不再维护增量 centroid/cov)
class VoxelBlock {
 public:
  explicit VoxelBlock(int capacity = 20) : capacity_(capacity) {
    points_.reserve(capacity_);
  }

  bool IsFull() const { return static_cast<int>(points_.size()) >= capacity_; }
  int NumPoints() const { return static_cast<int>(points_.size()); }
  const std::vector<Eigen::Vector3d>& Points() const { return points_; }

  /// 插入一个点
  void AddPoint(const Eigen::Vector3d& p) { points_.push_back(p); }

 private:
  int capacity_;
  std::vector<Eigen::Vector3d> points_;
};

// ---------------------------------------------------------------------------
// 体素地图(LRU 淘汰)
// ---------------------------------------------------------------------------
/// 哈希体素地图 + LRU 淘汰
class VoxelMap {
 public:
  using BlockPtr = std::shared_ptr<VoxelBlock>;
  using CacheEntry = std::pair<Voxel, BlockPtr>;
  using CacheList = std::list<CacheEntry>;

  explicit VoxelMap(int capacity = 1000000, double voxel_size = 0.2, int max_points_per_voxel = 20,
                    double min_distance_points = 0.05)
      : capacity_(capacity), voxel_size_(voxel_size), max_points_per_voxel_(max_points_per_voxel),
        min_distance_points_(min_distance_points) {}

  /// 插入点(含距离去重与容量控制)
  void InsertPoint(const Eigen::Vector3d& point);

  /// 获取指定体素块, 不存在时返回空指针
  BlockPtr GetBlock(const Voxel& v) const;

  /// 遍历所有体素块(供邻域搜索/地图裁剪使用)
  template <typename F>
  void ForEachBlock(F&& func) {
    for (auto& [voxel, it] : map_) {
      func(voxel, *(it->second));
    }
  }

  /// 删除指定体素(地图裁剪使用)
  void EraseVoxel(const Voxel& v) {
    auto search = map_.find(v);
    if (search != map_.end()) {
      cache_.erase(search->second);
      map_.erase(search);
    }
  }

 private:
  int capacity_;
  double voxel_size_;
  int max_points_per_voxel_;
  double min_distance_points_;

  using MapType = tsl::robin_map<Voxel, typename CacheList::iterator>;
  MapType map_;
  CacheList cache_;
};

}  // namespace ct_lio::core
