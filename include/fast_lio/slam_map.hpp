#ifndef FAST_LIO__SLAM_MAP_HPP_
#define FAST_LIO__SLAM_MAP_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_lio
{

class SlamMap
{
public:
  using Point = pcl::PointXYZI;
  using PointCloud = pcl::PointCloud<Point>;

  explicit SlamMap(double voxel_size)
  : inverse_voxel_size_(1.0 / voxel_size)
  {
  }

  template<typename PointT>
  void add(const pcl::PointCloud<PointT> & cloud)
  {
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity))
      {
        continue;
      }

      const VoxelKey key{
        voxel_coordinate(point.x),
        voxel_coordinate(point.y),
        voxel_coordinate(point.z)};
      auto & voxel = voxels_[key];
      voxel.sum_x += point.x;
      voxel.sum_y += point.y;
      voxel.sum_z += point.z;
      voxel.sum_intensity += point.intensity;
      ++voxel.count;
    }
  }

  std::size_t size() const
  {
    return voxels_.size();
  }

  PointCloud::Ptr snapshot() const
  {
    const std::size_t max_serializable_points =
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) /
      sizeof(Point);
    if (voxels_.size() > max_serializable_points) {
      throw std::runtime_error(
              "the /slam cloud is too large for PointCloud2; increase "
              "publish.slam_voxel_size");
    }

    auto cloud = std::make_shared<PointCloud>();
    cloud->points.reserve(voxels_.size());

    for (const auto & entry : voxels_) {
      const auto & voxel = entry.second;
      const double inverse_count = 1.0 / static_cast<double>(voxel.count);
      Point point{};
      point.x = static_cast<float>(voxel.sum_x * inverse_count);
      point.y = static_cast<float>(voxel.sum_y * inverse_count);
      point.z = static_cast<float>(voxel.sum_z * inverse_count);
      point.intensity = static_cast<float>(voxel.sum_intensity * inverse_count);
      cloud->points.push_back(point);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
  }

private:
  struct VoxelKey
  {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t seed = std::hash<std::int64_t>{}(key.x);
      combine(seed, std::hash<std::int64_t>{}(key.y));
      combine(seed, std::hash<std::int64_t>{}(key.z));
      return seed;
    }

  private:
    static void combine(std::size_t & seed, std::size_t value)
    {
      seed ^= value + static_cast<std::size_t>(0x9e3779b9U) +
        (seed << 6U) + (seed >> 2U);
    }
  };

  struct Voxel
  {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_intensity = 0.0;
    std::uint64_t count = 0U;
  };

  std::int64_t voxel_coordinate(float coordinate) const
  {
    return static_cast<std::int64_t>(
      std::floor(static_cast<double>(coordinate) * inverse_voxel_size_));
  }

  double inverse_voxel_size_;
  std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
};

}  // namespace fast_lio

#endif  // FAST_LIO__SLAM_MAP_HPP_
