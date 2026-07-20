#ifndef FAST_LIO__HEADLESS_HPP_
#define FAST_LIO__HEADLESS_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fast_lio::headless
{

struct Options
{
  std::string input_uri;
  std::string output_uri;
  std::string config_path;
};

enum class ProcessStatus
{
  waiting,
  consumed,
  solution,
};

struct SlamResult
{
  geometry_msgs::msg::TransformStamped map_to_body;
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::PointCloud2 registered_cloud;
};

class SlamEngine
{
public:
  virtual ~SlamEngine() = default;

  virtual const std::string & lidar_topic() const = 0;
  virtual const std::string & imu_topic() const = 0;
  virtual const std::string & imu_frame() const = 0;
  virtual const std::string & base_frame() const = 0;
  virtual bool uses_livox_custom_message() const = 0;
  virtual bool sensor_tree_bridge_enabled() const = 0;

  virtual void feed_lidar(sensor_msgs::msg::PointCloud2::UniquePtr message) = 0;
  virtual void feed_lidar(livox_ros_driver2::msg::CustomMsg::UniquePtr message) = 0;
  virtual void feed_imu(sensor_msgs::msg::Imu::UniquePtr message) = 0;
  virtual ProcessStatus process_next(SlamResult & result) = 0;
};

Options parse_options(int argc, char ** argv);
int transform_bag(const Options & options, SlamEngine & engine);
void print_usage(const char * executable);

}  // namespace fast_lio::headless

#endif  // FAST_LIO__HEADLESS_HPP_
