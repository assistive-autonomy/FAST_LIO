#include "fast_lio/headless.hpp"

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/message_definitions/local_message_definition_source.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/message_definition.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_interfaces/base_read_interface.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace fast_lio::headless
{
namespace
{

constexpr char kPathTopic[] = "/path";
constexpr char kRegisteredCloudTopic[] = "/cloud_registered";
constexpr char kTfTopic[] = "/tf";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kCdrFormat[] = "cdr";

std::string require_option_value(int argc, char ** argv, int & index, const std::string & option)
{
  if (index + 1 >= argc || std::string(argv[index + 1]).empty()) {
    throw std::invalid_argument("missing value for " + option);
  }
  return argv[++index];
}

void assign_once(std::string & destination, std::string value, const std::string & option)
{
  if (!destination.empty()) {
    throw std::invalid_argument(option + " was specified more than once");
  }
  if (value.empty()) {
    throw std::invalid_argument("empty value for " + option);
  }
  destination = std::move(value);
}

bool split_assignment(
  const std::string & argument, const std::string & option, std::string & value)
{
  const std::string prefix = option + "=";
  if (argument.rfind(prefix, 0) != 0) {
    return false;
  }
  value = argument.substr(prefix.size());
  return true;
}

std::filesystem::path normalized_path(const std::filesystem::path & path)
{
  std::error_code error;
  auto normalized = std::filesystem::weakly_canonical(path, error);
  if (!error) {
    return normalized;
  }
  error.clear();
  normalized = std::filesystem::absolute(path, error);
  if (error) {
    throw std::runtime_error(
            "could not resolve path '" + path.string() + "': " + error.message());
  }
  return normalized.lexically_normal();
}

void validate_paths(const Options & options)
{
  const std::filesystem::path input(options.input_uri);
  const std::filesystem::path output(options.output_uri);
  const std::filesystem::path config(options.config_path);

  std::error_code error;
  const auto input_status = std::filesystem::status(input, error);
  if (error || !std::filesystem::exists(input_status)) {
    throw std::runtime_error("input bag does not exist: " + input.string());
  }
  if (!std::filesystem::is_regular_file(input_status) &&
    !std::filesystem::is_directory(input_status))
  {
    throw std::runtime_error("input bag is not a file or directory: " + input.string());
  }

  error.clear();
  const auto output_status = std::filesystem::symlink_status(output, error);
  if (error && error != std::errc::no_such_file_or_directory) {
    throw std::runtime_error(
            "could not inspect output URI '" + output.string() + "': " + error.message());
  }
  if (!error && output_status.type() != std::filesystem::file_type::not_found) {
    throw std::runtime_error("output URI already exists: " + output.string());
  }

  if (normalized_path(input) == normalized_path(output)) {
    throw std::runtime_error("input and output URIs must be different");
  }

  auto output_parent = normalized_path(output).parent_path();
  error.clear();
  if (!std::filesystem::is_directory(output_parent, error) || error) {
    throw std::runtime_error(
            "output parent directory does not exist: " + output_parent.string());
  }

  error.clear();
  const auto config_status = std::filesystem::status(config, error);
  if (error || !std::filesystem::is_regular_file(config_status)) {
    throw std::runtime_error("config file does not exist: " + config.string());
  }
}

template<typename MessageT>
typename MessageT::UniquePtr deserialize_copy(
  const rosbag2_storage::SerializedBagMessage & source)
{
  if (!source.serialized_data) {
    throw std::runtime_error(
            "record on '" + source.topic_name + "' has no serialized payload");
  }

  // This constructor makes an owning copy. The reader-owned buffer is therefore never lent to
  // a deserializer or to FAST-LIO, and the exact object passed to Writer::write stays untouched.
  rclcpp::SerializedMessage serialized_copy(*source.serialized_data);
  auto message = std::make_unique<MessageT>();
  rclcpp::Serialization<MessageT> serializer;
  try {
    serializer.deserialize_message(&serialized_copy, message.get());
  } catch (const std::exception & error) {
    throw std::runtime_error(
            "could not deserialize '" + source.topic_name + "' at receive timestamp " +
            std::to_string(source.recv_timestamp) + ": " + error.what());
  }
  return message;
}

template<typename MessageT>
void write_generated(
  rosbag2_cpp::Writer & writer, const MessageT & message, const std::string & topic,
  rcutils_time_point_value_t recv_timestamp, rcutils_time_point_value_t send_timestamp)
{
  auto serialized = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<MessageT> serializer;
  serializer.serialize_message(&message, serialized.get());
  writer.write(
    serialized, topic, rosidl_generator_traits::name<MessageT>(), recv_timestamp, send_timestamp);
}

rosbag2_storage::TopicMetadata make_topic(
  const std::string & name, const std::string & type, const rclcpp::QoS & qos,
  const rosbag2_storage::MessageDefinition & definition)
{
  rosbag2_storage::TopicMetadata metadata;
  metadata.name = name;
  metadata.type = type;
  metadata.serialization_format = kCdrFormat;
  metadata.offered_qos_profiles.push_back(qos);
  metadata.type_description_hash = definition.type_hash;
  return metadata;
}

const rosbag2_storage::MessageDefinition & find_definition(
  const std::unordered_map<std::string, rosbag2_storage::MessageDefinition> & definitions,
  const std::string & type, rosbag2_storage::MessageDefinition & empty_fallback)
{
  const auto definition = definitions.find(type);
  if (definition != definitions.end()) {
    return definition->second;
  }
  empty_fallback = rosbag2_storage::MessageDefinition::empty_message_definition_for(type);
  return empty_fallback;
}

void require_topic_type(
  const std::unordered_map<std::string, rosbag2_storage::TopicMetadata> & topics,
  const std::string & topic, const std::string & expected_type)
{
  const auto found = topics.find(topic);
  if (found == topics.end()) {
    throw std::runtime_error("required input topic is missing: " + topic);
  }
  if (found->second.type != expected_type) {
    throw std::runtime_error(
            "topic '" + topic + "' has type '" + found->second.type +
            "', expected '" + expected_type + "'");
  }
  if (found->second.serialization_format != kCdrFormat) {
    throw std::runtime_error(
            "topic '" + topic + "' uses serialization format '" +
            found->second.serialization_format + "', expected 'cdr'");
  }
}

void require_generated_tf_compatibility(
  const std::unordered_map<std::string, rosbag2_storage::TopicMetadata> & topics,
  const std::string & topic)
{
  const auto found = topics.find(topic);
  if (found == topics.end()) {
    return;
  }
  const std::string tf_type = rosidl_generator_traits::name<tf2_msgs::msg::TFMessage>();
  if (found->second.type != tf_type || found->second.serialization_format != kCdrFormat) {
    throw std::runtime_error(
            "cannot append generated transforms to '" + topic + "' (found type '" +
            found->second.type + "' and serialization format '" +
            found->second.serialization_format + "')");
  }
}

bool stamp_is_zero(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

bool same_serialized_record(
  const rosbag2_storage::SerializedBagMessage & expected,
  const rosbag2_storage::SerializedBagMessage & actual)
{
  if (expected.topic_name != actual.topic_name ||
    expected.recv_timestamp != actual.recv_timestamp ||
    expected.send_timestamp != actual.send_timestamp ||
    !expected.serialized_data || !actual.serialized_data)
  {
    return false;
  }

  const auto & expected_data = *expected.serialized_data;
  const auto & actual_data = *actual.serialized_data;
  return expected_data.buffer_length == actual_data.buffer_length &&
         (expected_data.buffer_length == 0U ||
         std::memcmp(
           expected_data.buffer, actual_data.buffer, expected_data.buffer_length) == 0);
}

struct PassthroughVerification
{
  std::uint64_t original_records = 0;
  std::uint64_t generated_records = 0;
};

PassthroughVerification verify_passthrough(
  const std::string & input_uri, const std::string & output_uri,
  std::uint64_t expected_original_records)
{
  rosbag2_storage::StorageOptions input_storage;
  input_storage.uri = input_uri;
  input_storage.storage_id = "mcap";
  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = output_uri;
  output_storage.storage_id = "mcap";

  rosbag2_cpp::Reader input_reader;
  rosbag2_cpp::Reader output_reader;
  input_reader.open(input_storage, rosbag2_cpp::ConverterOptions{});
  output_reader.open(output_storage, rosbag2_cpp::ConverterOptions{});
  const rosbag2_storage::ReadOrder file_order(
    rosbag2_storage::ReadOrder::File, false);
  if (!input_reader.set_read_order(file_order) || !output_reader.set_read_order(file_order)) {
    throw std::runtime_error("could not verify preservation in original MCAP file order");
  }

  PassthroughVerification verification;
  while (input_reader.has_next()) {
    const auto expected = input_reader.read_next();
    bool matched = false;
    while (output_reader.has_next()) {
      const auto actual = output_reader.read_next();
      if (same_serialized_record(*expected, *actual)) {
        matched = true;
        ++verification.original_records;
        break;
      }
      ++verification.generated_records;
    }
    if (!matched) {
      throw std::runtime_error(
              "post-write preservation check failed before input record " +
              std::to_string(verification.original_records + 1U) +
              " on topic '" + expected->topic_name + "'");
    }
  }
  while (output_reader.has_next()) {
    (void)output_reader.read_next();
    ++verification.generated_records;
  }
  output_reader.close();
  input_reader.close();

  if (verification.original_records != expected_original_records) {
    throw std::runtime_error(
            "post-write preservation count mismatch: expected " +
            std::to_string(expected_original_records) + ", verified " +
            std::to_string(verification.original_records));
  }
  return verification;
}

}  // namespace

void print_usage(const char * executable)
{
  std::cout << "Usage: " << executable
            << " --input <bag.mcap|bag_directory> --output <new_bag_directory>"
            << " --config <fast_lio_params.yaml>\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "-h" || argument == "--help") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }

    std::string assigned_value;
    if (argument == "--input") {
      assign_once(
        options.input_uri, require_option_value(argc, argv, index, argument), argument);
    } else if (split_assignment(argument, "--input", assigned_value)) {
      assign_once(options.input_uri, std::move(assigned_value), "--input");
    } else if (argument == "--output") {
      assign_once(
        options.output_uri, require_option_value(argc, argv, index, argument), argument);
    } else if (split_assignment(argument, "--output", assigned_value)) {
      assign_once(options.output_uri, std::move(assigned_value), "--output");
    } else if (argument == "--config") {
      assign_once(
        options.config_path, require_option_value(argc, argv, index, argument), argument);
    } else if (split_assignment(argument, "--config", assigned_value)) {
      assign_once(options.config_path, std::move(assigned_value), "--config");
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }

  if (options.input_uri.empty()) {
    throw std::invalid_argument("--input is required");
  }
  if (options.output_uri.empty()) {
    throw std::invalid_argument("--output is required");
  }
  if (options.config_path.empty()) {
    throw std::invalid_argument("--config is required");
  }
  return options;
}

int transform_bag(const Options & options, SlamEngine & engine)
{
  validate_paths(options);

  rosbag2_cpp::Reader reader;
  rosbag2_cpp::Writer writer;
  bool reader_open = false;
  bool writer_open = false;

  std::uint64_t copied_records = 0;
  std::uint64_t lidar_records = 0;
  std::uint64_t imu_records = 0;
  std::uint64_t solution_records = 0;
  std::uint64_t bridge_records = 0;
  PassthroughVerification verification;
  std::unordered_map<std::string, std::uint64_t> copied_by_topic;

  try {
    rosbag2_storage::StorageOptions input_storage;
    input_storage.uri = options.input_uri;
    input_storage.storage_id = "mcap";
    reader.open(input_storage, rosbag2_cpp::ConverterOptions{});
    reader_open = true;

    if (!reader.set_read_order(
        rosbag2_storage::ReadOrder(rosbag2_storage::ReadOrder::File, false)))
    {
      throw std::runtime_error("the MCAP reader cannot provide original file order");
    }

    const auto input_metadata = reader.get_metadata();
    auto input_topics = reader.get_all_topics_and_types();
    std::unordered_map<std::string, rosbag2_storage::TopicMetadata> topics_by_name;
    for (const auto & topic : input_topics) {
      const auto inserted = topics_by_name.emplace(topic.name, topic);
      if (!inserted.second && inserted.first->second.type != topic.type) {
        throw std::runtime_error("input bag contains conflicting metadata for " + topic.name);
      }
    }

    // Metadata retains channels even when they have no messages. Supplement the storage query so
    // those channels are also created in the output bag.
    for (const auto & information : input_metadata.topics_with_message_count) {
      if (topics_by_name.emplace(
          information.topic_metadata.name, information.topic_metadata).second)
      {
        input_topics.push_back(information.topic_metadata);
      }
    }

    if (topics_by_name.count(kPathTopic) != 0U) {
      throw std::runtime_error(
              "input already contains /path; refusing to mix an existing trajectory with FAST-LIO");
    }
    if (topics_by_name.count(kRegisteredCloudTopic) != 0U) {
      throw std::runtime_error(
              "input already contains /cloud_registered; refusing to mix registered scans");
    }

    const std::string lidar_type = engine.uses_livox_custom_message() ?
      rosidl_generator_traits::name<livox_ros_driver2::msg::CustomMsg>() :
      rosidl_generator_traits::name<sensor_msgs::msg::PointCloud2>();
    const std::string imu_type = rosidl_generator_traits::name<sensor_msgs::msg::Imu>();
    require_topic_type(topics_by_name, engine.lidar_topic(), lidar_type);
    require_topic_type(topics_by_name, engine.imu_topic(), imu_type);
    require_generated_tf_compatibility(topics_by_name, kTfTopic);
    require_generated_tf_compatibility(topics_by_name, kTfStaticTopic);

    std::unordered_map<std::string, std::uint64_t> expected_by_topic;
    for (const auto & information : input_metadata.topics_with_message_count) {
      expected_by_topic[information.topic_metadata.name] += information.message_count;
    }
    for (const auto & topic : input_topics) {
      expected_by_topic.try_emplace(topic.name, 0U);
    }
    if (expected_by_topic[engine.lidar_topic()] == 0U) {
      throw std::runtime_error("selected lidar topic contains no messages: " + engine.lidar_topic());
    }
    if (expected_by_topic[engine.imu_topic()] == 0U) {
      throw std::runtime_error("selected IMU topic contains no messages: " + engine.imu_topic());
    }

    std::vector<rosbag2_storage::MessageDefinition> input_definitions;
    reader.get_all_message_definitions(input_definitions);
    std::unordered_map<std::string, rosbag2_storage::MessageDefinition> definitions_by_type;
    for (auto & definition : input_definitions) {
      definitions_by_type.try_emplace(definition.topic_type, std::move(definition));
    }

    rosbag2_cpp::LocalMessageDefinitionSource local_definitions;
    const std::string tf_type = rosidl_generator_traits::name<tf2_msgs::msg::TFMessage>();
    const std::string path_type = rosidl_generator_traits::name<nav_msgs::msg::Path>();
    const std::string cloud_type =
      rosidl_generator_traits::name<sensor_msgs::msg::PointCloud2>();
    const auto tf_definition = local_definitions.get_full_text(tf_type);
    const auto path_definition = local_definitions.get_full_text(path_type);
    const auto cloud_definition = local_definitions.get_full_text(cloud_type);

    rosbag2_storage::StorageOptions output_storage;
    output_storage.uri = options.output_uri;
    output_storage.storage_id = "mcap";
    output_storage.max_cache_size = 0;
    writer.open(output_storage, rosbag2_cpp::ConverterOptions{});
    writer_open = true;

    for (const auto & topic : input_topics) {
      rosbag2_storage::MessageDefinition empty_definition;
      writer.create_topic(
        topic, find_definition(definitions_by_type, topic.type, empty_definition));
    }

    if (topics_by_name.count(kTfTopic) == 0U) {
      rclcpp::QoS qos(rclcpp::KeepLast(100));
      qos.reliable().durability_volatile();
      writer.create_topic(make_topic(kTfTopic, tf_type, qos, tf_definition), tf_definition);
    }
    if (topics_by_name.count(kTfStaticTopic) == 0U) {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable().transient_local();
      writer.create_topic(
        make_topic(kTfStaticTopic, tf_type, qos, tf_definition), tf_definition);
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable().durability_volatile();
      writer.create_topic(make_topic(kPathTopic, path_type, qos, path_definition), path_definition);
    }
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort().durability_volatile();
      writer.create_topic(
        make_topic(kRegisteredCloudTopic, cloud_type, qos, cloud_definition), cloud_definition);
    }

    tf2::BufferCore static_tf_buffer;
    bool bridge_written = false;
    nav_msgs::msg::Path cumulative_path;
    sensor_msgs::msg::PointCloud2 final_registered_cloud;
    rcutils_time_point_value_t final_solution_recv_timestamp = 0;
    rcutils_time_point_value_t final_solution_send_timestamp = 0;

    std::cout << "Reading " << input_metadata.message_count
              << " input records in original MCAP file order..." << std::endl;

    while (reader.has_next()) {
      auto source = reader.read_next();
      if (!source) {
        throw std::runtime_error("MCAP reader returned a null record");
      }
      if (topics_by_name.count(source->topic_name) == 0U) {
        throw std::runtime_error(
                "record refers to a topic without metadata: " + source->topic_name);
      }

      // This exact reader-owned object is written first. In particular, its serialized payload,
      // receive timestamp, send timestamp, topic name, and position relative to every other input
      // record are not modified.
      writer.write(source);
      ++copied_records;
      ++copied_by_topic[source->topic_name];

      if (source->topic_name == kTfStaticTopic) {
        auto tf_message = deserialize_copy<tf2_msgs::msg::TFMessage>(*source);
        for (const auto & transform : tf_message->transforms) {
          if (!static_tf_buffer.setTransform(transform, "fastlio_headless_input", true)) {
            std::cerr << "Warning: ignored invalid static transform "
                      << transform.header.frame_id << " -> " << transform.child_frame_id
                      << " at receive timestamp " << source->recv_timestamp << std::endl;
          }
        }

        if (engine.sensor_tree_bridge_enabled() && !bridge_written) {
          try {
            auto bridge = static_tf_buffer.lookupTransform(
              engine.imu_frame(), engine.base_frame(), tf2::TimePointZero);
            bridge.header.frame_id = "body";
            bridge.child_frame_id = engine.base_frame();
            if (stamp_is_zero(bridge.header.stamp)) {
              bridge.header.stamp = static_cast<builtin_interfaces::msg::Time>(
                rclcpp::Time(source->recv_timestamp, RCL_ROS_TIME));
            }

            tf2_msgs::msg::TFMessage generated_bridge;
            generated_bridge.transforms.push_back(std::move(bridge));
            write_generated(
              writer, generated_bridge, kTfStaticTopic,
              source->recv_timestamp, source->send_timestamp);
            bridge_written = true;
            ++bridge_records;
          } catch (const tf2::TransformException &) {
            // A later /tf_static record may complete the chain.
          }
        }
      }

      bool sensor_record = false;
      if (source->topic_name == engine.lidar_topic()) {
        sensor_record = true;
        if (engine.uses_livox_custom_message()) {
          engine.feed_lidar(deserialize_copy<livox_ros_driver2::msg::CustomMsg>(*source));
        } else {
          engine.feed_lidar(deserialize_copy<sensor_msgs::msg::PointCloud2>(*source));
        }
        ++lidar_records;
      } else if (source->topic_name == engine.imu_topic()) {
        sensor_record = true;
        engine.feed_imu(deserialize_copy<sensor_msgs::msg::Imu>(*source));
        ++imu_records;
      }

      if (!sensor_record) {
        continue;
      }

      while (true) {
        SlamResult solution;
        const auto status = engine.process_next(solution);
        if (status == ProcessStatus::waiting) {
          break;
        }
        if (status == ProcessStatus::consumed) {
          continue;
        }

        ++solution_records;
        cumulative_path.header.frame_id = solution.pose.header.frame_id;
        cumulative_path.header.stamp = solution.pose.header.stamp;
        cumulative_path.poses.push_back(solution.pose);

        tf2_msgs::msg::TFMessage tf_message;
        tf_message.transforms.push_back(std::move(solution.map_to_body));
        write_generated(
          writer, tf_message, kTfTopic, source->recv_timestamp, source->send_timestamp);
        final_registered_cloud = std::move(solution.registered_cloud);
        final_solution_recv_timestamp = source->recv_timestamp;
        final_solution_send_timestamp = source->send_timestamp;
      }
    }

    if (copied_records != input_metadata.message_count) {
      throw std::runtime_error(
              "input-copy validation failed: metadata reports " +
              std::to_string(input_metadata.message_count) + " records, copied " +
              std::to_string(copied_records));
    }
    for (const auto & expected : expected_by_topic) {
      if (copied_by_topic[expected.first] != expected.second) {
        throw std::runtime_error(
                "input-copy validation failed for '" + expected.first + "': expected " +
                std::to_string(expected.second) + ", copied " +
                std::to_string(copied_by_topic[expected.first]));
      }
    }
    if (lidar_records == 0U || imu_records == 0U) {
      throw std::runtime_error("no selected lidar or IMU records were fed to FAST-LIO");
    }
    if (solution_records == 0U) {
      throw std::runtime_error("FAST-LIO produced no SLAM solution");
    }
    if (engine.sensor_tree_bridge_enabled() && !bridge_written) {
      throw std::runtime_error(
              "could not build the configured static transform chain from '" +
              engine.imu_frame() + "' to '" + engine.base_frame() + "'");
    }

    // The result bag intentionally contains one cumulative trajectory and one final registered
    // scan. Their ROS header stamps remain the last solved LiDAR scan time; their bag timestamps
    // are the receive/send timestamps of the source record that produced that solution.
    write_generated(
      writer, final_registered_cloud, kRegisteredCloudTopic,
      final_solution_recv_timestamp, final_solution_send_timestamp);
    write_generated(
      writer, cumulative_path, kPathTopic,
      final_solution_recv_timestamp, final_solution_send_timestamp);

    writer.close();
    writer_open = false;
    reader.close();
    reader_open = false;
    verification = verify_passthrough(
      options.input_uri, options.output_uri, input_metadata.message_count);
  } catch (...) {
    if (writer_open) {
      try {
        writer.close();
      } catch (...) {
      }
    }
    if (reader_open) {
      try {
        reader.close();
      } catch (...) {
      }
    }
    throw;
  }

  std::cout << "Headless FAST-LIO bag transform complete\n"
            << "  copied input records: " << copied_records
            << " (serialized payloads and receive/send timestamps unchanged)\n"
            << "  verified input records: " << verification.original_records
            << " (exact topic, payload, receive/send timestamp, and input order)\n"
            << "  lidar records fed:   " << lidar_records << "\n"
            << "  IMU records fed:     " << imu_records << "\n"
            << "  SLAM solutions:      " << solution_records << "\n"
            << "  generated /tf:       " << solution_records << "\n"
            << "  generated /path:     1 (cumulative)\n"
            << "  registered scans:    1 (final)\n"
            << "  generated /tf_static:" << bridge_records << "\n"
            << "  all generated records: " << verification.generated_records << "\n"
            << "  output: " << options.output_uri << std::endl;
  return 0;
}

}  // namespace fast_lio::headless
