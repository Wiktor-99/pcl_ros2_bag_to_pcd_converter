#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <string_view>

const std::string executable_name{"pcl_bag_to_pcd"};

class BagReader {
  using PointCloud = sensor_msgs::msg::PointCloud2;
public:
  BagReader(std::string bag_name, std::string topic_name) : bag_name_{bag_name}, topic_name_{topic_name} {
    reader_.open(bag_name_);
  }

  std::string make_output_path(const PointCloud& point_cloud_msg) const {
    std::stringstream output_path;
    output_path << output_path_folder_ << point_cloud_msg.header.stamp.sec << point_cloud_msg.header.stamp.nanosec << pcd_extention_;
    return output_path.str();
  }

  PointCloud deserialize_point_cloud_msg_message(const rosbag2_storage::SerializedBagMessage& point_cloud_msg) const {
    rclcpp::SerializedMessage serialized_msg{*point_cloud_msg.serialized_data};
    PointCloud output_point_cloud_msg{};
    serialization_.deserialize_message(&serialized_msg, &output_point_cloud_msg);
    return output_point_cloud_msg;
  }

  void save_message_in_pdc_format(const rosbag2_storage::SerializedBagMessage& serialized_point_cloud_msg) const {
    const auto point_cloud_msg{deserialize_point_cloud_msg_message(serialized_point_cloud_msg)};
    std::string output_path{make_output_path(point_cloud_msg)};
    constexpr bool save_using_binary_mode{true};
    RCLCPP_INFO_STREAM(rclcpp::get_logger(executable_name), "Message saved in " << output_path);
    pcl::io::savePCDFile(
    output_path, point_cloud_msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), save_using_binary_mode);
  }

  void save_point_cloud_massages() {
    while (reader_.has_next()) {
      auto serialized_point_cloud_msg = reader_.read_next();
      if (serialized_point_cloud_msg->topic_name != topic_name_) {
        continue;
      }
      save_message_in_pdc_format(*serialized_point_cloud_msg);
    }
  }

  rclcpp::Serialization<PointCloud> serialization_;
  rosbag2_cpp::Reader reader_;
  const std::string bag_name_;
  const std::string topic_name_;
  const std::string output_path_folder_{"/app/pcd/"};
  const std::string pcd_extention_{".pcd"};
};

int main(int argc, char ** argv)
{
  if (argc != 3) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(executable_name),
      "Arguments are missing. Usage: ros2 run <bag> <topic_name>");
    return 1;
  }

  rclcpp::init(argc, argv);
  BagReader bag_reader{argv[1], argv[2]};
  bag_reader.save_point_cloud_massages();
  rclcpp::shutdown();

  return 0;
}