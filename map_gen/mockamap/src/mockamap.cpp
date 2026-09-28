#include "maps.hpp"
#include <cmath>
#include <chrono>
#include <stdexcept>

class MapNode : public rclcpp::Node {
 public:
  MapNode() : Node("mockamap_node") {
    int seed, sx, sy, sz, type;
    double resolution, frequency;
    mocka::readParam(this, "seed", seed, 1024);
    mocka::readParam(this, "type", type, 1);
    mocka::readParam(this, "x_length", sx, 50);
    mocka::readParam(this, "y_length", sy, 50);
    mocka::readParam(this, "z_length", sz, 5);
    mocka::readParam(this, "resolution", resolution, 0.25);
    mocka::readParam(this, "update_freq", frequency, 1.0);
    if (!std::isfinite(resolution) || resolution <= 0 || !std::isfinite(frequency) || frequency <= 0 ||
        sx <= 0 || sy <= 0 || sz <= 0 || type < 1 || type > 4 ||
        std::min({sx, sy, sz}) / resolution < 2 ||
        double(sx)*sy*sz/std::pow(resolution, 3) > 20000000)
      throw std::invalid_argument("Invalid map dimensions, resolution, frequency or type");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", rclcpp::QoS(1).reliable().transient_local());
    mocka::Maps::BasicInfo info{};
    info.nh_private = this;
    info.seed = seed; info.scale = 1.0/resolution;
    info.sizeX = int(sx/resolution); info.sizeY = int(sy/resolution); info.sizeZ = int(sz/resolution);
    info.output = &message_; info.cloud = &cloud_;
    mocka::Maps map; map.setInfo(info); map.generate(type);
    auto publish = [this]() { message_.header.stamp = now(); publisher_->publish(message_); };
    publish();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/frequency), publish);
    RCLCPP_INFO(get_logger(), "Map ready: type=%d points=%u frame=odom", type, message_.width * message_.height);
  }
 private:
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<MapNode>()); }
  catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("mockamap"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
