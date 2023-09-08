#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
using std::placeholders::_1;

class PointFetcher : public rclcpp::Node
{
public:
  PointFetcher() : Node("pc_point_fetcher")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&PointFetcher::pointFetcherCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Started listening to /clicked_point topic...");
  }

private:
  void pointFetcherCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point: x = %f, y = %f, z = %f",
                msg->point.x, msg->point.y, msg->point.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointFetcher>());
  rclcpp::shutdown();
  return 0;
}