#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>

class OctoHandler : public rclcpp::Node
{
public:
    OctoHandler()
        : Node("load_local_octomap")
    {
        subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "octomap_full", 1, std::bind(&OctoHandler::octomapCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "move_group/monitored_planning_scene", 1);
        pub2_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 1);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(400),
            std::bind(&OctoHandler::timerCallback, this));
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        moveit_msgs::msg::PlanningSceneWorld psw;
        psw.octomap.header.stamp = this->get_clock()->now();
        psw.octomap.header.frame_id = "map";
        psw.octomap.octomap = *msg;

        psw.octomap.origin.position.x = 0;
        psw.octomap.origin.orientation.w = 1;

        moveit_msgs::msg::PlanningScene ps;
        ps.world = psw;
        ps.is_diff = true;

        map_msg_ = std::make_shared<moveit_msgs::msg::PlanningScene>(ps);
    }


    void timerCallback()
    {
        if (map_msg_)
        {
            pub_->publish(*map_msg_);
            pub2_->publish(*map_msg_);
        }
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit_msgs::msg::PlanningScene::SharedPtr map_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctoHandler>());
    rclcpp::shutdown();
    return 0;
}