#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ClockPublisher : public rclcpp::Node
{
public:
    ClockPublisher()
    : Node("clock_publisher"), sim_time_(rclcpp::Time(0))
    {
        publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ClockPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        sim_time_ = sim_time_ + rclcpp::Duration(0, 100000000);  // 100ms
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = sim_time_.seconds();
        clock_msg.clock.nanosec = sim_time_.nanoseconds() %  1000000000ul;
        publisher_->publish(clock_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    rclcpp::Time sim_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockPublisher>());
    rclcpp::shutdown();
    return 0;
}