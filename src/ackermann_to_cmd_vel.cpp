#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AckermannToTwistConverter : public rclcpp::Node
{
public:
    AckermannToTwistConverter() : Node("ackermann_to_twist_converter_node")
    {
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackermann_cmd", 
            10, 
            std::bind(&AckermannToTwistConverter::ackermannCallback, this, std::placeholders::_1)
        );
    }

private:
    void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist_msg;

        // Convert Ackermann steering_angle to Twist angular.z
        twist_msg.angular.z = msg->drive.steering_angle;

        // Convert Ackermann speed to Twist linear.x
        twist_msg.linear.x = msg->drive.speed;

        twist_publisher_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_subscriber_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToTwistConverter>());
    rclcpp::shutdown();
    return 0;
}
