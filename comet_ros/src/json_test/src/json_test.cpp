#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include "utils/PID.h"

using std::placeholders::_1;

class JsonTest : public rclcpp::Node
{
  public:
    JsonTest()
    : Node("json_test")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "serial_output", 10, std::bind(&JsonTest::topic_callback, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_commands", 10);
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        // Process the received message
        int odom_value = 0, motor_value = 0;
        try {
            auto json_msg = nlohmann::json::parse(msg->data);
            if (json_msg.contains("odom")) {
                odom_value = json_msg["odom"];
                RCLCPP_INFO(this->get_logger(), "Odom Value: %d", odom_value);
            } else {
                RCLCPP_WARN(this->get_logger(), "Received JSON does not contain 'odom' field.");
            }

            if (json_msg.contains("motor")) {
                motor_value = json_msg["motor"];
                RCLCPP_INFO(this->get_logger(), "Motor Value: %d", motor_value);
            } else {
                RCLCPP_WARN(this->get_logger(), "Received JSON does not contain 'motor' field.");
            }

            int output = pid.update(odom_value, motor_value);
            RCLCPP_INFO(this->get_logger(), "PID Output: %d", output);

            nlohmann::json output_json = nlohmann::json::object();
            output_json["motor"] = output;

            // Publish PID output
            auto output_msg = std_msgs::msg::String();
            output_msg.data = output_json.dump();
            publisher_->publish(output_msg);
        } catch (nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    PID pid{30.0, 0.0, 0.1};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JsonTest>());
  rclcpp::shutdown();
  return 0;
}