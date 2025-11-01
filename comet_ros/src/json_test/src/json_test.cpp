#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <nlohmann/json.hpp>
#include "utils/PID.h"
#include "msgs/message_generated.h"
#include "msgs/response_generated.h"

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
        odom_publisher_ = this->create_publisher<std_msgs::msg::Int32>("odom", 10);
        motor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("motor", 10);
    }

  private:
    std::vector<uint8_t> buildCommand(int voltage) {
        flatbuffers::FlatBufferBuilder builder(1024);
        auto motor = messages::CreateResponse(builder, voltage);
        builder.Finish(motor);

        uint8_t* buf = builder.GetBufferPointer();
        int size = builder.GetSize();
        return std::vector<uint8_t>(buf, buf + size);
    }

    std::string toHex(const std::vector<uint8_t>& data) {
        std::ostringstream oss;
        for (auto b : data)
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
        return oss.str();
    }

    std::vector<uint8_t> fromHex(const std::string& hex) {
        std::vector<uint8_t> data;
        data.reserve(hex.size() / 2);
        for (size_t i = 0; i < hex.size(); i += 2) {
            if (i + 1 >= hex.size()) break;  // odd length safety
            uint8_t byte = static_cast<uint8_t>(strtol(hex.substr(i, 2).c_str(), nullptr, 16));
            data.push_back(byte);
        }
        return data;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        // decode hex string
        std::vector<uint8_t> data = fromHex(msg->data);

        // verify and parse flatbuffer
        auto verifier = flatbuffers::Verifier(data.data(), data.size());
        if (!messages::VerifyCommandBuffer(verifier)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid flatbuffer message");
            RCLCPP_ERROR(this->get_logger(), "Data size: %zu", data.size());
            RCLCPP_ERROR(this->get_logger(), "Hex: %s", msg->data.c_str());
            RCLCPP_ERROR(this->get_logger(), "Buffer: %s", toHex(data).c_str());
            return;
        }

        auto cmd = messages::GetCommand(data.data());
        // RCLCPP_INFO(this->get_logger(), "Odom: %d, Motor: %d", cmd->odom(), cmd->motor());

        // run PID controller
        int output = pid.update(cmd->odom(), cmd->motor());
        // RCLCPP_INFO(this->get_logger(), "PID output: %d", output);

        // create response flatbuffer
        auto buf = buildCommand(output);
        std::string hexStr = toHex(buf);
        auto outMsg = std_msgs::msg::String();
        outMsg.data = hexStr;

        auto odomMsg = std_msgs::msg::Int32();
        auto motorMsg = std_msgs::msg::Int32();

        odomMsg.data = cmd->odom();
        motorMsg.data = cmd->motor();

        odom_publisher_->publish(odomMsg);
        motor_publisher_->publish(motorMsg);

        publisher_->publish(outMsg);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_publisher_;
    PID pid{20.0, 0.0, 0.5};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JsonTest>());
  rclcpp::shutdown();
  return 0;
}