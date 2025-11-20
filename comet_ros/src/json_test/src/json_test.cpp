#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <nlohmann/json.hpp>
#include "utils/PID.h"
#include "msgs/message_generated.h"
#include "msgs/response_generated.h"
#include "subsystems/drivebase.h"
#include "constants.h"
#include <vector>

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

        drivebase = new Drivebase();
    }

    private:
    std::vector<uint8_t> buildCommand(std::vector<int> voltages) {
        flatbuffers::FlatBufferBuilder builder(1024);
        auto v_offset = builder.CreateVector(voltages);
        auto voltages_offset = messages::CreateVoltages(builder, v_offset);
        auto response_offset = messages::CreateResponse(builder, voltages_offset);
        builder.Finish(response_offset);

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
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        // decode hex string
        std::vector<uint8_t> data = fromHex(msg->data);

        // verify and parse flatbuffer
        auto verifier = flatbuffers::Verifier(data.data(), data.size());
        if (!messages::VerifyControllerBuffer(verifier)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid flatbuffer message");
            RCLCPP_ERROR(this->get_logger(), "Data size: %zu", data.size());
            RCLCPP_ERROR(this->get_logger(), "Hex: %s", msg->data.c_str());
            RCLCPP_ERROR(this->get_logger(), "Buffer: %s", toHex(data).c_str());
            return;
        }

        auto cmd = messages::GetController(data.data());
        // RCLCPP_INFO(this->get_logger(), "Odom: %d, Motor: %d", cmd->odom(), cmd->motor());

        float left_stick_x = cmd->left_stick_x();
        float left_stick_y = cmd->left_stick_y();
        float right_stick_x = cmd->right_stick_x();
        float right_stick_y = cmd->right_stick_y();

        drivebase->errorDrive(left_stick_y, right_stick_x);

        // create response flatbuffer
        std::vector<int> voltagesVec = std::vector<int>(voltages.begin(), voltages.end());
        auto buf = buildCommand(voltagesVec);
        std::string hexStr = toHex(buf);
        auto outMsg = std_msgs::msg::String();
        outMsg.data = hexStr;

        publisher_->publish(outMsg);
        RCLCPP_INFO(this->get_logger(), "Published response: %s", outMsg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_publisher_;
    Drivebase* drivebase;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JsonTest>());
    rclcpp::shutdown();
    return 0;
}