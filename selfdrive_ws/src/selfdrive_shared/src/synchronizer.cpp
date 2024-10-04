#include "rclcpp/rclcpp.hpp"

#include "selfdrive_msgs/msg/device_state.hpp"
#include "selfdrive_msgs/msg/system_state.hpp"

#include "selfdrive_msgs/srv/set_device_state.hpp"
#include "selfdrive_msgs/srv/set_system_state.hpp"

#include "selfdrive_shared/node.hpp"
#include "selfdrive_shared/types.hpp"

class Synchronizer : public Selfdrive::Node
{
public:
    Synchronizer() : Selfdrive::Node("selfdrive_synchronizer", true)
    {
        // Publishers
        device_state_pub = this->create_publisher<selfdrive_msgs::msg::DeviceState>("/selfdrive/shared/device", 1);
        system_state_pub = this->create_publisher<selfdrive_msgs::msg::SystemState>("/selfdrive/shared/system", 1);

        // Services
        set_device_state_srv = this->create_service<selfdrive_msgs::srv::SetDeviceState>("/selfdrive/shared/set_device_state", std::bind(&Synchronizer::set_device_state_callback, this, std::placeholders::_1, std::placeholders::_2));
        set_system_state_srv = this->create_service<selfdrive_msgs::srv::SetSystemState>("/selfdrive/shared/set_system_state", std::bind(&Synchronizer::set_system_state_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void init() override
    {
        
    }

    void set_device_state_callback(const std::shared_ptr<selfdrive_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<selfdrive_msgs::srv::SetDeviceState::Response> response)
    {
        log("Setting device state: " + request->device + " to " + std::to_string(request->state), Selfdrive::Logging::DEBUG);
        
        // Did this device already exist?
        bool did_exist = has_device_state(request->device);

        // Set the device state locally
        this->set_device_state(request->device, static_cast<Selfdrive::DeviceState>(request->state));

        // Respond
        response->ok = true;

        if (!did_exist)
        {
            // Broadcast all other device states
            for (auto it = device_states_begin(); it != device_states_end(); it++)
            {
                selfdrive_msgs::msg::DeviceState msg;
                msg.device = it->first;
                msg.state = it->second;
                device_state_pub->publish(msg);
            }

            // Broadcast the system state
            selfdrive_msgs::msg::SystemState system_msg;
            system_msg.state = get_system_state();
            system_msg.mobility = is_mobility();
            system_state_pub->publish(system_msg);
            return;
        }

        // Publish the update
        selfdrive_msgs::msg::DeviceState msg;
        msg.device = request->device;
        msg.state = request->state;
        device_state_pub->publish(msg);
    }

    void set_system_state_callback(const std::shared_ptr<selfdrive_msgs::srv::SetSystemState::Request> request, std::shared_ptr<selfdrive_msgs::srv::SetSystemState::Response> response)
    {
        // Set the system state locally
        this->set_system_state(static_cast<Selfdrive::SystemState>(request->state), request->mobility);

        // Respond
        response->ok = true;

        // Publish the update
        selfdrive_msgs::msg::SystemState msg;
        msg.state = request->state;
        msg.mobility = request->mobility;
        system_state_pub->publish(msg);

        log("Setting system state to " + std::to_string(request->state) + " with mobility " + std::to_string(request->mobility), Selfdrive::Logging::DEBUG);
    }

private:
    rclcpp::Service<selfdrive_msgs::srv::SetDeviceState>::SharedPtr set_device_state_srv;
    rclcpp::Service<selfdrive_msgs::srv::SetSystemState>::SharedPtr set_system_state_srv;

    rclcpp::Publisher<selfdrive_msgs::msg::DeviceState>::SharedPtr device_state_pub;
    rclcpp::Publisher<selfdrive_msgs::msg::SystemState>::SharedPtr system_state_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Synchronizer>());
    rclcpp::shutdown();
    return 0;
}
