#pragma once

#include "rclcpp/rclcpp.hpp"
#include "external/termcolor.hpp"
#include "types.hpp"

#include "selfdrive_msgs/msg/system_state.hpp"
#include "selfdrive_msgs/msg/device_state.hpp"
#include "selfdrive_msgs/msg/performance.hpp"
#include "selfdrive_msgs/msg/log.hpp"

#include "selfdrive_msgs/srv/set_device_state.hpp"
#include "selfdrive_msgs/srv/set_system_state.hpp"

namespace Selfdrive
{
    class Node : public rclcpp::Node
    {
    public:
        /// @brief Construct a new Node object
        /// @param node_name
        /// @param has_ownership Optional parameter to specify if this node has ownership of the system state (only used for synchronization)
        Node(const std::string &node_name, const bool has_ownership = false);
        ~Node();

    protected:
        /// @brief Log a message with the given log level
        /// @param level 
        /// @param message 
        /// @param function_caller If not provided, the function name will be automatically determined
        /// @param line If not provided, the line number will be automatically determined
        void log(const std::string &message, Logging::LogLevel level = Logging::LogLevel::INFO, const std::string &function_caller = __builtin_FUNCTION(), int line = __builtin_LINE());

        /// @brief Set the system state
        /// @param state 
        void set_system_state(const Selfdrive::SystemState state) { set_system_state(state, has_mobility); }

        /// @brief Set the mobility
        /// @param has_mobility 
        void set_mobility(const bool has_mobility) { set_system_state(system_state, has_mobility); }

        /// @brief Set the system state and mobility
        /// @param state 
        /// @param has_mobility
        void set_system_state(const Selfdrive::SystemState state, const bool has_mobility);

        /// @brief Set the current nodes device state
        /// @param state 
        void set_device_state(const Selfdrive::DeviceState state) { set_device_state(this->get_name(), state); }

        /// @brief Set the device state of a specific node
        /// @param device 
        /// @param state 
        void set_device_state(const std::string &device, const Selfdrive::DeviceState state);

        /// @brief Get the system state
        Selfdrive::DeviceState get_device_state() { return get_device_state(this->get_name()); }

        /// @brief Get the device state of a specific node
        /// @param device 
        bool has_device_state(const std::string &device) { return device_states.find(device) != device_states.end(); }

        /// @brief Return an iterator to the beginning of the device states
        std::map<std::string, Selfdrive::DeviceState>::iterator device_states_begin() { return device_states.begin(); }

        /// @brief Return an iterator to the end of the device states
        std::map<std::string, Selfdrive::DeviceState>::iterator device_states_end() { return device_states.end(); }

        /// @brief Get the device state of a specific node
        /// @param device
        Selfdrive::DeviceState get_device_state(const std::string &device) { return device_states.at(device); }

        /// @brief Get the system state
        Selfdrive::SystemState get_system_state() { return system_state; }

        /// @brief Get the mobility
        bool is_mobility() { return has_mobility; }

        /// @brief Called when the node synchronizes with the system
        virtual void init() = 0;

        void perf_start(const std::string &name);

        void perf_stop(const std::string &name, const bool print_to_console = false);

    private:
        // State
        Selfdrive::SystemState system_state = Selfdrive::SystemState::DISABLED;
        std::map<std::string, Selfdrive::DeviceState> device_states;
        bool has_mobility = false;
        bool has_ownership = false;

        // Subscribers
        rclcpp::Subscription<selfdrive_msgs::msg::SystemState>::SharedPtr system_state_sub;
        rclcpp::Subscription<selfdrive_msgs::msg::DeviceState>::SharedPtr device_state_sub;

        // Publishers
        rclcpp::Publisher<selfdrive_msgs::msg::Performance>::SharedPtr performance_pub;
        rclcpp::Publisher<selfdrive_msgs::msg::Log>::SharedPtr log_pub;

        // Clients
        rclcpp::Client<selfdrive_msgs::srv::SetDeviceState>::SharedPtr set_device_state_client;
        rclcpp::Client<selfdrive_msgs::srv::SetSystemState>::SharedPtr set_system_state_client;

        // Functions
        void system_state_callback(const selfdrive_msgs::msg::SystemState::SharedPtr msg);
        void device_state_callback(const selfdrive_msgs::msg::DeviceState::SharedPtr msg);

        // Performance
        std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> start_times;
    };
}