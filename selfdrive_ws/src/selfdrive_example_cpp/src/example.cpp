#include "selfdrive_shared/node.hpp"
#include "selfdrive_msgs/msg/motor_input.hpp"

class ExampleCPP : public Selfdrive::Node
{
    public:
        ExampleCPP() : Selfdrive::Node("example_cpp") 
        {
            log("Hello from ExampleCPP", Selfdrive::Logging::DEBUG);
            log("Hello from ExampleCPP", Selfdrive::Logging::INFO);
            log("Hello from ExampleCPP", Selfdrive::Logging::WARN);
            log("Hello from ExampleCPP", Selfdrive::Logging::ERROR);
            log("Hello from ExampleCPP", Selfdrive::Logging::FATAL);
        }
        ~ExampleCPP() = default;

        void init() override
        {
            log("Initialized");
            set_device_state(Selfdrive::DeviceState::READY);

            perf_start("ExampleCPP::init");
            perf_stop("ExampleCPP::init", true);
        }

        void timer_callback()
        {
            selfdrive_msgs::msg::MotorInput motor_input;
            motor_input.forward_velocity = 0.5;
            motor_input.angular_velocity = 0.5;
            motor_input_pub->publish(motor_input);
        }

    rclcpp::TimerBase::SharedPtr timer = create_wall_timer(std::chrono::seconds(1), std::bind(&ExampleCPP::timer_callback, this));
    rclcpp::Publisher<selfdrive_msgs::msg::MotorInput>::SharedPtr motor_input_pub = create_publisher<selfdrive_msgs::msg::MotorInput>("/selfdrive/MotorInput", 10);
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ExampleCPP> example_cpp = std::make_shared<ExampleCPP>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(example_cpp);
    executor.spin();
    executor.remove_node(example_cpp);
    rclcpp::shutdown();
    return 0;
}