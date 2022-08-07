#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_interfaces_test/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace action_tutorials_cpp
{
    class FibonacciActionClient : public rclcpp::Node
    {
    public:
        using Fibonacci = action_interfaces_test::action::Fibonacci;
        using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

        explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
            : Node("fibonacci_action_client", options)
        {
            //設定外部接收的參數
            this->declare_parameter<std::string>("shape", "circle");
            // this->declare_parameter<std::int32_t>("mode", 0);

            this->client_ptr_ = rclcpp_action::create_client<action_interfaces_test::action::Fibonacci>(
                this,
                "fibonacci");

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&FibonacciActionClient::send_goal, this));
        }

        void send_goal()
        {
            using namespace std::placeholders;

            this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server())
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            //接收外部輸入參數
            this->get_parameter("shape", parameter_string_);
            RCLCPP_INFO(this->get_logger(), "輸入的模式 :  %s ", parameter_string_.c_str());

            // this->get_parameter("mode", parameter_number);
            // RCLCPP_INFO(this->get_logger(), "輸入模式 :  %d ", parameter_number);

            // input parameters
            auto goal_msg = Fibonacci::Goal();
            goal_msg.circle = parameter_string_; //可以改成parameters去接
            goal_msg.times = 360;
            
            RCLCPP_INFO(this->get_logger(), "Shape mode: %s ",goal_msg.circle.c_str());
            RCLCPP_INFO(this->get_logger(), "Sending goal");
            
            
	
            auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&FibonacciActionClient::result_callback, this, _1);
                
                
            
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
	    
            
        }

    private:
        //定義外部接收的參數
        std::string parameter_string_;
        // std::int32_t parameter_number;

        rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

        void goal_response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(
            GoalHandleFibonacci::SharedPtr,
            const std::shared_ptr<const Fibonacci::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %d", feedback->process);
        }

        void result_callback(const GoalHandleFibonacci::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
            RCLCPP_ERROR(this->get_logger(), "ok");
            rclcpp::shutdown();
        }
    }; // class FibonacciActionClient

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
