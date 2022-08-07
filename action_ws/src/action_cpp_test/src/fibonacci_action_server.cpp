#include <functional>
#include <memory>
#include <thread>

#include "action_interfaces_test/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_cpp_test/visibility_control.h"

namespace action_tutorials_cpp
{
    class FibonacciActionServer : public rclcpp::Node
    {
    public:
        using Fibonacci = action_interfaces_test::action::Fibonacci;
        using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

        ACTION_TUTORIALS_CPP_PUBLIC
        explicit FibonacciActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("fibonacci_action_server", options)
        {
            using namespace std::placeholders;
	   
            this->action_server_ = rclcpp_action::create_server<Fibonacci>(
                this,
                "fibonacci",
                std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
                std::bind(&FibonacciActionServer::handle_cancel, this, _1),
                std::bind(&FibonacciActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Fibonacci::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with shape %s", goal->circle.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Fibonacci::Feedback>();
            auto &process_number = feedback->process;
            process_number = 100; //set初始值
            auto result = std::make_shared<Fibonacci::Result>();
	    
	    if (goal->circle == "circle")
	    {
	    	//turtle1 is circle.
            	for (int i = 0; (i < (goal->times+10000)) && rclcpp::ok(); i+=30) //for loop 控制暫停
            	{
                	// Check if there is a cancel request
                	if (goal_handle->is_canceling())
                	{
                    	   result->finish = "Interrupt!";
                    	   goal_handle->canceled(result);
                           RCLCPP_INFO(this->get_logger(), "Goal canceled");
                           return;
                        }

                       // 主程式
                       process_number = i ;
		
                	// Publish feedback
                	goal_handle->publish_feedback(feedback);
                	RCLCPP_INFO(this->get_logger(), "Publish feedback %d", process_number);

                	loop_rate.sleep();
            	}
            }
	   
            // Check if goal is done
            if (rclcpp::ok())
            {
                result->finish = "Circling is OK.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
    }; // class FibonacciActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto action_server = std::make_shared<MinimalActionServer>();

//     rclcpp::spin(action_server);

//     rclcpp::shutdown();
//     return 0;
// }
