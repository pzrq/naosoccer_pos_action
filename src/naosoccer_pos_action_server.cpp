// Copyright 2025 Peter Schmidt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <thread>

#include "naosoccer_pos_action/action/pos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "naosoccer_pos_action/visibility_control.h"

namespace naosoccer_pos_action
{
    class NaosoccerPosActionServer : public rclcpp::Node
    {
    public:
        using Pos = naosoccer_pos_action_interfaces::action::Pos;
        using GoalHandlePos = rclcpp_action::ServerGoalHandle<Pos>;

        NAOSOCCER_POS_ACTION_CPP_PUBLIC
        explicit NaosoccerPosActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
          : Node("naosoccer_pos_action_server", options)
        {
          using namespace std::placeholders;

          this->action_server_ = rclcpp_action::create_server<Pos>(
            this,
            "naosoccer_pos_action",
            std::bind(&NaosoccerPosActionServer::handle_goal, this, _1, _2),
            std::bind(&NaosoccerPosActionServer::handle_cancel, this, _1),
            std::bind(&NaosoccerPosActionServer::handle_accepted, this, _1));
        }

    private:
        rclcpp_action::Server<Pos>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
          const rclcpp_action::GoalUUID & uuid,
          std::shared_ptr<const Pos::Goal> goal)
        {
          RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
          (void)uuid;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
          const std::shared_ptr<GoalHandlePos> goal_handle)
        {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandlePos> goal_handle)
        {
          using namespace std::placeholders;
          // this needs to return quickly to avoid blocking the executor, so spin up a new thread
          std::thread{std::bind(&NaosoccerPosActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandlePos> goal_handle)
        {
          RCLCPP_INFO(this->get_logger(), "Executing goal");
          rclcpp::Rate loop_rate(1);
          const auto goal = goal_handle->get_goal();
          auto feedback = std::make_shared<Pos::Feedback>();
          auto & sequence = feedback->partial_sequence;
          sequence.push_back(0);
          sequence.push_back(1);
          auto result = std::make_shared<Pos::Result>();

          for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
              result->sequence = sequence;
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal canceled");
              return;
            }
            // Update sequence
            sequence.push_back(sequence[i] + sequence[i - 1]);
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
          }

          // Check if goal is done
          if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          }
        }
    };  // class NaosoccerPosActionServer

}  // namespace naosoccer_pos_action

RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action::NaosoccerPosActionServer)
