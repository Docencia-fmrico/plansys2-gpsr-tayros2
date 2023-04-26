// Copyright 2019 Intelligent Robotics Lab
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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING)
  {
  }


  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"tay", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"garage", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"living_room", "room"});
    problem_expert_->addInstance(plansys2::Instance{"doora", "door"});
    problem_expert_->addInstance(plansys2::Instance{"doorb", "door"});
    problem_expert_->addInstance(plansys2::Instance{"doorc", "door"});
    problem_expert_->addInstance(plansys2::Instance{"doord", "door"});
    problem_expert_->addInstance(plansys2::Instance{"tools", "util"});
    problem_expert_->addInstance(plansys2::Instance{"clothes", "util"});
    problem_expert_->addInstance(plansys2::Instance{"silverware", "util"});
    problem_expert_->addInstance(plansys2::Instance{"towel", "util"});
    problem_expert_->addInstance(plansys2::Instance{"robot_gripper", "gripper"});
    problem_expert_->addInstance(plansys2::Instance{"granny", "human"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tay kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(human_at granny garage)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_at robot_gripper tay)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free robot_gripper)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at tools kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at clothes living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at silverware living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at towel bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate(
      "(connected_by_door kitchen bathroom doora)"));
    problem_expert_->addPredicate(plansys2::Predicate(
      "(connected_by_door bathroom kitchen doora)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_by_door kitchen bedroom doorb)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected_by_door bedroom kitchen doorb)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_by_door kitchen bathroom doora)"));
    problem_expert_->addPredicate(
      plansys2::Predicate(
        "(connected_by_door bathroom kitchen doora)"));
    // Provisional
    // problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen bathroom)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom kitchen)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen bedroom)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom kitchen)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected living_room garage)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected garage living_room)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom garage)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected garage bedroom)"));
    //----

    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected living_room kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open doora)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close doorb)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open doorc)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open doord)"));
    problem_expert_->addPredicate(plansys2::Predicate("(no_prio_task_remaining"));
    problem_expert_->addPredicate(plansys2::Predicate("(pick_request granny tools)"));
  }

  void step()
  {
    switch (state_) {
      case STARTING:
        {
          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(human_attended granny))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          } else {
            state_ = WORKING;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = WORKING;
          }
        }
        break;
      case WORKING:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            if (action_feedback.completion) {
              std::cout << "[" << action_feedback.action << " " <<
                "SUCCESS" << "]";
            } else {
              std::cout << "[" << action_feedback.action << " " <<
                "RUNNING..." << "]";
            }
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;
            }
          }
        }
        break;
    }
  }

private:
  typedef enum {STARTING, WORKING} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
