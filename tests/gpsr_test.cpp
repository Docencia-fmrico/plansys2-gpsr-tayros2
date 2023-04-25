// Copyright 2021 Intelligent Robotics Lab
// Copyright 2023 Tyros2 Group
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

#include <list>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


TEST(bt_action, open_door_btn)
{
  auto node = rclcpp::Node::make_shared("plansys2_opendoor_bt_node");
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("plansys2_opendoor_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <OpenDoor    name="open_door"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  auto last_status = BT::NodeStatus::FAILURE;

  // Cambiar a 5
  for (int i = 0 ; i <= 5; i++) {
    last_status = tree.OpenDoorNode()->executeTick();

    rclcpp::spin_some(node_sink->get_node_base_interface());
    rate.sleep();
  }

  ASSERT_EQ(last_status, BT::NodeStatus::SUCCESS);
}


TEST(bt_action, move_btn)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto move_action_node = plansys2_tests::TestAction::make_shared("move");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_gpsr_tayros2");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/house_granny_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/house_granny_domain.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(ask_charge_node->get_node_base_interface());
  exe.add_node(charge_node->get_node_base_interface());
  exe.add_node(execution_logger->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  problem_client->addInstance(plansys2::Instance("tay", "robot"));

  problem_client->addInstance(plansys2::Instance("entrance", "room"));
  problem_client->addInstance(plansys2::Instance("kitchen", "room"));
 
  problem_client->addPredicate(plansys2::Predicate("(connected entrance kitchen)"));
  problem_client->addPredicate(plansys2::Predicate("(connected kitchen entrance)"));

  problem_client->addPredicate(plansys2::Predicate("(robot_at tay entrance)"));

  problem_client->setGoal(plansys2::Goal("(and(robot_at tay kitchen))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (executor_client->execute_and_check_plan()) {
    rate.sleep();
  }

  auto result = executor_client->getResult();

  ASSERT_TRUE(result.value().success);

 // ASSERT_TRUE(
 //   execution_logger->sorted(
 // {
 //   "(askcharge leia entrance chargingroom)"
 // }));

  finish = true;
  t.join();
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
