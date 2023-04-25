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

#include "/src/behavior_tree_nodes/OpenGripper.hpp"
#include "/src/behavior_tree_nodes/CloseGripper.hpp"
#include "/src/behavior_tree_nodes/Move.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


TEST(bt_action, open_door_btn)
{
  auto node = rclcpp::Node::make_shared("plansys2_opendoor_bt_node");

  rclcpp::spin_some(node);

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
  int counter = 0;
  bool finish = false;

  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rate.sleep();
    counter++;
  }

  ASSERT_EQ(counter, 6);
}


TEST(bt_action, move_btn)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

  std::vector<std::string> plugins = {
    "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

  auto bt_action = std::make_shared<plansys2::BTAction>("assemble", 100ms);

  auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)", lc_node);

  bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
  bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
  bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

  bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(bt_action->get_node_base_interface());
  exe.add_node(lc_node->get_node_base_interface());

  bool finished = false;
  while (rclcpp::ok && !finished) {
    exe.spin_some();

    action_client->tick(lc_node->now());
    finished = action_client->get_status() == BT::NodeStatus::SUCCESS;
  }

  auto start = lc_node->now();
  while ( (lc_node->now() - start).seconds() < 2) {
    exe.spin_some();
  }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
