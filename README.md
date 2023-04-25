[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)

# PlanSys2 GPSR - TayROS2

<div align="center">
<img width=500px src="https://github.com/Docencia-fmrico/plansys2-gpsr-tayros2/blob/Readme/resources/PDDL_MAP.png" alt="explode"></a>
</div>

<h3 align="center"> PlanSys2 </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-TayRos2-yellow" alt="explode"></a>
</div>

[![main](https://github.com/Docencia-fmrico/patrolling-tayros2/actions/workflows/main.yml/badge.svg)](https://github.com/Docencia-fmrico/plansys2-gpsr-tayros2/actions/workflows/main.yml)


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Project Goal](#project-goal)
- [Package usage and dependencies](#package-usage-and-dependencies)
- [PDDL](#pddl)
- [Behavior Tree diagrams](#behavior-tree-diagram)
- [Launcher](#launcher)
- [Tests](#tests)
- [Team](#team)
- [Licencia](#licencia)

## Project Goal
The aim of this project is to create a ROS2 application that use PlanSys2 to resolve PDDL based planning systems. The project is focus in the resolution of tasks in a 5 roomed-house and the requests that a human ask the robot to do, as it is shown in the above image.

## Package usage and dependencies
This project requires the [ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) repositories, as well as the [br_navigation](https://github.com/fmrico/book_ros2) for the navigation of Tiago.

To use the repository, you will need to follow this steps:

1. Launch the ir_robots simulation launcher:
![Launch_ir_robots](https://user-images.githubusercontent.com/72991245/234414137-64e50ef9-e4d7-4d54-8688-44b4f0607b98.gif)

2. Launch the navigation from the br_navigation launcher:
![Launch_br_navigation](https://user-images.githubusercontent.com/72991245/234414154-3063c41e-56bc-4f29-8df4-69273a2a5cf9.gif)

3. Launch the gpsr-tayros2 launcher:
![Launch_plansys2](https://user-images.githubusercontent.com/72991245/234414174-55f67fa4-9a64-430a-8425-53af074eff32.gif)

4. Run the controller node:
![Run_plansys2](https://user-images.githubusercontent.com/72991245/234414191-6a4bc874-aecd-4078-b064-fa301b6a64ea.gif)


Note: Due to the problems in the updates of the Tiago, you could find some troubles in the navigation launch and using this project repository if you have updated the packages.

-----------------------------------------------------------------------
Launch Commands:
``` shell
ros2 launch ir_robots simulation.launch
ros2 launch br2_navigation tiago_navigation.launch.py
ros2 launch plansys2_gpsr_tayros2 plansys2_gpsr.launch.py
ros2 run plansys2_gpsr_tayros2 gpsr_controller_node
```
-----------------------------------------------------------------------


## PDDL

In order to solve this task, we need to represent the "robot world" in such a way that a planning algorithm can find an optimal solution to make the robot tidy up the house and assist the human with the tasks they request (such as opening and closing doors or bringing them something they need). To do this, we have used PDDL.

Some of the things we have improved for this task include the implementation of the "move_without_door" and "move_by_door" functions as durative actions. The problem we had with these functions is that the robot executed both actions simultaneously, causing the robot to be in two different positions in the house at the same time (which is impossible) or not finding a solution to the specified problem. 

``` pddl
  (:durative-action move_by_door
    :parameters (?r - robot ?from ?to - location ?d - door)
    :duration ( = ?duration 5)
    :condition (and
      (at start(robot_at ?r ?from))
      (at start(connected_by_door ?from ?to ?d))
    )
    :effect (and
      (at start(not (robot_at ?r ?from)))
      (at end(robot_at ?r ?to))
    )
  )

```

We have also added the "overall" functionality of PDDL to those conditions we believe they require it.

``` pddl
  (:durative-action pick
    :parameters (?u - util ?l - location ?r - robot ?g - gripper)
    :duration (= ?duration 1)
    :condition (and
      (at start(gripper_at ?g ?r))
      (at start(object_at ?u ?l))
      (at start(gripper_free ?g))
      (over all(robot_at ?r ?l))
      (over all(no_prio_task_remaining))
    )
    :effect (and
      ;it's really important to specify that the gripper is not free at the start of the function
      ;in orther to not grab more than one object.
      (at start(not (gripper_free ?g)))
      (at end(not (object_at ?u ?l)))
      (at end(robot_carry ?r ?g ?u))
    )
  )
```

And the use of "high-level" functions that need the execution of some other speficific funtions in orther to execute it.

``` pddl

  (:action organize_object
    :parameters (?h - human ?u - util ?l - location)
    :precondition (and
      (human_attended ?h)
      (object_at ?u ?l)
    )
    :effect (and
      (move_object ?u ?l)
    )
  )
```
-----------------------------------------------------------------------


### Demonstration

You can see the video demonstration here:

1. Simple example video #1 using the initial Transport method (organize objects): [Alternative Link (Youtube)](https://www.youtube.com/watch?v=6aOHB-sJ96M)

https://user-images.githubusercontent.com/72991245/234416000-699a8d96-427e-407f-aa10-15e49cf78ad9.mp4

2. Complete example video #2 using the Pick & Drop method: [Alternative Link (Youtube)](https://www.youtube.com/watch?v=F57P4S2pAxM&feature=youtu.be)

https://user-images.githubusercontent.com/72991245/234413454-8c0a70c2-16ee-43a5-8b04-d4e07adb2c7e.mp4

3. Example video #3 (Granny requests): [Alternative Link (Youtube)](https://www.youtube.com/watch?v=F57P4S2pAxM&feature=youtu.be)

https://user-images.githubusercontent.com/72991245/234413719-7fd676d1-9793-40ab-be9d-def6d1722a16.mp4

## Behavior Tree Diagram 

You can see the Behaviour Tree diagram made in **Groot**:

<div align="center">
<img width=800px src="" alt="explode"></a>
</div>

## Launcher

To launch all the action nodes and the bt nodes, a launcher has been implemented which also serves to connect the different nodes with their configuration parameter file (/config/params.yaml) and the bt nodes with their respective trees (.xml).

-----------------------------------------------------------------------
plansys2_gpsr.launch.py(snippet):
``` python
move_without_door_cmd = Node(
    package='plansys2_bt_actions',
    executable='bt_action_node',
    name='move_wod',
    namespace=namespace,
    output='screen',
    parameters=[
        example_dir + '/config/params.yaml',
        {
        'action_name': 'move_without_door',
        'publisher_port': 1676,
        'server_port': 1677,
        'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
        }
    ])
```
-----------------------------------------------------------------------

## Controller

In order to facilitate the user with the implementation of what would be the typical "problem.pddl" file and using the modularity offered by Plansys2, which allows us to use functions from the classes responsible for the operation of domain and problem processing as well as the correct functioning of the planner and executor; we have created a ros2 node as a "controller" that will be responsible for instantiating "the robot's world", indicating the goal to be achieved and ensuring proper robot operation (feedback). The logic for this node is located in the "gpsr_controller_node.cpp" file.

gpsr_controller_node.cpp:
``` c++
    // Initializing the objects that will allow us to use the functionality of the Plansys2 modules.
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    // ...
    // Initializing the robot's world.
    problem_expert_->addInstance(plansys2::Instance{"tay", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"garage", "room"});
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tay kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(human_at granny bedroom)"));
    // ...
    // Indicating the goal to be achieved.
    problem_expert_->setGoal(plansys2::Goal("(and(object_at tools bedroom))"));
    // ...
    // Executing the plan
    if (executor_client_->start_plan_execution(plan.value())) {
      state_ = WORKING;
    }
    //...
    // getting feedback
    auto feedback = executor_client_->getFeedBack();

    for (const auto & action_feedback : feedback.action_execution_status) {
      if (action_feedback.completion) {
        std::cout << "[" << action_feedback.action << " " <<
        "SUCCESS" << "]";
      }
      else {
        std::cout << "[" << action_feedback.action << " " <<
        "RUNNING..." << "]";
      }
    }
```
-----------------------------------------------------------------------

## Tests
### test 1

### test 2

### test 3


## Team

<div align="center">
<img width=200px src="https://github.com/Docencia-fmrico/bump-and-stop-tayros2/blob/readme/resources/figures/logo.png" alt="explode"></a>
</div>

- [Adrian Cobo](https://github.com/AdrianCobo)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)
- [Saul Navajas](https://github.com/SaulN99)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayROS2) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
