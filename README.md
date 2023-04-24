[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)

# PlanSys2 GPSR - TayROS2

<div align="center">
<img width=500px src="https://github.com/Docencia-fmrico/" alt="explode"></a>
</div>

<h3 align="center"> PlanSys2 </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-TayRos2-yellow" alt="explode"></a>
</div>

[![main](https://github.com/Docencia-fmrico/patrolling-tayros2/actions/workflows/main.yml/badge.svg)](https://github.com/Docencia-fmrico/patrolling-tayros2/actions/workflows/main.yml)


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Project Goal](#project-goal)
- [PDDL](#pddl)
- [Logic and functionality](#logic-and-functionality)
- [Behavior Tree diagrams](#behavior-tree-diagram)
- [Launcher](#launcher)
- [Tests](#tests)
- [Team](#team)
- [Licencia](#licencia)

## Project Goal

## PDDL


## Logic and functionality

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
      ;it's really important to specify that the gripper is not free at the start of the function in orther to not grab
      ;more than one object.
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

You can see the video demonstration here: [Alternative Link (Youtube)](https://www.youtube.com)


https://user-images.githubusercontent.com/72991245/220482297-1e9a4d7f-dd42




## Behavior Tree Diagram 

You can see the Behaviour Tree diagram made in **Groot**:

<div align="center">
<img width=800px src="" alt="explode"></a>
</div>

## Launcher

-----------------------------------------------------------------------
launch.py:
``` python

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
