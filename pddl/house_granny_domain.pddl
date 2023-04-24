(define (domain robots-granny-house)

  (:requirements :typing :fluents :durative-actions :universal-preconditions :conditional-effects)

  (:types
    room corridor - location
    robot gripper human door util
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (object_at ?u - util ?l - location)
    (human_at ?h - human ?l - location)

    (gripper_free ?g - gripper)
    (gripper_at ?g - gripper ?r - robot)
    (connected_by_door ?l1 ?l2 - location ?d - door)
    (connected ?l1 ?l2 - location)
    (open ?d - door)
    (close ?d - door)
    (human_attended ?h - human)
    (close_door_request ?h - human ?d - door)
    (open_door_request ?h - human ?d - door)
    (pick_request ?h - human ?u - util)

    ; testing prio y durative actions
    (no_prio_task_remaining)
  )

  (:durative-action open-door
    :parameters (?r - robot ?l1 ?l2 - location ?d - door)
    :duration (= ?duration 1)
    :condition (and
      (at start(robot_at ?r ?l1))
      (at start(close ?d))
      (at start(connected_by_door ?l1 ?l2 ?d))
    )
    :effect (and
      (at start(not (close ?d)))
      (at end(open ?d))
    )
  )

  (:durative-action close-door
    :parameters (?r - robot ?l1 ?l2 - location ?d - door)
    :duration (= ?duration 1)
    :condition (and
      (at start(robot_at ?r ?l1))
      (at start(open ?d))
      (at start(connected_by_door ?l1 ?l2 ?d))
    )
    :effect (and
      (at start(not (open ?d)))
      (at end(close ?d))
    )
  )

  ; acciones instantaneas ya que sino atraviesa a la vez una habitacion
  ; con puerta y una sin puerta estando en 2 sitios distintos a la vez
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

  (:durative-action move_without_door
    :parameters (?r - robot ?from ?to - location)
    :duration ( = ?duration 5)
    :condition (and
      (at start(robot_at ?r ?from))
      (at start(connected ?from ?to))
    )
    :effect (and
      (at start(not (robot_at ?r ?from)))
      (at end(robot_at ?r ?to))
    )
  )


  (:durative-action prio_transport
    :parameters (?r - robot ?u - util ?l1 ?l2 - location ?g - gripper ?h - human)
    :duration ( = ?duration 5)
    :condition (and

        (at start(robot_at ?r ?l1))
        (at start(gripper_at ?g ?r))
        (at start(gripper_free ?g))
        (at start(object_at ?u ?l1))

        (over all(pick_request ?h ?u))

    )
    :effect (and
        (at start(not(robot_at ?r ?l1)))
        (at end(robot_at ?r ?l2))

        (at start(not(object_at ?u ?l1)))
        (at end(object_at ?u ?l2))

        (at start(not (gripper_free ?g)))
        (at end(gripper_free ?g))

        (at end(no_prio_task_remaining))
    )
)

  (:durative-action transport
    :parameters (?r - robot ?u - util ?l1 ?l2 - location ?g - gripper)
    :duration ( = ?duration 5)
    :condition (and

        (at start(robot_at ?r ?l1))
        (at start(gripper_at ?g ?r))
        (at start(gripper_free ?g))
        (at start(object_at ?u ?l1))

        (over all(no_prio_task_remaining))

    )
    :effect (and
        (at start(not(robot_at ?r ?l1)))
        (at end(robot_at ?r ?l2))

        (at start(not(object_at ?u ?l1)))
        (at end(object_at ?u ?l2))

        (at start(not (gripper_free ?g)))
        (at end(gripper_free ?g))
    )
)

  (:action attend_pick_request
    :parameters (?u - util ?l - location ?r - robot ?g - gripper ?h - human)
    :precondition (and
      (pick_request ?h ?u)
      (object_at ?u ?l)
      (human_at ?h ?l)
    )
    :effect (and
      (not (pick_request ?h ?u))
      (human_attended ?h)
    )
  )

  (:action attend_open_door_request
    :parameters (?d - door ?h - human)
    :precondition (and
      (open_door_request ?h ?d)
      (open ?d)
    )
    :effect (and
      (not (open_door_request ?h ?d))
      (human_attended ?h)
      (no_prio_task_remaining)
    )
  )

  (:action attend_close_door_request
    :parameters (?d - door ?h - human)
    :precondition (and
      (close_door_request ?h ?d)
      (close ?d)
    )
    :effect (and
      (not (close_door_request ?h ?d))
      (human_attended ?h)
      (no_prio_task_remaining)
    )
  )

)