# Actions

`/execute_grasp` uses `assistant_msgs/ExecuteGrasp`.

Goal: `object_id`, `geometry_msgs/Pose target_pose`.

Result: `success`, `message`.

Feedback stages: `target_received`, `validating`, `planning`,
`moving_to_pregrasp`, `approaching`, `closing_gripper`, `lifting`,
`returning`, and `complete`.

The motion node rechecks the selected object in the latest detection array and
revalidates calibration, target pose, bounds, reachability, selection stability,
hardware, and MoveIt readiness. Mock mode emits the same stage sequence as a
simulation and never commands the arm.
