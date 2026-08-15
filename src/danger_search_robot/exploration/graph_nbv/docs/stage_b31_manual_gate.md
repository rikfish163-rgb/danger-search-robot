# Stage B.3.1 Manual Gate Mode

This mode bypasses the failed automatic straight-drive test.

## Required setup

Before launching the node:

1. Manually drive the robot to the desired corridor start.
2. Stop the robot.
3. Point the robot body toward the corridor/deeper area.
4. Keep SLAM, `/map_confirmed`, TF, move_base, and TEB running.

## Initialization

At launch time:

- after several consecutive stable poses, their mean `map_level -> body`
  position becomes `gate_origin`;
- the mean stable body yaw becomes `gate_forward`;
- the gate is locked only after that stability check;
- no startup `/cmd_vel` is published.

## Gate semantics

While locked, local graph nodes, global frontier candidates, and every pose
of a `/move_base/make_plan` result must remain on the permitted side of the
direction-projection gate.

After the forward region has no reachable frontier for the configured number
of consecutive planning cycles, the gate unlocks. The node then allows global
relocation to the remaining map, while the navigation and recovery nodes keep
ownership of velocity commands. The gate unlock is therefore not a direct
reverse command.

Before locking, the node requires several consecutive robot poses to agree in
position and yaw. If the robot never becomes stable before the configured
timeout, the node stops safely and reports `ABORTED_SAFE_MANUAL_GATE_UNSTABLE`.

## Runtime heartbeat

The latched `~runtime_status` topic publishes JSON with schema
`graph_nbv_runtime_v1`. It includes the state, gate status, active/last goal,
goal failure budget, map/TF age, and recovery reason. A run is only accepted
when its final state and room/target records are checked together; process
exit alone is not a success criterion.

The health guard also rejects non-finite or numerically divergent planar TF
poses (`max_pose_abs`, default 200 m). Such a pose enters the same bounded
health-recovery path and ends in a safe abort if it does not recover, instead
of being used to generate a new exploration goal.
