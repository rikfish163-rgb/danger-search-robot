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

- current `map_level -> body` position becomes `gate_origin`;
- current body yaw becomes `gate_forward`;
- the gate is immediately locked;
- no startup `/cmd_vel` is published.

## Gate semantics

While locked, local graph nodes, global frontier candidates, and every pose
of a `/move_base/make_plan` result must remain on the permitted side of the
direction-projection gate.

After the forward region has no reachable frontier for eight consecutive
planning cycles, the gate unlocks.
