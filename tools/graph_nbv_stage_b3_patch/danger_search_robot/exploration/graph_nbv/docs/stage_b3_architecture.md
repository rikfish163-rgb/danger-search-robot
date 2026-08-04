# Graph NBV Stage B.3

Stage B.3 contains all Stage B.2 behavior and adds a startup gate state
machine.

## Startup

1. Wait for `/map_confirmed`.
2. Wait for `map_level -> body`.
3. Wait for `odom -> base`.
4. Record both starting poses.
5. Publish `/cmd_vel` at 20 Hz.
6. Drive at 1.0 m/s with `linear.y = -0.005`.
7. Slow at 11.00 m.
8. Stop at 11.85 m.
9. Wait two seconds.
10. Record the actual map-frame gate point.

## Gate direction

The direction is calculated from:

`map_start -> map_gate`

It therefore remains valid when the corridor aligns with map x, map y, or
an arbitrary angle.

## Locked exploration

While the gate is locked:

- local transit and target nodes behind the gate are removed;
- local graph edges may not cross the gate;
- global frontier approach points behind the gate are removed;
- every pose returned by `/move_base/make_plan` is checked;
- a plan is rejected if any pose crosses behind the gate margin.

## Unlock

When no reachable frontier remains in the locked forward region for eight
consecutive planning cycles, the gate unlocks. Behind-gate candidates then
receive a score penalty rather than a hard rejection.

## Markers

- red line: locked gate;
- gray line: unlocked gate;
- green arrow: permitted forward direction.
