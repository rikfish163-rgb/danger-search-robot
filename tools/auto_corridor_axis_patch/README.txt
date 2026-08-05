AUTO CORRIDOR AXIS PATCH

This replacement removes both of these dependencies:
1. robot startup yaw;
2. manual RViz Publish Point calibration.

The node estimates:
- the corridor orientation;
- the unexplored forward direction;
- the lateral centerline offset.

It waits for five stable map-based estimates before locking the axis.

Important topics:
  /corridor_room_explorer/status
  /corridor_room_explorer/axis_diagnostic
  /corridor_room_explorer/markers

Marker colors:
  yellow line = current automatic estimate, not locked;
  cyan line = locked automatic corridor centerline;
  green arrow = selected corridor goal;
  orange arrow = selected room goal.

Always run dry_run=true first.
