This patch adds a separate corridor-first, room-second exploration node.

It does not modify or delete:
- hybrid_exploration_node.py
- explore_lite.launch
- room_entry_detector.py

First run:
  roslaunch danger_search_robot corridor_room_exploration.launch dry_run:=true

The dry run publishes:
  /corridor_room_explorer/markers
  /corridor_room_explorer/status

Only after confirming the green selected-goal arrow is in front of the robot:
  roslaunch danger_search_robot corridor_room_exploration.launch dry_run:=false
