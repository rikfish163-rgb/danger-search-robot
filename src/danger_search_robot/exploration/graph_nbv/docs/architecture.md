# Graph NBV — Stage B

## Purpose

Validate map-only NBV viewpoint generation and execution before adding
persistent global branches and visual coverage.

## Inputs

- `/map_confirmed`
- TF `map_level -> body`

## Local graph

- Nodes: safe known-free viewpoints around the robot
- Edges: straight collision-free connections
- Path cost: Dijkstra distance from the robot node

## Utility

`U(v) = map_gain + frontier_gain + clearance - path_cost - revisit`

## Output

- move_base goal in `map_level`
- graph markers
- selected viewpoint marker
- status and finished topics

## Deliberately excluded in Stage B

- corridor-axis detection
- door or room segmentation
- visual coverage
- persistent global branch graph
- branch-continuity locking
