# Graph NBV Stage B.2

## Fixed failure

Stage B used only information-rich viewpoints as graph nodes. Once a room
was mapped, the doorway and corridor became low-gain cells and disappeared
from the graph. The robot therefore had no graph path out of the room.

## Local graph

Two node roles are now separated:

- transit node: any safe known-free sampled position;
- target node: a transit node with visible unknown/frontier gain.

Dijkstra may traverse zero-gain nodes to reach a target.

## Global relocation

When no local target exists:

1. detect frontier components on the complete map;
2. generate safe approach points;
3. reject points without true ray-cast visibility;
4. call `/move_base/make_plan`;
5. select the best reachable global frontier;
6. relocate and resume local NBV.

Local exhaustion no longer ends exploration.
