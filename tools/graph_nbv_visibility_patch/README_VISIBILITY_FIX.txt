GRAPH NBV VISIBILITY FIX

Root cause fixed:
- old unknown gain counted every UNKNOWN cell inside a circular kernel;
- walls did not occlude the gain;
- outside/behind-wall UNKNOWN could dominate the score.

New behavior:
- circular gain is only a cheap pre-filter;
- final gain is unique UNKNOWN cells reached by ray casting;
- every ray stops at a dilated occupied wall;
- candidates require visible frontier support;
- wall-corner viewpoints receive an additional penalty.

Install over the Stage-B node, then test dry_run=true first.
