# EGO upstream source

Source: [ZJU-FAST-Lab/ego-planner-swarm, ros2_version](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/23a8d5a191711dd65633df689bd00f55d4dea8f9).
The original [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)
README directs ROS 2 users to this port. This integration uses drone ID 0 with
an empty peer list.

`upstream-lock.json` records the full commit and SHA-256 of each original file.
Sources are ordinary vendored files, with the upstream GPLv3 license and source
notices retained. No Git submodule or network fetch is needed during a build.
Most lines in this package are imported upstream code, not first-party changes.
Some inherited Fast-Planner headers declare LGPL-3.0-or-later; those notices
are preserved and `COPYING.LESSER` supplies the corresponding license text.

Included: planner manager, B-spline optimizer, A*, grid map support, polynomial
initialization, visualization and object-prediction support needed for linking.
Excluded: upstream FSM, trajectory controller, simulators, TCP/swarm transport,
demo launches and ROS package manifests. The first-party CMake target builds the
core directly; the upstream sensor mapper and object predictor are not started.

Local patches to the imported source:

- `GridMap` accepts an external resolution/occupancy query. This lets EGO use
  flight-stack's classified map without starting its own sensor subscriptions.
- `EGOPlannerManager::initPlanModules` accepts that map; its unused generated
  `DataDisp` message include is removed.
- The manager accepts a supplied route polynomial for initialization and an
  optional stopping boundary. The rebound optimizer can fix its final three
  control points; the default diagnostic behavior is unchanged. The adapter
  imposes exact start PVA and a stationary endpoint before optimization.
- Visualization calls tolerate a null observer for the in-process optimizer.
- Visualization markers consistently use the existing `map` frame.

Patched files use LF line endings. Other imported source bytes match the lock.
Original CRLF and trailing whitespace are preserved in the upstream import;
exclude this vendored subtree when checking first-party whitespace changes.
First-party adapter code is GPL-3.0-only, matching the combined package license.
