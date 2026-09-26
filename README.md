# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

---

## My Solution

The robot navigates to a clicked goal using four nodes, each covering one
part of the assignment's simple model of the brain:

| Brain module | Node | Role |
|---|---|---|
| Perception | `costmap` | Turns each lidar scan into a local grid of obstacle costs around the robot |
| World modeling + memory | `map_memory` | Stitches those local grids into a global map that remembers obstacles once they're out of view |
| Configuration + planning | `planner` | Tracks whether there's an active goal and runs A* to find a safe path to it |
| Action | `control` | Follows the path with pure pursuit and sends velocity commands to the wheels |

**Demo video:** [link here]

### How data flows

1. `/lidar` scans go into **costmap**, which publishes `/costmap`.
2. **map_memory** combines `/costmap` with the robot's pose from
   `/odom/filtered` and publishes `/map`.
3. **planner** uses `/map`, the pose, and a goal from `/goal_point` to
   publish `/path`.
4. **control** follows `/path` using the pose and publishes `/cmd_vel`.

### Node details

**costmap.** A 30×30 m grid at 0.05 m resolution, centered on the lidar and
rebuilt every scan. Each valid reading is converted from polar to Cartesian
coordinates and floored into a cell, which is marked 100. Obstacles are then
inflated with `100·(1 − d/2.0)` out to 2.0 m, keeping the higher cost where
halos overlap.

**map_memory.** A fixed 40×40 m grid at 0.1 m in `sim_world`, centered on
the room. Once per second, each obstacle cell in the latest costmap is placed
in the world (cell center → rotate by the robot's yaw → shift by its
position) and merged with `max()`. The map is published every second,
starting at launch.

**planner.** A two-state machine (waiting for a goal / driving to it) around
8-connected A*. Cells within 1.4 m of an obstacle are blocked, and the outer
halo adds a step-cost penalty so paths keep extra clearance. It replans
whenever a new map arrives, and clears the path within 0.5 m of the goal.

**control.** Pure pursuit with a 1.5 m lookahead at 0.5 m/s, capped at
1 rad/s. If the target is more than 60° off heading, the robot turns in place
first. It stops within 0.5 m of the goal.

## Design Choices

**Planning around the robot's real size.** I measured the robot in Foxglove
at about 2.1 × 1.5 m. Its center-to-corner distance is about 1.3 m, so with a
0.1 m margin, A* blocks any cell within 1.4 m of an obstacle. The costmap
inflation radius had to grow from 1 m to 2 m for this to work. The blocking
threshold comes from the formula `100·(1 − 1.4/2.0) = 30`, and cells with
cost 1–30 form a "soft" zone that A* is allowed into but prefers to avoid
(step cost × `1 + 5·cost/100`).

**Planning and control use the robot's center, not the lidar.**
`/odom/filtered` reports the lidar's pose, and the lidar is mounted 0.8 m
forward of the chassis center. A differential-drive robot turns around its
center, so both nodes shift the pose back by 0.8 m along the heading. Planning
around the lidar point instead would have needed about a 2.1 m radius and
blocked many real gaps.

**Map fusion only adds obstacles and keeps the maximum.** My costmap doesn't
distinguish "unseen" from "free" (both are 0), so overwriting the map with new
data would erase obstacles as soon as they were hidden behind other objects.
Since the world is static, only cells with cost > 0 are fused, and each cell
keeps `max(old, new)`.

**Fusing on every tick instead of every 1.5 m of travel.** Because `max()`
fusion is idempotent, fusing the same view repeatedly can't corrupt the map.
This captures what the robot sees while turning in place, and gives the
planner a map at startup without driving first.

**The costmap (0.05 m) is finer than the map (0.1 m),** so rotated costmap
cells can't leave holes in the global map.

**Edge cases:**
- A* may step from a blocked cell into a lower-cost neighbor, so a robot that
  ends up too close to a wall can still plan its way out.
- Goals inside the blocked zone or off the map are rejected with a warning.
- If no path exists, the planner publishes an empty path so the robot stops.
- The controller publishes a stop command once rather than continuously, so
  it doesn't fight the Foxglove teleop panel on `/cmd_vel`.

## Known Limitations and Future Improvements

- **Raytracing in the costmap** (marking cells as unknown vs. free) would let
  the map clear phantom obstacles and handle moving objects.
- **Timestamp pairing in map memory.** Fusion uses the latest costmap with the
  latest pose, so fusing while the robot turns can slightly smear obstacles.
  Pairing each costmap with the odometry nearest its timestamp would fix this.
- **Backward mapping** (for each map cell, sample the costmap) would remove
  holes without needing a finer costmap, and reduce costmap computation.
- **Path smoothing** to remove the 45° steps of grid-based A*.
- **Speed control** that slows down on sharp curves and near the goal.
- **A rectangular footprint check** instead of a circle, to fit through
  tighter gaps.
- **ROS parameters** instead of compile-time constants, for tuning without
  rebuilding.
- **Unit tests** for the core functions (cell conversions, yaw, A*).