# Crosswalk Guardian (ROS 2 + Gazebo)

ROS 2 Humble + Gazebo simulation for a crosswalk safety system using TurtleBot3.
Robot motion is gated by a traffic-light supervisory topic for safe behavior.

## Features
- Safe startup: actuation node starts first (robot stays idle until valid signal)
- Traffic light states: GREEN / YELLOW / RED
- GREEN: robot patrols curb-to-curb
- YELLOW: publishes warning + continuous sound + 180° turn at curb only
- RED: robot stops immediately

## Structure
- `cg_nodes/` : Python nodes
- `worlds/` : Gazebo world file
- `models/` : Custom models used by the world
- `run_sim.sh` : Launch helper

## Quick Start
```bash
cd crosswalk_guardian_project
./run_sim.sh
