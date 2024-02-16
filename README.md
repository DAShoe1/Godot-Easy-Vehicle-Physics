# Godot-Easy-Vehicle-Physics
A ray cast based rigid body vehicle for Godot 4.2

## Description
A physics based vehicle controller designed to play well on a keyboard and be easy to configure. All parameters are contained in one script (vehicle.gd) and tooltips are provided for all of them.

## Features
This controller features many systems that assist with making a competent vehicle such as:
- Steering Assists
- Countersteer Assists
- Traction Control
- Anti-Lock Brakes
- Stability Assist
- In Air Assist
- Automatic Gearbox
- Locking Differentials
- Tire Grip Assists
- Calculated suspension parameters; proper suspension parameters can be difficult to get right, therefore this controller will calculate spring rates and damping rates based on ratios and weight distribution.

## Examples
4 example vehicles are included:
- demo_arcade.tscn: Handles similar to an arcade style race car, lots of grip, easy to control, and lots of assists.
- demo_simcade.tscn: Handles closer to a real car with assists to help keep the car under control.
- demo_monster_truck.tscn: Handles like a monster truck, with very little assists.
- demo_drift.tscn: Setup for easy drifting.

## Controls
- Steering: Left: ← or a Right: → or d
- Throttle: ↑ or w
- Brake: ↓ or s
- Handbrake: Space
- Clutch: C
- Toggle Gearbox (Auto/Manual): T
- Shift Up: F or +
- Shift Down: R or -
- Debug Info: ~
- Switch Debug Screens: < or >

## Physics Engine
This controller works well in both the Godot and Jolt physics engines. A physics tick rate of at least 120 is recommended and higher values can be used, but note that vehicle behavior will be different when switching tick rates.

## Acknowledgments
Huge thanks to Dechode and Wolfe for sharing their projects to the Godot community. Sharing this project is my way of paying it forward. Portions of this code are based on [Dechode's Godot Advanced Vehicle](https://github.com/Dechode/Godot-Advanced-Vehicle) and [SERIES: Driving Simulator Workshop](https://lupine-vidya.itch.io/gdsim/devlog/677572/series-driving-simulator-workshop-mirror) and attribution is included in the appropriate source files.
The car model provided as a demo is from [Kenney.nl Car Kit](https://www.kenney.nl/assets/car-kit)
