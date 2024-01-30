# Godot-Easy-Vehicle-Physics
A ray cast based rigid body vehicle for Godot 4.2

## Description
A physics based vehicle controller designed to play well on a keyboard and be easy to configure. All parameters are contained in one script (vehicle.gd) and tooltips are provided for all of them. A vehicle scene (car.tscn) provides an example of the car configured in an arcade style and a demo scene (demo.tscn) provides a basic track, input controler, and engine sound controller as an example.

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
- Calculated suspension parameters; Proper suspension parameters can be difficult to get right, therefore this controller will calculate spring rates and damping rates based on ratios and weight distribution.

## Controls
Steering: Left: ← or a Right: → or d
Throttle: ↑ or w
Brake: ↓ or s
Handbrake: Space
Clutch: C
Toggle Gearbox (Auto/Manual): T
Shift Up: F or +
Shift Down: R or -

## Physics Engine
This controller works well in both the Godot and Jolt physics engines. A physics tick rate of at least 120 is recommended and higher values can be used, but note that vehicle behavior will be different when switching tick rates.

## Acknowledgments
Huge thanks to Decode for sharing his project to the Godot community. Sharing this project is my way of paying it forward. Portions of this code are based on [Dechode's Godot Advanced Vehicle](https://github.com/Dechode/Godot-Advanced-Vehicle) and attribution is included in the appropriate source files.
The car model provided as a demo is from [Kenney.nl Car Kit](https://www.kenney.nl/assets/car-kit)
