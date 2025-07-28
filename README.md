
<div align="right">
  <details>
    <summary >🌐 Language</summary>
    <div>
      <div align="center">
        <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=en">English</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=zh-CN">简体中文</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=zh-TW">繁體中文</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=ja">日本語</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=ko">한국어</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=hi">हिन्दी</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=th">ไทย</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=fr">Français</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=de">Deutsch</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=es">Español</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=it">Italiano</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=ru">Русский</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=pt">Português</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=nl">Nederlands</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=pl">Polski</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=ar">العربية</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=fa">فارسی</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=tr">Türkçe</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=vi">Tiếng Việt</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=id">Bahasa Indonesia</a>
        | <a href="https://openaitx.github.io/view.html?user=DAShoe1&project=Godot-Easy-Vehicle-Physics&lang=as">অসমীয়া</
      </div>
    </div>
  </details>
</div>

# Godot-Easy-Vehicle-Physics
A ray cast based rigid body vehicle system for Godot 4.2 and above

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

## Projects using Godot Easy Vehicle Physics
If you are working on a project using this add-on and want to list it here, open an issue to let me know.
- [Jack of all Cars](https://dashoe1.itch.io/jack-of-all-cars)
- [Another Lap](https://store.steampowered.com/app/3550210/Another_Lap/)
