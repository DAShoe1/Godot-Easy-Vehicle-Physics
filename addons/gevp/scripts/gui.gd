extends Control

@export var vehicle : Vehicle

@onready var speed_label = $VBoxContainer/Speed
@onready var rpm_label = $VBoxContainer/RPM
@onready var gear_label = $VBoxContainer/Gear

func _process(delta):
	speed_label.text = str(round(vehicle.speed * 3.6)) + " km/h"
	rpm_label.text = str(round(vehicle.motor_rpm)) + " rpm"
	gear_label.text = "Gear: " + str(vehicle.current_gear)
