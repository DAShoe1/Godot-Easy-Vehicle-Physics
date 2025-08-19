extends Control

@export var vehicle : GEVPVehicle

@onready var speed_label = $VBoxContainer/Speed
@onready var rpm_label = $VBoxContainer/RPM
@onready var gear_label = $VBoxContainer/Gear

func _process(delta):
    speed_label.text = str(round(vehicle.Speed * 3.6)) + " km/h"
    rpm_label.text = str(round(vehicle.MotorRpm)) + " rpm"
    gear_label.text = "Gear: " + str(vehicle.CurrentGear)
