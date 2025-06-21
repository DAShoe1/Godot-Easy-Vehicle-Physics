extends Node3D
## Controls any [Vehicle] node using custom-defined input maps.
class_name VehicleController

## The [Vehicle] that this vehicle controller will send
## input values to. Required for the vehicle controller to work properly.
@export var vehicle_node : Vehicle

@export_group("Input Maps", "string_")
## The name of the input map used for this vehicle's brakes input.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_brake_input: String = "Brakes"
## The name of the input map used for steering this vehicle left.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_steer_left: String = "Steer Left"
## The name of the input map used for steering this vehicle right.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_steer_right: String = "Steer Right"
## The name of the input map used for this vehicle's throttle input.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_throttle_input: String = "Throttle"
## The name of the input map used for this vehicle's handbrake input.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_handbrake_input: String = "Handbrake"
## The name of the input map used for this vehicle's clutch input.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_clutch_input: String = "Clutch"
## The name of the input map used for enabling or disabling
## the transmission of this vehicle.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_toggle_transmission: String = "Toggle Transmission"
## The name of the input map used for shifting up a gear when
## manual transmission is enabled.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_shift_up: String = "Shift Up"
## The name of the input map used for shifting down a gear when
## manual transmission is enabled.
## [br]The input map must be present in your project, and can be set at [code]Project > Project Settings > Input Map[/code].
## [br]Leave blank to disable.
@export var string_shift_down: String = "Shift Down"

func _physics_process(_delta):
	
	if string_brake_input != "":
		vehicle_node.brake_input = Input.get_action_strength(string_brake_input)

	if string_steer_left != "" and string_steer_right != "":
		vehicle_node.steering_input = Input.get_action_strength(string_steer_left) - Input.get_action_strength(string_steer_right)

	if string_throttle_input != "":
		vehicle_node.throttle_input = pow(Input.get_action_strength(string_throttle_input), 2.0)

	if string_handbrake_input != "":
		vehicle_node.handbrake_input = Input.get_action_strength(string_handbrake_input)
	
	if string_clutch_input != "":
		vehicle_node.clutch_input = clampf(Input.get_action_strength(string_clutch_input) + Input.get_action_strength(string_handbrake_input), 0.0, 1.0)
	
	if string_toggle_transmission != "":
		if Input.is_action_just_pressed(string_toggle_transmission):
			vehicle_node.automatic_transmission = not vehicle_node.automatic_transmission
	
	if string_shift_up != "":
		if Input.is_action_just_pressed(string_shift_up):
			vehicle_node.manual_shift(1)
	
	if string_shift_down != "":
		if Input.is_action_just_pressed(string_shift_down):
			vehicle_node.manual_shift(-1)
	
	# Reverse gear logic

	if vehicle_node.current_gear == -1:
		vehicle_node.brake_input = Input.get_action_strength(string_throttle_input)
		vehicle_node.throttle_input = Input.get_action_strength(string_brake_input)
