extends Node

@export var vehicle : Vehicle
@export var show_debug := true

@onready var debug_ui : DebugUI = $DebugUI

var debug_sets := ["All", "Inputs", "Tire Forces", "Suspension Forces", "Drivetrain", "Stability"]
var current_debug_set := 0

func _process(delta):
	if Input.is_action_just_pressed("ShowDebug"):
		show_debug = !show_debug
		debug_ui.clear_debug()
	
	if Input.is_action_just_pressed("DebugNext"):
		switch_debug_set(current_debug_set + 1)
	
	if Input.is_action_just_pressed("DebugPrevious"):
		switch_debug_set(current_debug_set - 1)
	
	if not show_debug:
		return
	
	var center_of_gravity := vehicle.global_transform * vehicle.center_of_mass
	var front_axle := vehicle.global_transform * vehicle.front_axle_position
	var rear_axle := vehicle.global_transform * vehicle.rear_axle_position
	var center_axle : Vector3 = lerp(front_axle, rear_axle, 0.5)
	debug_ui.draw_debug_circle("cog", center_of_gravity, 2.0, Color.BLUE)
	debug_ui.draw_debug_circle("faxle", front_axle, 2.0, Color.GREEN)
	debug_ui.draw_debug_circle("raxle", rear_axle, 2.0, Color.GREEN)
	debug_ui.draw_debug_circle("caxle", center_axle, 2.0, Color.GREEN)
	
	debug_ui.draw_debug_text("title", "Debug: " + debug_sets[current_debug_set], Vector2(10, 100), Color.WHITE, true)
	
	if Engine.physics_ticks_per_second < 120:
		debug_ui.draw_debug_text("lowphysicsrate", "LOW PHYSICS TICK RATE.", Vector2(10, 300), Color.RED, true)
		debug_ui.draw_debug_text("lowphysicsrate2", "Set physics ticks per second to at least 120.", Vector2(10, 320), Color.RED, true)
	
	if debug_sets[current_debug_set] == "Tire Forces":
		debug_ui.draw_debug_text("tslip", "Tire Slip", Vector2(10, 120), Color.YELLOW, true)
		debug_ui.draw_debug_text("tforce", "Tire Force", Vector2(10, 140), Color.RED, true)
	
	if debug_sets[current_debug_set] == "Suspension Forces":
		debug_ui.draw_debug_text("sforce", "Total Suspension Force", Vector2(10, 120), Color.RED, true)
		debug_ui.draw_debug_text("sarb", "Antiroll Bar Force", Vector2(10, 140), Color.BLUE, true)
		debug_ui.draw_debug_text("sdamping", "Damping Force", Vector2(10, 160), Color.YELLOW, true)
		debug_ui.draw_debug_text("slength", "Current Suspension Length", Vector2(10, 180), Color.GREEN, true)
	
	if debug_sets[current_debug_set] == "Inputs":
		debug_ui.draw_debug_text("isteer", "Steering Input", Vector2(10, 120), Color.GRAY, true)
		debug_ui.draw_debug_text("irsteer", "Assisted Steering Input", Vector2(10, 140), Color.WHITE, true)
		debug_ui.draw_debug_text("ithrottle", "Throttle Input", Vector2(10, 160), Color.GREEN, true)
		debug_ui.draw_debug_text("iengine", "Engine Torque Output", Vector2(10, 180), Color.AQUA, true)
		debug_ui.draw_debug_text("idt", "Drivetrain Torque Output", Vector2(10, 200), Color.DEEP_SKY_BLUE, true)
		debug_ui.draw_debug_text("ibrake", "Brake Input", Vector2(10, 220), Color.RED, true)
		debug_ui.draw_debug_text("iclutch", "Clutch Input", Vector2(10, 240), Color.YELLOW, true)
		debug_ui.draw_debug_text("ihandbrake", "Handbrake Input", Vector2(10, 260), Color.ORANGE, true)
	
	if debug_sets[current_debug_set] == "Drivetrain":
		debug_ui.draw_debug_text("dtinfo", "Torque Split", Vector2(10, 120), Color.ORANGE, true)
	
	if debug_sets[current_debug_set] == "Stability":
		debug_ui.draw_debug_text("styaw", "Yaw Torque", Vector2(10, 120), Color.RED, true)
		debug_ui.draw_debug_text("stup", "Keep Upright Torque", Vector2(10, 140), Color.MAGENTA, true)
	
	if debug_sets[current_debug_set] == "Inputs" or debug_sets[current_debug_set] == "All":
		debug_ui.draw_debug_line("front_steer", front_axle, vehicle.global_transform.basis * Vector3(-sin(vehicle.steering_input * vehicle.max_steering_angle), 0.0, -cos(vehicle.steering_input * vehicle.max_steering_angle)), Color.GRAY)
		debug_ui.draw_debug_line("front_real_steer", front_axle, vehicle.global_transform.basis * Vector3(-sin(vehicle.true_steering_amount), 0.0, -cos(vehicle.true_steering_amount)), Color.WHITE)
		debug_ui.draw_debug_line("throttle_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(0.0, 0.0))
		debug_ui.draw_debug_line("throttle", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * vehicle.throttle_amount, Color.GREEN, false, Vector2(0.0, 0.0))
		debug_ui.draw_debug_line("torque_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(2.0, 0.0))
		debug_ui.draw_debug_line("torque", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * (vehicle.torque_output / vehicle.max_torque), Color.AQUA, false, Vector2(2.0, 0.0))
		debug_ui.draw_debug_line("dt_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(4.0, 0.0))
		debug_ui.draw_debug_line("dt", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * (vehicle.clutch_torque / vehicle.max_torque), Color.DEEP_SKY_BLUE, false, Vector2(4.0, 0.0))
		debug_ui.draw_debug_line("brake_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(6.0, 0.0))
		debug_ui.draw_debug_line("brake", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * (vehicle.brake_amount), Color.RED, false, Vector2(6.0, 0.0))
		debug_ui.draw_debug_line("clutch_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(8.0, 0.0))
		debug_ui.draw_debug_line("clutch", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * (vehicle.clutch_amount), Color.YELLOW, false, Vector2(8.0, 0.0))
		debug_ui.draw_debug_line("hb_back", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * 1.0, Color.BLACK, false, Vector2(10.0, 0.0))
		debug_ui.draw_debug_line("hb", center_of_gravity + (vehicle.global_transform.basis.x * 1.5), vehicle.global_transform.basis.y * (vehicle.handbrake_input), Color.ORANGE, false, Vector2(10.0, 0.0))
	
	if debug_sets[current_debug_set] == "Drivetrain" or debug_sets[current_debug_set] == "All":
		debug_ui.draw_debug_line("front_torque", center_axle, (front_axle - center_axle) * vehicle.true_torque_split , Color.ORANGE)
		debug_ui.draw_debug_line("rear_torque", center_axle, (rear_axle - center_axle) * (1.0 - vehicle.true_torque_split) , Color.ORANGE)
		debug_ui.draw_debug_line("fl_split", front_axle, (vehicle.front_left_wheel.global_position - front_axle) * ((vehicle.front_axle.applied_split + 1.0) * 0.5) * vehicle.true_torque_split, Color.ORANGE)
		debug_ui.draw_debug_line("fr_split", front_axle, (vehicle.front_right_wheel.global_position - front_axle) * (1.0 - ((vehicle.front_axle.applied_split + 1.0) * 0.5)) * vehicle.true_torque_split, Color.ORANGE)
		debug_ui.draw_debug_line("rl_split", rear_axle, (vehicle.rear_left_wheel.global_position - rear_axle) * ((vehicle.rear_axle.applied_split + 1.0) * 0.5) * (1.0 - vehicle.true_torque_split), Color.ORANGE)
		debug_ui.draw_debug_line("rr_split", rear_axle, (vehicle.rear_right_wheel.global_position - rear_axle) * (1.0 - ((vehicle.rear_axle.applied_split + 1.0) * 0.5)) * (1.0 - vehicle.true_torque_split), Color.ORANGE)
	
	if debug_sets[current_debug_set] == "Stability" or debug_sets[current_debug_set] == "All":
		var normalized_vector := vehicle.stability_torque_vector.normalized()
		debug_ui.draw_debug_line("upright", center_of_gravity, Vector3(normalized_vector.z, 0.0, normalized_vector.x) * vehicle.stability_torque_vector.length() * 0.001, Color.MAGENTA)
		debug_ui.draw_debug_line("yaw_front", front_axle, (vehicle.global_transform.basis.x * vehicle.stability_yaw_torque * 0.001) / vehicle.stability_yaw_strength, Color.RED)
		debug_ui.draw_debug_line("yaw_rear", rear_axle, (-vehicle.global_transform.basis.x * vehicle.stability_yaw_torque * 0.001) / vehicle.stability_yaw_strength, Color.RED)
	
	for wheel in vehicle.wheel_array:
		var wheel_color := Color.GREEN
		if not wheel.limit_spin:
			wheel_color = Color.RED
		
		debug_ui.draw_debug_circle(wheel.name + "spring", wheel.global_position, 2.0, wheel_color)
		
		if debug_sets[current_debug_set] == "Suspension Forces" or debug_sets[current_debug_set] == "All":
			debug_ui.draw_debug_line(wheel.name + "spring_force", wheel.global_position, wheel.global_transform.basis.y * wheel.spring_force * 0.0002, Color.RED)
			debug_ui.draw_debug_line(wheel.name + "arb_force", wheel.global_position, wheel.global_transform.basis.y * (wheel.antiroll_force + wheel.damping_force) * 0.0002, Color.BLUE)
			debug_ui.draw_debug_line(wheel.name + "damp_force", wheel.global_position, wheel.global_transform.basis.y * wheel.damping_force * 0.0002, Color.YELLOW)
			debug_ui.draw_debug_line(wheel.name + "spring_length_back", wheel.global_position, -wheel.global_transform.basis.y, Color.DARK_GREEN, false, Vector2(4 * signf(wheel.position.x), 0))
			debug_ui.draw_debug_line(wheel.name + "spring_length", wheel.global_position, -wheel.global_transform.basis.y * (wheel.spring_current_length / wheel.spring_length), Color.GREEN, false, Vector2(4 * signf(wheel.position.x), 0))
			
		
		if debug_sets[current_debug_set] == "Tire Forces" or debug_sets[current_debug_set] == "All":
			var wheel_force : Vector3 = ((wheel.global_transform.basis.x * wheel.force_vector.x) + (wheel.global_transform.basis.z * wheel.force_vector.y)) * 0.0002
			var wheel_slip : Vector3 = ((wheel.global_transform.basis.x * -wheel.slip_vector.x) + (wheel.global_transform.basis.z * -wheel.slip_vector.y)) * 2.0
			debug_ui.draw_debug_line(wheel.name + "force", wheel.last_collision_point, wheel_force, Color.RED)
			debug_ui.draw_debug_line(wheel.name + "slip", wheel.last_collision_point, wheel_slip, Color.YELLOW)
	
func switch_debug_set(value : int):
	current_debug_set = value % debug_sets.size()
	debug_ui.clear_debug()
