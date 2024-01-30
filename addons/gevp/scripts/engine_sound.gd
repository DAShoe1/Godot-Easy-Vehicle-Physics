extends AudioStreamPlayer3D

@export var vehicle : Vehicle
@export var sample_rpm := 4000.0

func _physics_process(delta):
	pitch_scale = vehicle.motor_rpm / sample_rpm
	volume_db = linear_to_db((vehicle.throttle_amount * 0.5) + 0.5)
