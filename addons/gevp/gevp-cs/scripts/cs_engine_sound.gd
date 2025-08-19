extends AudioStreamPlayer3D

@export var vehicle : GEVPVehicle
@export var sample_rpm := 4000.0

func _physics_process(delta):
    pitch_scale = vehicle.MotorRpm / sample_rpm
    volume_db = linear_to_db((vehicle.ThrottleAmount * 0.5) + 0.5)
