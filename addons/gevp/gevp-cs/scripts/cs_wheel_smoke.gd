extends GPUParticles3D

@export var vehicle : Vehicle

@export var longitudinal_slip_threshold := 0.5
@export var lateral_slip_threshold := 1.0

func _process(delta):
	if is_instance_valid(vehicle):
		for wheel in vehicle.wheel_array:
			if absf(wheel.slip_vector.x) > lateral_slip_threshold or absf(wheel.slip_vector.y) > longitudinal_slip_threshold:
				var smoke_transform : Transform3D = wheel.global_transform
				smoke_transform.origin = wheel.last_collision_point
				emit_particle(smoke_transform,  wheel.global_transform.basis * ((wheel.local_velocity * 0.2) - (Vector3.FORWARD * wheel.spin * wheel.tire_radius * 0.2)) * self.global_transform.basis, Color.WHITE, Color.WHITE, 5) #EMIT_FLAG_POSITION + EMIT_FLAG_VELOCITY)
