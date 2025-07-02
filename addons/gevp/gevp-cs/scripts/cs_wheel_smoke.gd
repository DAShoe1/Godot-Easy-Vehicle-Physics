extends GPUParticles3D

@export var vehicle : GEVPVehicle

@export var longitudinal_slip_threshold := 0.5
@export var lateral_slip_threshold := 1.0

func _process(delta):
    if is_instance_valid(vehicle):
        for wheel : GEVPWheel in vehicle.WheelArray:
            if absf(wheel.SlipVector.x) > lateral_slip_threshold or absf(wheel.SlipVector.y) > longitudinal_slip_threshold:
                var smoke_transform : Transform3D = wheel.global_transform
                smoke_transform.origin = wheel.LastCollisionPoint
                emit_particle(smoke_transform,  wheel.global_transform.basis * ((wheel.LocalVelocity * 0.2) - (Vector3.FORWARD * wheel.Spin * wheel.TireRadius * 0.2)) * self.global_transform.basis, Color.WHITE, Color.WHITE, 5) #EMIT_FLAG_POSITION + EMIT_FLAG_VELOCITY)
