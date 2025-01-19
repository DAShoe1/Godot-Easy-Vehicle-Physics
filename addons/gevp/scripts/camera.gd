extends Camera3D

## The distance, in meters, that the camera will be from the
## [Node3D] in [member follow_this].
@export var follow_distance = 5
## The height, in meters, that the camera will be from the
## [Node3D] in [member follow_this].
@export var follow_height = 2
## The speed, in meters per second, that the camera will
## move to reach the [Node3D] in [member follow_this].
@export var speed:=20.0
## The [Node3D] that the camera will follow.
@export var follow_this : Node3D

var start_rotation : Vector3
var start_position : Vector3

func _ready():
	start_rotation = rotation
	start_position = position

func _process(delta):
	var delta_v := global_transform.origin - follow_this.global_transform.origin
	delta_v.y = 0.0
	if (delta_v.length() > follow_distance):
		delta_v = delta_v.normalized() * follow_distance
		delta_v.y = follow_height
		global_position = follow_this.global_transform.origin + delta_v
	
	look_at(follow_this.global_transform.origin, Vector3.UP)
