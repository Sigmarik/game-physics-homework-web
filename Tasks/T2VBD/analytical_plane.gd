extends MeshInstance3D

func get_plane() -> Plane:
	var a = (global_transform * Vector3(1, 0, 0)).normalized()
	var b = (global_transform * Vector3(0, 1, 0)).normalized()
	var normal = a.cross(b).normalized()
	return Plane(normal, global_transform.origin)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
