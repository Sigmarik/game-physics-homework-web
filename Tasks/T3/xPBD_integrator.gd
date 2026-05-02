extends Node3D

var bodies: Array[Node]

@export var apply_gyro_term = true
@export var use_implicit_term = true

func update_rigid_body_list():
	bodies = get_tree().get_nodes_in_group("xpbd_bodies")

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	update_rigid_body_list.call_deferred()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta: float) -> void:
	for body in bodies:
		if not body is RigidBody3D:
			continue

		if apply_gyro_term:
			var gyroscopic_term : Vector3 = Vector3.ZERO
			if use_implicit_term:
				gyroscopic_term = body.get_implicit_gyroscopic_term(delta)
			else:
				gyroscopic_term = body.get_gyroscopic_term()
			body.custom_angular_velocity += gyroscopic_term * delta
		var axis_angle = body.custom_angular_velocity * delta
		var angle = axis_angle.length()
		if angle > 0.0001:
			var axis = axis_angle.normalized()
			var rot = Basis(axis, angle)
			body.global_transform.basis = rot * body.global_transform.basis
