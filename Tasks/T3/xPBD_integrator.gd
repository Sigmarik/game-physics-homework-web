extends Node3D

var bodies: Array[PhysicalBox]

@export var apply_gyro_term = true
@export var use_implicit_term = true

func update_rigid_body_list():
	var unfiltered_bodies = get_tree().get_nodes_in_group("xpbd_bodies")
	bodies.clear()
	for body in unfiltered_bodies:
		if body is PhysicalBox:
			bodies.append(body)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	update_rigid_body_list.call_deferred()


func _process(_delta: float) -> void:
	for body in bodies:
		if body.show_angular_velocity:
			DebugDraw3D.draw_arrow(body.global_position, body.global_position + body.custom_angular_velocity, Color.CYAN, 0.1, true)

		if body.show_initial_angular_velocity:
			DebugDraw3D.draw_arrow(body.global_position, body.global_position + body.initial_angular_velocity, Color.RED, 0.1, true)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta: float) -> void:
	for body in bodies:
		body.old_basis = body.global_basis
		body.old_position = body.global_position

		if apply_gyro_term:
			var gyroscopic_term : Vector3 = Vector3.ZERO
			if use_implicit_term:
				gyroscopic_term = body.get_implicit_gyroscopic_term(delta)
			else:
				gyroscopic_term = body.get_gyroscopic_term()
			body.custom_angular_velocity += gyroscopic_term * delta
		body.apply_rot_difference(body.custom_angular_velocity * delta)
		body.global_position += body.custom_velocity * delta + delta * delta * body.gravity_scale * Vector3(0, -9.8, 0) * 0.5

		body.reset_constraint_lambdas()
	
	for idx in range(10):
		for body in bodies:
			body.iterate_constraints(delta)
	
	for body in bodies:
		body.custom_angular_velocity = PhysicalBox.rotation_difference(body.old_basis, body.global_basis) / delta
		body.custom_velocity = (body.global_position - body.old_position) / delta
