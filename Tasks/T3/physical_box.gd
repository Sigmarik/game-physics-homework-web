extends RigidBody3D


var custom_angular_velocity : Vector3 = Vector3.ZERO


func _ready() -> void:
	add_to_group("xpbd_bodies")
	custom_integrator = true

	var spin_speed := 5.0
	var initial_omega := Vector3(0, 0.1, 1) * spin_speed
	custom_angular_velocity = initial_omega


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	# Do nothing
	pass


func _process(delta: float) -> void:
	DebugDraw3D.draw_arrow(global_position, global_position + custom_angular_velocity, Color.CYAN, 0.1, true)


func get_ws_inertia() -> Basis:
	return get_inverse_inertia_tensor().inverse()


func get_gyroscopic_term() -> Vector3:
	var momentum_derivative := -custom_angular_velocity.cross(get_ws_inertia() * custom_angular_velocity)
	return get_inverse_inertia_tensor() * momentum_derivative
