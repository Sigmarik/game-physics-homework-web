extends RigidBody3D


@export var show_angular_velocity = true
@export var show_initial_angular_velocity = true

var custom_angular_velocity : Vector3 = Vector3.ZERO
@export var initial_angular_velocity : Vector3 = Vector3(0, 0.1, 1) * 5


func _ready() -> void:
	add_to_group("xpbd_bodies")
	custom_integrator = true

	custom_angular_velocity = initial_angular_velocity


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	# Do nothing
	pass


func _process(delta: float) -> void:
	if show_angular_velocity:
		DebugDraw3D.draw_arrow(global_position, global_position + custom_angular_velocity, Color.CYAN, 0.1, true)

	if show_initial_angular_velocity:
		DebugDraw3D.draw_arrow(global_position, global_position + initial_angular_velocity, Color.RED, 0.1, true)


func get_ws_inertia() -> Basis:
	return get_inverse_inertia_tensor().inverse()


func get_gyroscopic_term() -> Vector3:
	var momentum_derivative := -custom_angular_velocity.cross(get_ws_inertia() * custom_angular_velocity)
	return get_inverse_inertia_tensor() * momentum_derivative

func get_implicit_gyroscopic_term(delta: float) -> Vector3:
	# Get inertia tensor (world space) and its inverse
	var I: Basis = get_ws_inertia()
	var I_inv: Basis = get_inverse_inertia_tensor()

	# Current angular velocity
	var omega_old: Vector3 = custom_angular_velocity

	# Initial guess: use explicit result (optional, helps convergence)
	var omega_new: Vector3 = omega_old

	# Fixed-point iteration to solve:
	# omega_new = omega_old - delta * I_inv * (omega_new x (I * omega_new))
	for _i in range(8):  # 8 iterations are usually enough
		var torque_gyro: Vector3 = omega_new.cross(I * omega_new)
		var omega_next: Vector3 = omega_old - delta * (I_inv * torque_gyro)
		
		# Check for convergence
		if omega_next.distance_squared_to(omega_new) < 1e-12:
			omega_new = omega_next
			break
		omega_new = omega_next
    
	# Return angular acceleration (implicit gyroscopic term)
	return (omega_new - omega_old) / delta
