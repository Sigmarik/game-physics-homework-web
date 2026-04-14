extends Node

# --- Exported parameters ----------------------------------------------------
@export var gravity := Vector3(0, -9.8, 0)
@export var damping := 0.99
@export var spring_stiffness_override := -1.0
@export var collision_restitution := 0.2
@export var max_delta := 0.0025

# --- Internal data ----------------------------------------------------------
var time_since_last_tick = 0.0
var delta_time = 0.05
var last_frame_dt = 1.0

const DAMPING_K = 0.001

# ----------------------------------------------------------------------------
func _ready() -> void:
	pass

# Solve 3x3 linear system: A * x = b   (using Basis inverse)
func solve_linear_system(matrix: Basis, vector: Vector3) -> Vector3:
	return matrix.inverse() * vector

# Transform a vector by a 3x3 matrix: matrix * vector
func transform_vector(matrix: Basis, vector: Vector3) -> Vector3:
	return matrix * vector

# Helper: add two Basis matrices column‑wise
func add_basis(a: Basis, b: Basis) -> Basis:
	return Basis(
		a[0] + b[0],
		a[1] + b[1],
		a[2] + b[2]
	)

# ----------------------------------------------------------------------------
func _process(delta: float) -> void:
	time_since_last_tick += delta
	if time_since_last_tick < delta_time:
		return
	time_since_last_tick -= delta_time

	var points = get_tree().get_nodes_in_group("vbd_point")
	var planes = get_tree().get_nodes_in_group("vbd_plane")

	for point in points:
		var old_pos = point.global_position

		if point.fixed_in_place or point.dragging:
			point.planned_position = point.movable_node.global_position
			continue

		var old_last = point.last_frame_position
		var vel = (old_pos - old_last) / delta_time

		var extrapolated_position = old_pos + vel * delta_time + gravity * delta_time * delta_time
		point.planned_position = extrapolated_position

	for iteration in range(0, 10):
		for point in points:
			if point.fixed_in_place or point.dragging:
				continue
			var old_pos = point.global_position
			var old_last = point.last_frame_position
			var vel = (old_pos - old_last) / delta_time

			var extrapolated_position = old_pos + vel * delta_time + gravity * delta_time * delta_time

			var neohookean_info = point.get_neohookean_info()
			var energy_derivative: Vector3 = -point.get_spring_force() + neohookean_info.derivative
			var energy_hessian: Basis = add_basis(point.get_spring_hessian(), neohookean_info.hessian)

			# damping_hessian = energy_hessian * (DAMPING_K / delta_time)
			var damping_hessian = Basis(
				energy_hessian[0] * (DAMPING_K / delta_time),
				energy_hessian[1] * (DAMPING_K / delta_time),
				energy_hessian[2] * (DAMPING_K / delta_time)
			)

			var damping_force = transform_vector(damping_hessian, vel * delta_time)
			energy_derivative += damping_force

			# energy_hessian += damping_hessian
			energy_hessian = add_basis(energy_hessian, damping_hessian)

			var force_component = -point.mass / (delta_time * delta_time) * (old_pos - extrapolated_position) - energy_derivative

			var hessian_identity_multiplier: float = point.mass / (delta_time * delta_time)
			var hessian_identity_component = Basis(
				Basis.IDENTITY[0] * hessian_identity_multiplier,
				Basis.IDENTITY[1] * hessian_identity_multiplier,
				Basis.IDENTITY[2] * hessian_identity_multiplier
			)
			var hessian = add_basis(hessian_identity_component, energy_hessian)
			var delta_position = solve_linear_system(hessian, force_component)

			point.planned_position = old_pos + delta_position

	for point in points:
		var old_pos = point.global_position
		point.last_frame_position = old_pos
		point.global_position = point.planned_position

	last_frame_dt = delta