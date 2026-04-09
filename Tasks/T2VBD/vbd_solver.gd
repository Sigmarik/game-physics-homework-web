extends Node

# --- Exported parameters ----------------------------------------------------
@export var gravity := Vector3(0, -9.8 * 0, 0)  # Gravity acceleration
@export var damping := 0.99                     # Velocity damping factor
@export var spring_stiffness_override := -1.0   # If >=0, overrides point's SPRING_STIFFNESS
@export var collision_restitution := 0.2        # Bounciness when hitting a plane
@export var max_delta := 0.0025                  # Clamp delta to avoid instability

# --- Internal data ----------------------------------------------------------
var time_since_last_tick = 0.0
var delta_time = 0.05
var last_frame_dt = 1.0

# ----------------------------------------------------------------------------
func _ready() -> void:
	pass

func solve_linear_system(matrix: DenseMatrix, vector: Vector3) -> Vector3:
	# Convert the Vector3 to a column matrix (3x1) using VectorN
	var vec_n = VectorN.from_packed_array(PackedFloat64Array([vector.x, vector.y, vector.z]))
	var b_column = vec_n.column_vector()
	
	# Solve A * x = b, returns a DenseMatrix (3x1)
	var solution_matrix = matrix.solve(b_column)
	
	# Extract the components from the solution column matrix
	var x = solution_matrix.get_element(0, 0)
	var y = solution_matrix.get_element(1, 0)
	var z = solution_matrix.get_element(2, 0)
	
	return Vector3(x, y, z)

# ----------------------------------------------------------------------------
func _process(delta: float) -> void:
	time_since_last_tick += delta;
	if (time_since_last_tick < delta_time): return
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

	for iteration in range(0, 1):
		for point in points:
			if point.fixed_in_place or point.dragging: continue
			var old_pos = point.global_position
			var old_last = point.last_frame_position
			var vel = (old_pos - old_last) / delta_time
			
			var extrapolated_position = old_pos + vel * delta_time + gravity * delta_time * delta_time

			var force_component = -point.mass / (delta_time * delta_time) * (old_pos - extrapolated_position) + point.get_spring_force()

			var hessian_identity_multiplier: float = point.mass / (delta_time * delta_time)
			var hessian_identity_component = DenseMatrix.identity(3)
			hessian_identity_component.multiply_scaler_in_place(hessian_identity_multiplier)
			var hessian = hessian_identity_component.add_dense(point.get_spring_hessian())
			var delta_position = solve_linear_system(hessian, force_component)

			point.planned_position = old_pos + delta_position
	
	for point in points:
		var old_pos = point.global_position
		point.last_frame_position = old_pos
		point.global_position = point.planned_position

	last_frame_dt = delta
