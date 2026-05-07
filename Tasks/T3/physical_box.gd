class_name PhysicalBox
extends RigidBody3D


@export var show_angular_velocity = true
@export var show_initial_angular_velocity = true

@export var initial_angular_velocity : Vector3 = Vector3(0, 0.1, 1) * 5
var custom_angular_velocity : Vector3 = Vector3.ZERO
var custom_velocity : Vector3 = Vector3.ZERO

var old_position := Vector3.ZERO
var old_basis := Basis()

var extent := Vector3.ZERO

var constraints : Array[Constraint] = []
var temporary_constraints : Array[Constraint] = []

@export var add_spring_constraint := false


func reset_temporary_constraints() -> void:
	temporary_constraints.clear()


func get_extent() -> Vector3:
	var shape_node: CollisionShape3D = null
	for child in get_children():
		if child is CollisionShape3D:
			shape_node = child
			break
	if shape_node == null:
		push_error("PhysicalBox has no CollisionShape3D child.")
		return Vector3.ZERO

	var box_shape: BoxShape3D = shape_node.shape as BoxShape3D
	if box_shape == null:
		push_error("CollisionShape3D does not contain a BoxShape3D.")
		return Vector3.ZERO

	return box_shape.size * 0.5


func get_local_inertia_diag() -> Vector3:
	# 1) Find the box shape
	var shape_node: CollisionShape3D = null
	for child in get_children():
		if child is CollisionShape3D:
			shape_node = child
			break
	if shape_node == null:
		push_error("PhysicalBox has no CollisionShape3D child.")
		return Vector3.ZERO

	var box_shape: BoxShape3D = shape_node.shape as BoxShape3D
	if box_shape == null:
		push_error("CollisionShape3D does not contain a BoxShape3D.")
		return Vector3.ZERO

	# 2) Get the full dimensions (extents are half the size)
	var half_ext: Vector3 = box_shape.extents
	var w: float = half_ext.x * 2.0
	var h: float = half_ext.y * 2.0
	var d: float = half_ext.z * 2.0

	# 3) Mass – use the body’s mass property
	var m: float = mass

	# 4) Principal moments for a uniform box
	var Ix: float = (m / 12.0) * (h * h + d * d)
	var Iy: float = (m / 12.0) * (w * w + d * d)
	var Iz: float = (m / 12.0) * (w * w + h * h)

	return Vector3(Ix, Iy, Iz)


func _ready() -> void:
	if Engine.is_editor_hint():
		# Editor: do nothing permanent, just let _process draw
		return

	add_to_group("xpbd_bodies")
	custom_integrator = true

	custom_angular_velocity = initial_angular_velocity

	freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
	freeze = true

	if add_spring_constraint:
		var spring := SpringConstraint.new()
		spring.relative_shift = Vector3(0, 1, 0)
		constraints.append(spring)

	extent = get_extent()


# --- Safety first : only modify global transform when inside the tree ---
func apply_rot_difference(axis_angle : Vector3) -> void:
	if not is_inside_tree():
		return
	var angle = axis_angle.length()
	if angle > 0.0001:
		var axis = axis_angle.normalized()
		var rot = Basis(axis, angle)
		global_transform.basis = rot * global_transform.basis


static func rotation_difference(basis_a : Basis, basis_b : Basis) -> Vector3:
	var diff_basis := basis_b.orthonormalized() * basis_a.orthonormalized().inverse()
	var quat := diff_basis.get_rotation_quaternion().normalized()
	return quat.get_axis().normalized() * quat.get_angle()


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	# Do nothing
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO


func _process(delta: float) -> void:
	# Accessing global_position while out of tree causes the error, so guard it.
	if not is_inside_tree():
		return

	if show_angular_velocity:
		DebugDraw3D.draw_arrow(global_position, global_position + custom_angular_velocity, Color.CYAN, 0.1, true)

	if show_initial_angular_velocity:
		DebugDraw3D.draw_arrow(global_position, global_position + initial_angular_velocity, Color.RED, 0.1, true)


func get_ws_inertia() -> Basis:
	if not is_inside_tree():
		return Basis()
	var I_local_vec := get_local_inertia_diag()
	var I_local := Basis.from_scale(I_local_vec)
	var R := global_basis
	return R * I_local * R.transposed()


func get_inverse_inertia_ws() -> Basis:
	if not is_inside_tree():
		return Basis()
	var I_local_vec := get_local_inertia_diag()
	if I_local_vec.x == 0 or I_local_vec.y == 0 or I_local_vec.z == 0:
		push_error("Mass is zero -> infinite inertia.")
		return Basis()  # zero matrix

	var inv_diag := Vector3.ONE / I_local_vec
	var I_inv_local := Basis.from_scale(inv_diag)
	var R := global_basis
	return R * I_inv_local * R.transposed()


func get_gyroscopic_term() -> Vector3:
	if not is_inside_tree():
		return Vector3.ZERO
	var momentum_derivative := -custom_angular_velocity.cross(get_ws_inertia() * custom_angular_velocity)
	return get_inverse_inertia_ws() * momentum_derivative


func get_implicit_gyroscopic_term(delta: float) -> Vector3:
	if not is_inside_tree():
		return Vector3.ZERO
	# Get inertia tensor (world space) and its inverse
	var I: Basis = get_ws_inertia()
	var I_inv: Basis = get_inverse_inertia_ws()

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


static func basis_to_rotation_vector(basis: Basis) -> Vector3:
	# A Basis can contain scale, but a Quaternion represents a pure rotation.
	# Using orthonormalized() removes scale and skew for an accurate conversion.
	var quat := Quaternion(basis.orthonormalized())
	# get_angle() returns the float, get_axis() returns the Vector3 (which is normalized).
	return quat.get_axis() * quat.get_angle()


static func rotation_vector_to_basis(rot_vec: Vector3) -> Basis:
	var angle := rot_vec.length()
	# A zero rotation shouldn't cause a zero-length axis error.
	if angle == 0.0:
		return Basis()
	var axis := rot_vec.normalized()
	# Godot's Basis constructor creates a pure rotation matrix from an axis and angle.
	return Basis(axis, angle)


func reset_constraint_lambdas() -> void:
	for constraint in constraints + temporary_constraints:
		constraint.lambda = 0


func draw_constraints() -> void:
	# Constraint drawing might use global_position – guard it
	if not is_inside_tree():
		return
	for constraint in constraints + temporary_constraints:
		constraint.draw_constraint(self)


func iterate_constraints_xpbd(delta : float) -> void:
	if not is_inside_tree():
		return
	var linear_shift := Vector3.ZERO
	var rotation_shift := Vector3.ZERO
	for constraint in constraints + temporary_constraints:
		var shift := constraint.get_delta_and_update_lambda(self, delta)
		linear_shift += shift.positional
		rotation_shift += shift.angular
	global_position += linear_shift
	apply_rot_difference(rotation_shift)


func iterate_constraints_explicit(delta : float) -> void:
	if not is_inside_tree():
		return
	var linear_shift := Vector3.ZERO
	var rotation_shift := Vector3.ZERO
	for constraint in constraints + temporary_constraints:
		var value := constraint.get_constraint_value(self)
		var linear_gradient := constraint.get_positional_gradient(self)
		var angular_gradient := constraint.get_angular_gradient(self)
		if value > 0:
			linear_gradient *= -1
			angular_gradient *= -1

		linear_gradient /= constraint.compliance
		angular_gradient /= constraint.compliance
		linear_shift += linear_gradient
		rotation_shift += angular_gradient
	global_position += linear_shift * delta * delta / mass
	apply_rot_difference(get_inverse_inertia_ws() * rotation_shift * delta * delta)


func iterate_soft_constraints(delta: float) -> void:
	if not is_inside_tree():
		return
	var I_inv := get_inverse_inertia_ws()

	var linear_shift := Vector3.ZERO
	var rotation_shift := Vector3.ZERO

	for constraint in constraints + temporary_constraints:
		var beta := 1.0 / constraint.compliance

		# Constraint properties
		var C := constraint.get_constraint_value(self)
		var J_lin := constraint.get_positional_gradient(self)
		var J_ang := constraint.get_angular_gradient(self)

		# Relative velocity along constraint direction
		var Jv := J_lin.dot(custom_velocity) + J_ang.dot(custom_angular_velocity)

		# Effective mass (world‑space)
		var w: float = 0.0
		if mass > 0.0:
			w += J_lin.length_squared() / mass
		var I_inv_J_ang := I_inv * J_ang
		w += J_ang.dot(I_inv_J_ang)

		# Compliance (inverse stiffness) – stored in the constraint
		var gamma := constraint.softness

		# Equation: (delta * w + gamma) * lambda = -(Jv + (beta/delta) * C)
		var denom := delta * w + gamma
		if abs(denom) < 1e-12:
			continue

		var lambda_force := -(Jv + (beta / delta) * C) / denom

		# Apply velocity update: v += delta * M^{-1} * J^T * lambda
		var impulse_lin := (delta * lambda_force) * J_lin / mass  if mass > 0.0 else Vector3.ZERO
		var impulse_ang := (delta * lambda_force) * I_inv_J_ang

		linear_shift += impulse_lin
		rotation_shift += impulse_ang

	# Position integration using the new velocities
	global_position += linear_shift * delta * delta / mass
	apply_rot_difference(get_inverse_inertia_ws() * rotation_shift * delta * delta)

func apply_impulse_at_point(impulse : Vector3, local_point : Vector3) -> void:
	var global_arm := global_basis * local_point
	var linear_impulse := impulse.normalized() * impulse.dot(global_arm.normalized())
	var angular_impulse := impulse.cross(global_arm)
	custom_velocity += linear_impulse / mass
	custom_angular_velocity += get_inverse_inertia_ws() * angular_impulse

func resolve_velocity_difference_at_point(velocity_change : Vector3, local_point : Vector3) -> void:
	# custom_velocity += velocity_change
	var global_arm := global_basis * local_point
	var linear_impulse = velocity_change.normalized() * abs(velocity_change.dot(global_arm.normalized()))
	var angular_vel_change := -velocity_change.cross(global_arm)
	custom_velocity += linear_impulse
	
	if angular_vel_change.length_squared() > 0:
		var impulsed_change := get_inverse_inertia_ws() * angular_vel_change
		var corrective_multiplier := angular_vel_change.length() / impulsed_change.dot(angular_vel_change.normalized())
		custom_angular_velocity += impulsed_change * corrective_multiplier

func resolve_position_delta_at_point(pos_change : Vector3, local_point : Vector3) -> void:
	var global_arm := global_basis * local_point
	var linear_impulse = pos_change.normalized() * abs(pos_change.dot(global_arm.normalized()))
	var angular_vel_change := -pos_change.cross(global_arm)
	global_position += linear_impulse
	
	if angular_vel_change.length_squared() > 0:
		var impulsed_change := get_inverse_inertia_ws() * angular_vel_change
		var corrective_multiplier := angular_vel_change.length() / impulsed_change.dot(angular_vel_change.normalized())
		apply_rot_difference(impulsed_change * corrective_multiplier)

func get_velocity_at_point(local_point : Vector3) -> Vector3:
	var global_arm := global_basis * local_point
	return custom_velocity + custom_angular_velocity.cross(global_arm)

func resolve_constraints_using_sequential_impulses() -> void:
	for constraint in constraints + temporary_constraints:
		constraint.apply_sequential_impulses(self)


func get_normals() -> Array[Vector3]:
	return [(global_basis * Vector3(1, 0, 0)).normalized(), (global_basis * Vector3(0, 1, 0)).normalized(), (global_basis * Vector3(0, 0, 1)).normalized()]


func debug_draw() -> void:
	# var normal = 2
	# var edges = get_edges(normal)
	# for idx in range(0, 8, 2):
	# 	DebugDraw3D.draw_line(edges[idx], edges[idx + 1], Color.BLACK)
	pass


func get_vertices() -> Array[Vector3]:
	var vertices : Array[Vector3] = []
	for sgn_x in [-1, 1]:
		for sgn_y in [-1, 1]:
			for sgn_z in [-1, 1]:
				var sgn := Vector3(sgn_x, sgn_y, sgn_z)
				vertices.append(global_transform * (extent * sgn))
	return vertices


func sat_project(direction : Vector3) -> Array[float]:
	var result : Array[float] = []
	for vtx in get_vertices():
		result.append(vtx.dot(direction))
	return result


func get_edges(index : int) -> Array[Vector3]:
	if index == 0:
		var vertices : Array[Vector3] = []
		for sgn_y in [-1, 1]:
			for sgn_z in [-1, 1]:
				for sgn_x in [-1, 1]:
					var sgn := Vector3(sgn_x, sgn_y, sgn_z)
					vertices.append(global_transform * (extent * sgn))
		return vertices
	if index == 1:
		var vertices : Array[Vector3] = []
		for sgn_z in [-1, 1]:
			for sgn_x in [-1, 1]:
				for sgn_y in [-1, 1]:
					var sgn := Vector3(sgn_x, sgn_y, sgn_z)
					vertices.append(global_transform * (extent * sgn))
		return vertices
	if index == 2:
		var vertices : Array[Vector3] = []
		for sgn_x in [-1, 1]:
			for sgn_y in [-1, 1]:
				for sgn_z in [-1, 1]:
					var sgn := Vector3(sgn_x, sgn_y, sgn_z)
					vertices.append(global_transform * (extent * sgn))
		return vertices
	return get_vertices()


static func array_min(arr : Array[float]) -> float:
	var result : float = INF
	for elem in arr:
		result = min(result, elem)
	return result

static func array_max(arr : Array[float]) -> float:
	var result : float = -INF
	for elem in arr:
		result = max(result, elem)
	return result


static func sat_difference(alpha_projection : Array[float], beta_projection : Array[float]) -> float:
	var alpha_min : float = array_min(alpha_projection)
	var alpha_max : float = array_max(alpha_projection)
	var beta_min : float = array_min(beta_projection)
	var beta_max : float = array_max(beta_projection)

	var penetration_depth : float = min(beta_max - alpha_min, alpha_max - beta_min)
	if penetration_depth <= 0:
		return 0
	
	if beta_max - alpha_min < alpha_max - beta_min:
		return penetration_depth
	else:
		return -penetration_depth


static func get_max_index(arr) -> int:
	if arr.is_empty():
		return -1

	var max_val = arr[0]
	var max_idx = 0

	for i in range(1, arr.size()):
		if arr[i] > max_val:
			max_val = arr[i]
			max_idx = i
			
	return max_idx

static func get_min_index(arr) -> int:
	if arr.is_empty():
		return -1

	var max_val = arr[0]
	var max_idx = 0

	for i in range(1, arr.size()):
		if arr[i] < max_val:
			max_val = arr[i]
			max_idx = i
			
	return max_idx


func sat_face_corner(beta : PhysicalBox) -> Array[Vector3]:
	var shortest_intersection : float = INF
	var this_point := Vector3.ZERO
	var that_point := Vector3.ZERO
	for direction in get_normals():
		var alpha_projection := sat_project(direction)
		var beta_projection := beta.sat_project(direction)
		var difference := sat_difference(alpha_projection, beta_projection)
		if difference == 0:
			# var plane := Plane((global_position + beta.global_position) * 0.5, direction)
			# DebugDraw3D.draw_plane(plane, Color.LIME)
			return []
		if abs(difference) < shortest_intersection:
			shortest_intersection = abs(difference)
			var index : int = get_max_index(beta_projection) if difference > 0 else get_min_index(beta_projection)
			that_point = beta.get_vertices()[index]
			this_point = that_point - direction * difference
	if shortest_intersection == INF: return []
	return [this_point, that_point]


static func get_closest_points(line_a_start: Vector3, line_a_end: Vector3, line_b_start: Vector3, line_b_end: Vector3) -> Array[Vector3]:
	var dir_a := line_a_end - line_a_start
	var dir_b := line_b_end - line_b_start
	var offset := line_a_start - line_b_start
	var aa := dir_a.length_squared()
	var bb := dir_b.length_squared()
	var ab := dir_a.dot(dir_b)
	var a_dot_offset := dir_a.dot(offset)
	var b_dot_offset := dir_b.dot(offset)
	var denom := aa * bb - ab * ab   # > 0 (lines not parallel)
	var param_a := (ab * b_dot_offset - bb * a_dot_offset) / denom
	var param_b := (aa * b_dot_offset - ab * a_dot_offset) / denom
	return [line_a_start + param_a * dir_a, line_b_start + param_b * dir_b]


static func sat_edge_edge(alpha: PhysicalBox, beta: PhysicalBox) -> Array[Vector3]:
	var shortest_intersection : float = INF
	var points : Array[Vector3] = []

	var alpha_normals := alpha.get_normals()
	var beta_normals := beta.get_normals()

	var line_a_start := Vector3.ZERO
	var line_b_start := Vector3.ZERO
	var line_a_end := Vector3.ZERO
	var line_b_end := Vector3.ZERO

	for dir_alpha_idx in range(alpha_normals.size()):
		for dir_beta_idx in range(beta_normals.size()):
			var dir_alpha := alpha_normals[dir_alpha_idx]
			var dir_beta := beta_normals[dir_beta_idx]
			var direction := dir_alpha.cross(dir_beta)
			if direction.length() == 0: continue
			direction = direction.normalized()

			var alpha_projection := alpha.sat_project(direction)
			var beta_projection := beta.sat_project(direction)
			var difference := sat_difference(alpha_projection, beta_projection)
			if difference == 0:
				# var plane := Plane((alpha.global_position + beta.global_position) * 0.5, direction)
				# DebugDraw3D.draw_plane(plane, Color.LIME)
				return []
			if abs(difference) >= shortest_intersection: continue

			var alpha_edges := alpha.get_edges(dir_alpha_idx)
			var beta_edges := beta.get_edges(dir_beta_idx)

			var edge_idx_alpha := 0
			var edge_idx_beta := 0
			for idx in range(0, alpha_edges.size(), 2):
				if alpha_edges[idx].dot(direction) > alpha_edges[edge_idx_alpha].dot(direction) and difference > 0: continue
				edge_idx_alpha = idx
			for idx in range(0, beta_edges.size(), 2):
				if beta_edges[idx].dot(direction) < beta_edges[edge_idx_beta].dot(direction) and difference > 0: continue
				edge_idx_beta = idx

			line_a_start = alpha_edges[edge_idx_alpha]
			line_a_end = alpha_edges[edge_idx_alpha + 1]
			line_b_start = beta_edges[edge_idx_beta]
			line_b_end = beta_edges[edge_idx_beta + 1]
			points = get_closest_points(line_a_start, line_a_end, line_b_start, line_b_end)

			shortest_intersection = abs(difference)
	# DebugDraw3D.draw_line(line_a_start, line_a_end, Color.ORANGE)
	# DebugDraw3D.draw_line(line_b_start, line_b_end, Color.ORANGE)
	return points
			

static func sat_detect(alpha: PhysicalBox, beta: PhysicalBox) -> Array[Vector3]:
	var face_corner_alpha := alpha.sat_face_corner(beta)
	if face_corner_alpha.is_empty(): return []
	var face_corner_beta := beta.sat_face_corner(alpha)
	if face_corner_beta.is_empty(): return []
	var edge_edge := sat_edge_edge(alpha, beta)
	if edge_edge.is_empty(): return []

	var closest_pair := face_corner_alpha
	var best_distance := face_corner_alpha[0].distance_squared_to(face_corner_alpha[1])
	for pair in [face_corner_beta, edge_edge]:
		var distance = pair[0].distance_squared_to(pair[1])
		if distance > best_distance: continue
		closest_pair = pair

	return closest_pair


static func add_optional_collision_constraints(alpha: PhysicalBox, beta: PhysicalBox) -> void:
	# Use our robust SAT detection that correctly handles edge‑to‑edge contacts
	var contacts := sat_detect(alpha, beta)
	if contacts.is_empty():
		return

	var world_point_alpha: Vector3 = contacts[0]
	var world_point_beta: Vector3 = contacts[1]
	# DebugDraw3D.draw_sphere(world_point_alpha, 0.05, Color.YELLOW)
	# DebugDraw3D.draw_sphere(world_point_beta, 0.05, Color.YELLOW)

	var local_alpha_point := alpha.global_transform.affine_inverse() * world_point_alpha
	var local_beta_point  := beta.global_transform.affine_inverse() * world_point_beta

	# From alpha's view, anchor is beta's point
	var ac = PivotConstraint.new()
	ac.anchor_object = beta
	ac.anchor_position = local_beta_point
	ac.relative_shift = local_alpha_point
	ac.allowed_length = 0.0
	ac.compliance = 0.001
	alpha.temporary_constraints.append(ac)

	# Symmetric constraint from beta's view
	var bc = PivotConstraint.new()
	bc.anchor_object = alpha
	bc.anchor_position = local_alpha_point
	bc.relative_shift = local_beta_point
	bc.allowed_length = 0.0
	bc.compliance = 0.001
	beta.temporary_constraints.append(bc)
