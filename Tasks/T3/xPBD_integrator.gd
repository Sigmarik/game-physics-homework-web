extends Node3D

var bodies: Array[PhysicalBox]

@export var apply_gyro_term = true
@export var use_implicit_term = true

var paused := false

enum ConstraintResolutionMode
{
	xPBD,
	explicit_forces,
	soft_constraints,
	sequential_impulses
}

enum CollisionDetectionMode
{
	none,
	each_to_each,
	spatial_grid,
	sweep_and_prune,
	bounding_volume_hierarchy
}

@export var constraint_mode := ConstraintResolutionMode.xPBD
@export var collision_mode := CollisionDetectionMode.none

func update_rigid_body_list():
	var unfiltered_bodies = get_tree().get_nodes_in_group("xpbd_bodies")
	bodies.clear()
	for body in unfiltered_bodies:
		if body is PhysicalBox:
			bodies.append(body)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	update_rigid_body_list.call_deferred()


func _input(event):
	if event.is_action_pressed("ui_accept"):
		paused = not paused


func _process(_delta: float) -> void:
	for body in bodies:
		if body.show_angular_velocity:
			DebugDraw3D.draw_arrow(body.global_position, body.global_position + body.custom_angular_velocity, Color.CYAN, 0.1, true)

		if body.show_initial_angular_velocity:
			DebugDraw3D.draw_arrow(body.global_position, body.global_position + body.initial_angular_velocity, Color.RED, 0.1, true)

	for body in bodies:
		body.draw_constraints()
		body.debug_draw()


func detect_collisions_per_object_pair() -> void:
	for i in range(bodies.size() - 1):
		for j in range(i + 1, bodies.size()):
			var alpha := bodies[i]
			var beta := bodies[j]
			PhysicalBox.add_optional_collision_constraints(alpha, beta)


func detect_collisions_gridded() -> void:
	if bodies.size() < 2: return

	var grid_size : float = 0
	for body in bodies:
		grid_size += max(body.size.x, body.size.y, body.size.z) * 0.5
	grid_size /= bodies.size()

	var omni_bodies : Array[PhysicalBox] = []
	var regular_bodies : Array[PhysicalBox] = []
	var min_corner : Vector3
	var first_regular := true

	for body in bodies:
		if max(body.size.x, body.size.y, body.size.z) > grid_size * 3:
			omni_bodies.append(body)
		else:
			regular_bodies.append(body)
			if first_regular:
				min_corner = body.global_position
				first_regular = false
			else:
				min_corner = min_corner.min(body.global_position)

	if omni_bodies.size() >= 2:
		for i in range(omni_bodies.size() - 1):
			for j in range(i + 1, omni_bodies.size()):
				PhysicalBox.add_optional_collision_constraints(omni_bodies[i], omni_bodies[j])

	if regular_bodies.size() > 0:
		var grid_origin := min_corner
		var grid : Dictionary = {}

		for body in regular_bodies:
			var grid_position := Vector3i(
				floor((body.global_position.x - grid_origin.x) / grid_size),
				floor((body.global_position.y - grid_origin.y) / grid_size),
				floor((body.global_position.z - grid_origin.z) / grid_size)
			)
			body.grid_position = grid_position
			if not grid.has(grid_position):
				grid[grid_position] = []
			grid[grid_position].append(body)

		# for cell in grid.keys():
		# 	var cell_min = grid_origin + Vector3(cell.x, cell.y, cell.z) * grid_size
		# 	var cell_aabb = AABB(cell_min, Vector3(grid_size, grid_size, grid_size))
		# 	DebugDraw3D.draw_aabb(cell_aabb, Color.BLACK)

		var processed_pairs : Dictionary = {}
		for body_a in regular_bodies:
			for dx in range(-1, 2):
				for dy in range(-1, 2):
					for dz in range(-1, 2):
						var check_cell = body_a.grid_position + Vector3i(dx, dy, dz)
						if grid.has(check_cell):
							for body_b in grid[check_cell]:
								if body_a == body_b:
									continue
								var id_a = body_a.get_instance_id()
								var id_b = body_b.get_instance_id()
								var pair_key = Vector2i(min(id_a, id_b), max(id_a, id_b))
								if not processed_pairs.has(pair_key):
									processed_pairs[pair_key] = true
									PhysicalBox.add_optional_collision_constraints(body_a, body_b)

	for omni in omni_bodies:
		for regular in regular_bodies:
			PhysicalBox.add_optional_collision_constraints(omni, regular)


func detect_collisions_sweep_and_prune() -> void:
	if bodies.size() < 2: return

	var processed_pairs : Dictionary = {}
	var intervals : Array = []

	for body in bodies:
		var aabb = body.get_aabb()
		var min_x = aabb.position.x
		var max_x = aabb.position.x + aabb.size.x
		intervals.append([min_x, max_x, body, aabb])

	intervals.sort_custom(func(left, right): return left[0] < right[0])

	for index_a in range(intervals.size()):
		var interval_a = intervals[index_a]
		for index_b in range(index_a + 1, intervals.size()):
			var interval_b = intervals[index_b]
			if interval_b[0] > interval_a[1]:
				break
			if interval_a[3].intersects(interval_b[3]):
				var body_a : PhysicalBox = interval_a[2]
				var body_b : PhysicalBox = interval_b[2]
				var id_a = body_a.get_instance_id()
				var id_b = body_b.get_instance_id()
				var pair_key = Vector2i(min(id_a, id_b), max(id_a, id_b))
				if not processed_pairs.has(pair_key):
					processed_pairs[pair_key] = true
					PhysicalBox.add_optional_collision_constraints(body_a, body_b)


func detect_collisions_lbvh() -> void:
	if bodies.size() < 2: return

	# --- Compute centroids and bounding box of all centroids ---
	var centroid_min := bodies[0].get_aabb().get_center()
	var centroid_max := centroid_min
	var centroids : Array[Vector3] = []
	for body in bodies:
		var center := body.get_aabb().get_center()
		centroids.append(center)
		centroid_min = centroid_min.min(center)
		centroid_max = centroid_max.max(center)

	var epsilon := 0.001
	var range_x = max(centroid_max.x - centroid_min.x, epsilon)
	var range_y = max(centroid_max.y - centroid_min.y, epsilon)
	var range_z = max(centroid_max.z - centroid_min.z, epsilon)
	var scale_x = 1023.0 / range_x
	var scale_y = 1023.0 / range_y
	var scale_z = 1023.0 / range_z

	# --- Assign Morton-like codes (concatenated 10‑bit coordinates) ---
	var morton_items := []
	for index in range(bodies.size()):
		var center := centroids[index]
		var x_norm := int(clamp((center.x - centroid_min.x) * scale_x, 0.0, 1023.0))
		var y_norm := int(clamp((center.y - centroid_min.y) * scale_y, 0.0, 1023.0))
		var z_norm := int(clamp((center.z - centroid_min.z) * scale_z, 0.0, 1023.0))
		var code := (x_norm << 20) | (y_norm << 10) | z_norm
		morton_items.append({"code": code, "body_index": index, "aabb": bodies[index].get_aabb()})

	morton_items.sort_custom(_compare_morton)

	# --- Build BVH tree ---
	var root_node = _build_lbvh_tree(morton_items, 0, morton_items.size() - 1)

	# --- Traverse BVH to register collision constraints ---
	var processed_pairs : Dictionary = {}
	_traverse_lbvh_self(root_node, bodies, processed_pairs)


# --- LBVH helper functions (defined as class members) ---

func _compare_morton(left, right) -> bool:
	return left.code < right.code


func _build_lbvh_tree(morton_items : Array, start_index : int, end_index : int):
	if start_index == end_index:
		var item = morton_items[start_index]
		return {"is_leaf": true, "body_index": item.body_index, "aabb": item.aabb}
	var mid := (start_index + end_index) / 2
	var left_node = _build_lbvh_tree(morton_items, start_index, mid)
	var right_node = _build_lbvh_tree(morton_items, mid + 1, end_index)
	var merged_aabb = left_node.aabb.merge(right_node.aabb)
	return {"is_leaf": false, "left": left_node, "right": right_node, "aabb": merged_aabb}


func _traverse_lbvh_self(node, bodies : Array, processed_pairs : Dictionary) -> void:
	if not node.is_leaf:
		_traverse_lbvh_self(node.left, bodies, processed_pairs)
		_traverse_lbvh_self(node.right, bodies, processed_pairs)
		_traverse_lbvh_pair(node.left, node.right, bodies, processed_pairs)


func _traverse_lbvh_pair(node_a, node_b, bodies : Array, processed_pairs : Dictionary) -> void:
	if not node_a.aabb.intersects(node_b.aabb): return

	if node_a.is_leaf and node_b.is_leaf:
		if node_a.body_index == node_b.body_index: return
		var body_a = bodies[node_a.body_index]
		var body_b = bodies[node_b.body_index]
		var id_a = body_a.get_instance_id()
		var id_b = body_b.get_instance_id()
		var pair_key = Vector2i(min(id_a, id_b), max(id_a, id_b))
		if not processed_pairs.has(pair_key):
			processed_pairs[pair_key] = true
			PhysicalBox.add_optional_collision_constraints(body_a, body_b)
		return

	if node_a.is_leaf:
		_traverse_lbvh_pair(node_a, node_b.left, bodies, processed_pairs)
		_traverse_lbvh_pair(node_a, node_b.right, bodies, processed_pairs)
	elif node_b.is_leaf:
		_traverse_lbvh_pair(node_a.left, node_b, bodies, processed_pairs)
		_traverse_lbvh_pair(node_a.right, node_b, bodies, processed_pairs)
	else:
		_traverse_lbvh_pair(node_a.left, node_b.left, bodies, processed_pairs)
		_traverse_lbvh_pair(node_a.left, node_b.right, bodies, processed_pairs)
		_traverse_lbvh_pair(node_a.right, node_b.left, bodies, processed_pairs)
		_traverse_lbvh_pair(node_a.right, node_b.right, bodies, processed_pairs)


func detect_collisions() -> void:
	if collision_mode == CollisionDetectionMode.none: return
	if bodies.size() < 2: return

	if collision_mode == CollisionDetectionMode.each_to_each: detect_collisions_per_object_pair()
	if collision_mode == CollisionDetectionMode.spatial_grid: detect_collisions_gridded()
	if collision_mode == CollisionDetectionMode.sweep_and_prune: detect_collisions_sweep_and_prune()
	if collision_mode == CollisionDetectionMode.bounding_volume_hierarchy: detect_collisions_lbvh()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta: float) -> void:
	if paused: return

	for body in bodies:
		body.reset_temporary_constraints()

	for body in bodies:
		if body.stationary: continue

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
		body.custom_velocity += body.gravity_scale * Vector3(0, -9.8, 0) * 0.5 * delta
		body.global_position += body.custom_velocity * delta

		body.reset_constraint_lambdas()

	detect_collisions()
	
	if constraint_mode == ConstraintResolutionMode.xPBD:
		for idx in range(15):
			for body in bodies:
				if body.stationary: continue
				body.iterate_constraints_xpbd(delta)
				body.update_velocities(delta)
	elif constraint_mode == ConstraintResolutionMode.explicit_forces:
		for body in bodies:
			if body.stationary: continue
			body.iterate_constraints_explicit(delta)
	elif constraint_mode == ConstraintResolutionMode.soft_constraints:
		for body in bodies:
			if body.stationary: continue
			body.iterate_soft_constraints(delta)
			# body.update_velocities(delta)
	elif constraint_mode == ConstraintResolutionMode.sequential_impulses:
		for idx in range(2):
			for body in bodies:
				if body.stationary: continue
				body.resolve_constraints_using_sequential_impulses()
		for body in bodies:
			# Simulate a bit of damping present in XPBD
			body.custom_velocity *= 0.995
			body.custom_angular_velocity *= 0.995

	
	if constraint_mode != ConstraintResolutionMode.sequential_impulses:
		for body in bodies:
			if body.stationary: continue
			body.update_velocities(delta)
