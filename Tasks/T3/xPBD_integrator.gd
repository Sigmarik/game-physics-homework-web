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


func detect_collisions() -> void:
	if collision_mode == CollisionDetectionMode.none: return
	if bodies.size() < 2: return

	if collision_mode == CollisionDetectionMode.each_to_each: detect_collisions_per_object_pair()
	if collision_mode == CollisionDetectionMode.spatial_grid: detect_collisions_gridded()


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
