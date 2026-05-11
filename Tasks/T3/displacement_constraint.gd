class_name DisplacementConstraint
extends Constraint

var relative_shift: Vector3 = Vector3.ZERO
var anchor_position: Vector3 = Vector3.ZERO
var anchor_object: Node3D = null
var world_normal := Vector3.ZERO

func get_anchor_pos() -> Vector3:
	return anchor_object.global_transform * anchor_position if anchor_object else anchor_position

func get_constraint_value(node: PhysicalBox) -> float:
	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var delta := get_anchor_pos() - global_pivot

	return max(0, delta.dot(world_normal))

func get_positional_gradient(node: PhysicalBox) -> Vector3:
	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var delta := get_anchor_pos() - global_pivot
	delta = delta.project(world_normal)
	var dot := delta.dot(world_normal)
	var dist := delta.length()

	if dot < 0:
		return Vector3.ZERO
	return -delta / dist * abs(pivot_shift.normalized().dot(delta.normalized()))


func get_angular_gradient(node: PhysicalBox) -> Vector3:
	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var delta := get_anchor_pos() - global_pivot
	delta = delta.project(world_normal)
	var dot := delta.dot(world_normal)
	var dist := delta.length()
	if dot < 0:
		return Vector3.ZERO
	return -pivot_shift.cross(delta / dist)

func apply_friction_adjustment(node: PhysicalBox, dt: float) -> void:
	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var delta := get_anchor_pos() - global_pivot

	var anchor_velocity := Vector3.ZERO
	var point_velocity := node.get_velocity_at_point(relative_shift)
	var relative_velocity := point_velocity

	if anchor_object is PhysicalBox and !anchor_object.stationary:
		var anchor_box := anchor_object as PhysicalBox
		anchor_velocity = anchor_box.get_velocity_at_point(anchor_position)
		relative_velocity -= anchor_velocity

	var projected_vel_difference := relative_velocity.project(world_normal)
	var planar_vel_difference := relative_velocity - projected_vel_difference
	var planar_normalized := planar_vel_difference.normalized()
	var magnitude = min(planar_normalized.length(), projected_vel_difference.length() * 100)
	var correction = planar_normalized * magnitude
	# DebugDraw3D.draw_arrow(global_pivot, global_pivot + correction, Color.ORANGE_RED, 0.1, true)
	node.resolve_shift_at_point(-correction * dt * dt, relative_shift)

func draw_constraint(node : PhysicalBox) -> void:
	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var anchor := get_anchor_pos()
	if anchor.distance_squared_to(global_pivot) < 1e-9:
		return
	DebugDraw3D.draw_line(global_pivot, anchor, Color.RED)

# What a nice and non-convoluted way of resolving rigid constraints.
func apply_sequential_impulses(node : PhysicalBox) -> void:
	if get_constraint_value(node) == 0:
		return

	var pivot_shift := node.global_basis * relative_shift
	var global_pivot := node.global_position + pivot_shift
	var delta := get_anchor_pos() - global_pivot
	var normal := world_normal
	var position_shift := delta.project(world_normal)

	var anchor_velocity := Vector3.ZERO
	var point_velocity := node.get_velocity_at_point(relative_shift)
	var relative_velocity := point_velocity

	var this_fraction : float = 1
	var that_fraction : float = 0
	if anchor_object is PhysicalBox and !anchor_object.stationary:
		var anchor_box := anchor_object as PhysicalBox
		anchor_velocity = anchor_box.get_velocity_at_point(anchor_position)
		relative_velocity -= anchor_velocity
		# Yup, I am not doing inertia consideration for impulses. Sue me.
		this_fraction = anchor_box.mass / (anchor_box.mass + node.mass)
		that_fraction = node.mass / (anchor_box.mass + node.mass)

	var projected_vel_difference := relative_velocity.project(normal)

	if position_shift.dot(world_normal) > 0:
		node.resolve_shift_at_point(position_shift * this_fraction, relative_shift)
		if anchor_object is PhysicalBox and !anchor_object.stationary:
			var anchor_box := anchor_object as PhysicalBox
			anchor_box.resolve_shift_at_point(-position_shift * that_fraction, anchor_position)

	if position_shift.dot(projected_vel_difference) < 0:
		node.resolve_velocity_difference_at_point(-projected_vel_difference * this_fraction, relative_shift)
		if anchor_object is PhysicalBox and !anchor_object.stationary:
			var anchor_box := anchor_object as PhysicalBox
			anchor_box.resolve_velocity_difference_at_point(projected_vel_difference * that_fraction, anchor_position)
