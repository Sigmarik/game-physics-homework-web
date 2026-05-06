class_name PivotConstraint
extends Constraint

var relative_shift: Vector3 = Vector3.ZERO
var anchor_position: Vector3 = Vector3.ZERO
var anchor_object: Node3D = null
var allowed_length: float = 0

func get_anchor_pos() -> Vector3:
    return anchor_object.global_transform * anchor_position if anchor_object else anchor_position

func capture_distance(node: PhysicalBox) -> void:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    allowed_length = get_anchor_pos().distance_to(global_pivot)

func get_constraint_value(node: PhysicalBox) -> float:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    return global_pivot.distance_to(get_anchor_pos()) - allowed_length

func get_positional_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var delta := get_anchor_pos() - global_pivot
    var dist := delta.length()
    if dist < 1e-9:
        return Vector3.ZERO
    return -delta / dist   # unit direction from pivot to anchor

func get_angular_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var delta := get_anchor_pos() - global_pivot
    var dist := delta.length()
    if dist < 1e-9:
        return Vector3.ZERO
    return -pivot_shift.cross(delta / dist)

func draw_constraint(node : PhysicalBox) -> void:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var anchor := get_anchor_pos()
    DebugDraw3D.draw_line(global_pivot, anchor, Color.RED)

    # if not Engine.is_editor_hint():
    #     var velocity = node.get_velocity_at_point(relative_shift)
    #     DebugDraw3D.draw_arrow(global_pivot, global_pivot + velocity, Color.GREEN_YELLOW, 0.1, true)

# What a nice and non-convoluted way of resolving rigid constraints.
func apply_sequential_impulses(node : PhysicalBox) -> void:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var delta := get_anchor_pos() - global_pivot
    var normal := delta.normalized()
    var position_shift := delta.normalized() * (allowed_length - delta.length())

    var anchor_velocity := Vector3.ZERO
    var point_velocity := node.get_velocity_at_point(relative_shift)
    var relative_velocity := point_velocity

    var this_fraction : float = 1
    var that_fraction : float = 0
    if anchor_object is PhysicalBox:
        var anchor_box := anchor_object as PhysicalBox
        anchor_velocity = anchor_box.get_velocity_at_point(anchor_position)
        relative_velocity -= anchor_velocity
        # Yup, I am not doing inertia consideration for impulses. Sue me.
        this_fraction = anchor_box.mass / (anchor_box.mass + node.mass)
        that_fraction = node.mass / (anchor_box.mass + node.mass)

    var projected_vel_difference := relative_velocity.project(normal)

    DebugDraw3D.draw_arrow(global_pivot, global_pivot - position_shift, Color.BLUE, 0.1, true)
    node.resolve_position_delta_at_point(-position_shift * this_fraction, relative_shift)
    if anchor_object is PhysicalBox:
        var anchor_box := anchor_object as PhysicalBox
        anchor_box.resolve_position_delta_at_point(position_shift * that_fraction, anchor_position)

    if (projected_vel_difference.dot(delta) > 0) == (delta.length() > allowed_length):
        return

    node.resolve_velocity_difference_at_point(-projected_vel_difference * this_fraction, relative_shift)
    if anchor_object is PhysicalBox:
        var anchor_box := anchor_object as PhysicalBox
        anchor_box.resolve_velocity_difference_at_point(projected_vel_difference * that_fraction, anchor_position)
        anchor_box.resolve_position_delta_at_point(-position_shift * that_fraction, anchor_position)

