class_name PivotConstraint
extends Constraint

var relative_shift: Vector3 = Vector3.ZERO
var anchor_position: Vector3 = Vector3.ZERO
var anchor_object: Node3D = null

func get_anchor_pos() -> Vector3:
    return anchor_object.global_transform * anchor_position if anchor_object else anchor_position

func get_constraint_value(node: PhysicalBox) -> float:
    var pivot_shift = node.global_basis * relative_shift
    var global_pivot = node.global_position + pivot_shift
    return global_pivot.distance_to(get_anchor_pos())   # C = distance

func get_positional_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift = node.global_basis * relative_shift
    var global_pivot = node.global_position + pivot_shift
    var delta = get_anchor_pos() - global_pivot
    var dist = delta.length()
    if dist < 1e-9:
        return Vector3.ZERO
    return -delta / dist   # unit direction from pivot to anchor

func get_angular_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift = node.global_basis * relative_shift
    var global_pivot = node.global_position + pivot_shift
    var delta = get_anchor_pos() - global_pivot
    var dist = delta.length()
    if dist < 1e-9:
        return Vector3.ZERO
    return -pivot_shift.cross(delta / dist)

func draw_constraint(node : PhysicalBox) -> void:
    var pivot_shift = node.global_basis * relative_shift
    var global_pivot = node.global_position + pivot_shift
    var anchor = get_anchor_pos()
    DebugDraw3D.draw_line(global_pivot, anchor, Color.RED)

