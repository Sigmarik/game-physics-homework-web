class_name SpringConstraint
extends Constraint

var relative_shift: Vector3 = Vector3.ZERO
var anchor_position: Vector3 = Vector3.ZERO
var anchor_object: Node3D = null

func get_anchor_pos() -> Vector3:
    return anchor_object.global_transform * anchor_position if anchor_object else anchor_position

func get_constraint_value(node: PhysicalBox) -> float:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    return global_pivot.distance_squared_to(get_anchor_pos())

func get_positional_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var delta := get_anchor_pos() - global_pivot
    return -delta

func get_angular_gradient(node: PhysicalBox) -> Vector3:
    var pivot_shift := node.global_basis * relative_shift
    var global_pivot := node.global_position + pivot_shift
    var delta := get_anchor_pos() - global_pivot
    return -pivot_shift.cross(delta)