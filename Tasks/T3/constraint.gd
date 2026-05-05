@abstract
class_name Constraint
extends Object

var lambda : float = 0

@abstract
func get_constraint_value(node : PhysicalBox) -> float

@abstract
func get_positional_gradient(node : PhysicalBox) -> Vector3

@abstract
func get_angular_gradient(node : PhysicalBox) -> Vector3

func get_compliance() -> float:
    return 0.001

func get_delta_lambda(node : PhysicalBox, delta : float) -> float:
    var a_tilde := get_compliance() / (delta * delta)
    var top := -get_constraint_value(node) - a_tilde * lambda
    var angular_grad := get_angular_gradient(node)
    var positional_grad := get_positional_gradient(node)
    var angular_inertia_inverse := node.get_inverse_inertia_ws()
    var gradient_scalar := positional_grad.length_squared() / node.mass + angular_grad.dot(angular_inertia_inverse * angular_grad)
    var bottom := gradient_scalar + a_tilde
    return top / bottom

class CompressedDelta:
    var positional : Vector3 = Vector3.ZERO
    var angular : Vector3 = Vector3.ZERO

func get_delta_and_update_lambda(node : PhysicalBox, delta : float) -> CompressedDelta:
    var delta_lambda := get_delta_lambda(node, delta)
    var compressed_delta := CompressedDelta.new()
    compressed_delta.positional = get_positional_gradient(node) / node.mass * delta_lambda
    compressed_delta.angular = node.get_inverse_inertia_ws() * get_angular_gradient(node) * delta_lambda
    lambda += delta_lambda
    return compressed_delta
