extends Node2D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
@export var mass: float = 1.0

var springs: Array[Node2D]
var velocity: Vector2

func total_force() -> Vector2:
	var force: Vector2 = Vector2(0, 0)
	for spring in springs:
		force += spring.force(get_parent())
	return force

func solve_explicit_euler(delta: float) -> void:
	var parent = get_parent()
	var force: Vector2 = total_force()
	parent.global_position += velocity * delta
	velocity += force / mass * delta

func solve_implicit_euler(delta: float) -> void:
	var parent = get_parent()
	var original_position = parent.global_position
	var original_velocity = velocity
	
	var predicted_position = original_position + original_velocity * delta
	parent.global_position = predicted_position
	var force_at_predicted = total_force()
	
	velocity = original_velocity + (force_at_predicted / mass) * delta
	var new_position = original_position + velocity * delta
	parent.global_position = new_position

func solve_symplectic_euler(delta: float) -> void:
	var parent = get_parent()
	var force: Vector2 = total_force()
	velocity += force / mass * delta
	parent.global_position += velocity * delta

func solve_velocity_verlet(delta: float) -> void:
	var parent = get_parent()
	var current_position = parent.global_position
	var current_velocity = velocity
	var current_force = total_force()
	var current_acceleration = current_force / mass
	
	parent.global_position = current_position + current_velocity * delta + 0.5 * current_acceleration * delta * delta
	
	var new_force = total_force()
	var new_acceleration = new_force / mass
	velocity = current_velocity + 0.5 * (current_acceleration + new_acceleration) * delta

func _process(delta: float) -> void:
	var root = get_tree().current_scene
	var variant = root.get_meta("spring_solver_mode")
	match variant:
		0: solve_explicit_euler(delta)
		1: solve_implicit_euler(delta)
		2: solve_symplectic_euler(delta)
		3: solve_velocity_verlet(delta)

func get_energy() -> float:
	return velocity.length_squared() * mass * 0.5
