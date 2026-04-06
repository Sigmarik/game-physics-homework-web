extends CollisionShape2D

## Mass of the ball (kg)
@export var mass: float = 1.0

## Previous position before the last integration step.
## The solver will update this after moving the ball.
var previous_position: Vector2

## Array of springs attached to this ball.
## Each spring should be a reference to a node or custom object
## that implements an `apply_constraint()` method.
var springs: Array = []


func _ready() -> void:
	# Initialize previous_position to the starting position
	previous_position = position


## Adds a spring to the ball.
## @param spring: The spring node or constraint object.
func add_spring(spring: Node) -> void:
	springs.append(spring)


## Removes a spring from the ball.
## @param spring: The spring to remove.
func remove_spring(spring: Node) -> void:
	springs.erase(spring)
