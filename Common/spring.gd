@tool
extends Line2D

@export var coefficient: float = 1

@export var target_a: Node2D
@export var target_b: Node2D
@export var straight_length: float = 30.0   # Increased default for better visibility
@export var amplitude: float = 15.0
@export var coils: int = 6
@export var color: Color = Color.WHITE
@export var show_preview_in_editor: bool = true

@export var target_child_script: Script
@export var target_variable_name: String = "springs"

var initial_length: float = 0

func _ready():
	antialiased = true
	update_references()

func _process(_delta):
	default_color = color

	# Determine endpoints in local coordinates
	var a_local: Vector2
	var b_local: Vector2

	if Engine.is_editor_hint():
		if target_a and target_b and target_a.is_inside_tree() and target_b.is_inside_tree():
			a_local = to_local(target_a.global_position)
			b_local = to_local(target_b.global_position)
		elif show_preview_in_editor:
			# Use a larger preview so it's clearly visible
			a_local = Vector2(-100, 0)
			b_local = Vector2(100, 0)
		else:
			points = []
			return
	else:
		# ----- GAME MODE -----
		if not target_a or not target_b or not target_a.is_inside_tree() or not target_b.is_inside_tree():
			points = []
			return
		a_local = to_local(target_a.global_position)
		b_local = to_local(target_b.global_position)

	# Compute the spring
	var d = b_local - a_local
	var dist = d.length()
	if dist == 0:
		points = [a_local, b_local]
		return

	var dir = d.normalized()
	var perp = Vector2(-dir.y, dir.x) * amplitude

	# If too short, draw a straight line
	if dist <= 2.0 * straight_length:
		points = [a_local, b_local]
		return

	var zig_start = a_local + dir * straight_length
	var zig_end   = b_local - dir * straight_length
	var zig_vec = zig_end - zig_start
	var zig_dist = zig_vec.length()

	if zig_dist == 0 or coils < 1:
		points = [a_local, zig_start, b_local]
		return

	# Build zigzag
	var zig_points_count = 2 * coils + 1
	var zig_points = []
	for i in range(zig_points_count):
		var t = float(i) / (zig_points_count - 1)
		var pos_on_axis = zig_start + zig_vec * t
		var offset = 0.0
		if i > 0 and i < zig_points_count - 1:
			offset = amplitude if (i % 2 == 1) else -amplitude
		zig_points.append(pos_on_axis + perp * offset)

	points = [a_local] + zig_points + [b_local]
	
func update_references():
	if not target_child_script:
		return  # No script specified, nothing to do

	if target_variable_name.is_empty():
		return

	# Process both targets
	_update_references_for_target(target_a)
	_update_references_for_target(target_b)
	
	initial_length = target_a.global_position.distance_to(target_b.global_position)

func force(target: Node2D) -> Vector2:
	var arm: Vector2 = target_a.global_position - target_b.global_position
	var force = arm.normalized() * coefficient * (initial_length - arm.length())
	return force if target == target_a else -force

func _update_references_for_target(target: Node2D):
	if not target:
		return

	# Look for a child node that has the specified script
	for child in target.get_children():
		if child.get_script() == target_child_script:
			# Set the variable on that child to reference this Line2D
			child[target_variable_name].append(self)
			break

func get_energy() -> float:
	return (target_a.global_position.distance_to(target_b.global_position) - initial_length) ** 2 * coefficient / 2
